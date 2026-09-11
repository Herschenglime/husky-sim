#!/usr/bin/env python3
# Copyright 2026.
"""Readiness gate for A200 simulation and navigation bringup.

Implements sequential polling gates using rclpy.spin_once() to guarantee
clean, deterministic bringup across two distinct launch stages:

Stage 1 ('sim'):
1. /clock is advancing (Gazebo physics and clock bridge active).
2. 2D lidar is publishing scans on sensors/lidar2d_0/scan_filtered (or raw scan).
3. Controller manager has joint_state_broadcaster and platform_velocity_controller
   in state 'active'. Automatically loads, configures, and activates via services.
4. Transform from 'odom' to 'base_link' is resolvable in TF.

Stage 2 ('map'):
1. If slam:=true, verifies slam_toolbox lifecycle node is active (re-triggers
   configure/activate if stranded by heavy world startup).
2. Map is actively published on /{ns}/map (transient local QoS).
3. If slam:=false (localization), publishes seed pose to /{ns}/initialpose.

Exits with code 0 once all gates for the requested stage pass, or code 1 on timeout.
"""

import math
import sys
import time

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ReadinessGate(Node):

    def __init__(self):
        super().__init__('readiness_gate')

        self.declare_parameter('namespace', 'a200_0000')
        self.declare_parameter('stage', 'sim')
        self.declare_parameter('timeout_sec', 120.0)
        self.declare_parameter('required_controllers',
                               ['joint_state_broadcaster', 'platform_velocity_controller'])
        self.declare_parameter('slam', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.ns = self.get_parameter('namespace').value.strip('/')
        self.stage = self.get_parameter('stage').value.strip().lower()
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.required_controllers = set(self.get_parameter('required_controllers').value)
        self.slam = bool(self.get_parameter('slam').value)

        self.get_logger().info(
            f'Starting readiness gate [stage: {self.stage}] for namespace "{self.ns}" '
            f'(timeout: {self.timeout_sec:.0f}s)...'
        )

    def run(self) -> bool:
        if self.stage == 'sim':
            return self._run_sim_stage()
        elif self.stage == 'map':
            return self._run_map_stage()
        else:
            self.get_logger().error(f'Unknown readiness stage "{self.stage}". Expected "sim" or "map".')
            return False

    def _run_sim_stage(self) -> bool:
        start_time = time.monotonic()

        # Gate 1: /clock advancing
        self.get_logger().info('Gate 1/4: Waiting for /clock to advance...')
        clock_ticks = 0
        last_sim_time = None

        def clock_cb(msg: Clock):
            nonlocal clock_ticks, last_sim_time
            curr = msg.clock.sec + msg.clock.nanosec * 1e-9
            if last_sim_time is not None and curr > last_sim_time:
                clock_ticks += 1
            last_sim_time = curr

        sub_clock = self.create_subscription(Clock, '/clock', clock_cb, 10)

        while rclpy.ok() and clock_ticks < 3:
            if time.monotonic() - start_time > self.timeout_sec:
                self.get_logger().error('Gate 1/4 FAILED: Timed out waiting for /clock!')
                return False
            rclpy.spin_once(self, timeout_sec=0.2)

        self.destroy_subscription(sub_clock)
        self.get_logger().info('Gate 1/4 passed: simulation clock active.')

        # Gate 2: Lidar publishing
        self.get_logger().info('Gate 2/4: Waiting for lidar scan...')
        lidar_ready = False

        def scan_cb(msg: LaserScan):
            nonlocal lidar_ready
            if len(msg.ranges) > 0:
                lidar_ready = True

        filtered_topic = f'/{self.ns}/sensors/lidar2d_0/scan_filtered' if self.ns else '/sensors/lidar2d_0/scan_filtered'
        raw_topic = f'/{self.ns}/sensors/lidar2d_0/scan' if self.ns else '/sensors/lidar2d_0/scan'

        sub_filtered = self.create_subscription(LaserScan, filtered_topic, scan_cb, qos_profile_sensor_data)
        sub_raw = self.create_subscription(LaserScan, raw_topic, scan_cb, qos_profile_sensor_data)

        while rclpy.ok() and not lidar_ready:
            if time.monotonic() - start_time > self.timeout_sec:
                self.get_logger().error('Gate 2/4 FAILED: Timed out waiting for lidar scan!')
                return False
            rclpy.spin_once(self, timeout_sec=0.2)

        self.destroy_subscription(sub_filtered)
        self.destroy_subscription(sub_raw)
        self.get_logger().info('Gate 2/4 passed: lidar scan active.')

        # Gate 3: Controllers active
        if not self._check_controllers_gate(start_time):
            return False

        # Gate 4: Odom transform available
        self.get_logger().info('Gate 4/4: Waiting for odom -> base_link transform...')
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self)
        odom_ready = False

        while rclpy.ok() and not odom_ready:
            if time.monotonic() - start_time > self.timeout_sec:
                self.get_logger().error('Gate 4/4 FAILED: Timed out waiting for odom -> base_link transform!')
                return False

            try:
                if tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
                    odom_ready = True
                    break
            except Exception:
                pass

            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info('Gate 4/4 passed: odom -> base_link transform active.')
        self.get_logger().info('=== [Stage 1/2: Sim Gate] Simulation, controllers, and odometry verified! ===')
        return True

    def _run_map_stage(self) -> bool:
        start_time = time.monotonic()
        map_topic = f'/{self.ns}/map' if self.ns else '/map'
        self.get_logger().info(f'Waiting for map on topic "{map_topic}"...')

        map_received = False

        def map_cb(msg: OccupancyGrid):
            nonlocal map_received
            if msg.info.width > 0 and msg.info.height > 0:
                map_received = True

        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        sub_map = self.create_subscription(OccupancyGrid, map_topic, map_cb, map_qos)

        last_lifecycle_check = 0.0

        while rclpy.ok() and not map_received:
            if time.monotonic() - start_time > self.timeout_sec:
                self.get_logger().error('Stage 2 FAILED: Timed out waiting for map topic!')
                return False

            # Monitor and recover lifecycle nodes if stranded
            now = time.monotonic()
            if now - last_lifecycle_check > 5.0:
                last_lifecycle_check = now
                if self.slam:
                    self._check_slam_lifecycle()
                else:
                    self._check_localization_lifecycle()

            rclpy.spin_once(self, timeout_sec=0.2)

        self.destroy_subscription(sub_map)
        self.get_logger().info('Map successfully received!')

        # If localization mode (AMCL), publish seed initialpose at spawn and wait for TF
        if not self.slam:
            init_x = float(self.get_parameter('initial_x').value)
            init_y = float(self.get_parameter('initial_y').value)
            init_yaw = float(self.get_parameter('initial_yaw').value)
            self.get_logger().info(
                f'Seeding initial pose at spawn ({init_x:.3f}, {init_y:.3f}, yaw={init_yaw:.3f} rad)...'
            )
            init_topic = f'/{self.ns}/initialpose' if self.ns else '/initialpose'
            init_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=5
            )
            pub_init = self.create_publisher(PoseWithCovarianceStamped, init_topic, init_qos)

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = init_x
            msg.pose.pose.position.y = init_y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.z = math.sin(init_yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(init_yaw / 2.0)
            # Covariance matrix (diagonal entries for x, y, yaw)
            msg.pose.covariance[0] = 0.25   # x variance
            msg.pose.covariance[7] = 0.25   # y variance
            msg.pose.covariance[35] = 0.068 # yaw variance (~15 deg std dev)

            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, self)

            loc_ready = False
            last_pub = 0.0
            self.get_logger().info('Waiting for AMCL to establish map -> odom transform...')

            while rclpy.ok() and not loc_ready:
                if time.monotonic() - start_time > self.timeout_sec:
                    self.get_logger().error('Stage 2 FAILED: Timed out waiting for map -> odom transform from AMCL!')
                    self.destroy_publisher(pub_init)
                    return False

                now = time.monotonic()
                if now - last_pub > 1.0:
                    last_pub = now
                    msg.header.stamp = self.get_clock().now().to_msg()
                    pub_init.publish(msg)

                try:
                    if tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
                        loc_ready = True
                        break
                except Exception:
                    pass

                rclpy.spin_once(self, timeout_sec=0.2)

            self.destroy_publisher(pub_init)
            self.get_logger().info('AMCL map -> odom transform verified!')

        self.get_logger().info('=== [Stage 2/2: Map Gate] Map & localization verified! ===')
        return True

    def _manage_node_lifecycle(self, node_name: str):
        """Query lifecycle state via service and activate if unconfigured or inactive."""
        get_srv = f'{node_name}/get_state'
        change_srv = f'{node_name}/change_state'

        if not hasattr(self, '_lifecycle_clients'):
            self._lifecycle_clients = {}

        if get_srv not in self._lifecycle_clients:
            self._lifecycle_clients[get_srv] = self.create_client(GetState, get_srv)
        if change_srv not in self._lifecycle_clients:
            self._lifecycle_clients[change_srv] = self.create_client(ChangeState, change_srv)

        client_get = self._lifecycle_clients[get_srv]
        client_change = self._lifecycle_clients[change_srv]

        if not client_get.service_is_ready():
            return

        try:
            req = GetState.Request()
            future = client_get.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            if not future.done() or future.result() is None:
                return

            current_label = future.result().current_state.label
            if current_label == 'unconfigured':
                self.get_logger().warn(
                    f'{node_name} is unconfigured; triggering configure and activate...'
                )
                if client_change.wait_for_service(timeout_sec=1.0):
                    req_c = ChangeState.Request()
                    req_c.transition.id = Transition.TRANSITION_CONFIGURE
                    f_c = client_change.call_async(req_c)
                    rclpy.spin_until_future_complete(self, f_c, timeout_sec=1.0)

                    req_a = ChangeState.Request()
                    req_a.transition.id = Transition.TRANSITION_ACTIVATE
                    f_a = client_change.call_async(req_a)
                    rclpy.spin_until_future_complete(self, f_a, timeout_sec=1.0)
            elif current_label == 'inactive':
                self.get_logger().warn(
                    f'{node_name} is inactive; triggering activate...'
                )
                if client_change.wait_for_service(timeout_sec=1.0):
                    req_a = ChangeState.Request()
                    req_a.transition.id = Transition.TRANSITION_ACTIVATE
                    f_a = client_change.call_async(req_a)
                    rclpy.spin_until_future_complete(self, f_a, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().debug(f'Lifecycle service check failed on {node_name}: {e}')

    def _check_slam_lifecycle(self):
        """Check slam_toolbox lifecycle state and transition if stuck in unconfigured/inactive."""
        node_name = f'/{self.ns}/slam_toolbox' if self.ns else '/slam_toolbox'
        self._manage_node_lifecycle(node_name)

    def _check_localization_lifecycle(self):
        """Check map_server and amcl lifecycle states and transition if stranded."""
        nodes = ['map_server', 'amcl']
        for n in nodes:
            node_name = f'/{self.ns}/{n}' if self.ns else f'/{n}'
            self._manage_node_lifecycle(node_name)

    def _check_controllers_gate(self, start_time: float) -> bool:
        """Gate 3/4: Verify required controllers are active, recovering via service calls."""
        self.get_logger().info('Gate 3/4: Waiting for controllers to become active...')
        cm_path = f'/{self.ns}/controller_manager' if self.ns else '/controller_manager'
        cm_client = self._get_cm_client(cm_path, ListControllers, 'list_controllers')

        controllers_ready = False
        last_spawn_time = 0.0

        while rclpy.ok() and not controllers_ready:
            if time.monotonic() - start_time > self.timeout_sec:
                self.get_logger().error('Gate 3/4 FAILED: Timed out waiting for controllers!')
                return False

            if not cm_client.service_is_ready():
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

            req = ListControllers.Request()
            res = self._call_service_sync(cm_client, req, timeout_sec=1.0)
            if res is not None:
                loaded_states = {c.name: c.state for c in res.controller}
                active = {name for name, state in loaded_states.items() if state == 'active'}
                missing = self.required_controllers - active
                if not missing:
                    controllers_ready = True
                    break

                now = time.monotonic()
                if now - last_spawn_time > 5.0:
                    last_spawn_time = now
                    self.get_logger().warn(
                        f'Controllers not active: {sorted(missing)}. Recovering via services...'
                    )
                    self._ensure_controllers(cm_path, missing, loaded_states)

            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Gate 3/4 passed: controllers active.')
        return True

    def _call_service_sync(self, client, request, timeout_sec: float = 2.0):
        """Asynchronously invoke service and spin until complete or timeout."""
        try:
            if not client.service_is_ready():
                if not client.wait_for_service(timeout_sec=timeout_sec):
                    return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                return future.result()
        except Exception as e:
            name = getattr(client, 'service_name', 'unknown')
            self.get_logger().debug(f'Service call failed on {name}: {e}')
        return None

    def _get_cm_client(self, cm_path: str, srv_type, srv_suffix: str):
        if not hasattr(self, '_cm_clients'):
            self._cm_clients = {}
        srv_name = f'{cm_path}/{srv_suffix}'
        if srv_name not in self._cm_clients:
            self._cm_clients[srv_name] = self.create_client(srv_type, srv_name)
        return self._cm_clients[srv_name]

    def _ensure_controllers(
        self,
        cm_path: str,
        missing: set[str],
        loaded_controllers: dict[str, str]
    ):
        """Load, configure, and activate missing controllers via controller_manager services."""
        to_activate = []

        load_client = self._get_cm_client(cm_path, LoadController, 'load_controller')
        config_client = self._get_cm_client(cm_path, ConfigureController, 'configure_controller')
        switch_client = self._get_cm_client(cm_path, SwitchController, 'switch_controller')

        for name in sorted(missing):
            curr_state = loaded_controllers.get(name)

            if curr_state is None:
                self.get_logger().info(f'Loading controller {name}...')
                req_load = LoadController.Request()
                req_load.name = name
                res_load = self._call_service_sync(load_client, req_load, timeout_sec=2.0)
                if res_load is None or not res_load.ok:
                    self.get_logger().warn(f'Failed to load controller {name}')
                    continue
                curr_state = 'unconfigured'

            if curr_state == 'unconfigured':
                self.get_logger().info(f'Configuring controller {name}...')
                req_cfg = ConfigureController.Request()
                req_cfg.name = name
                res_cfg = self._call_service_sync(config_client, req_cfg, timeout_sec=2.0)
                if res_cfg is None or not res_cfg.ok:
                    self.get_logger().warn(f'Failed to configure controller {name}')
                    continue
                curr_state = 'inactive'

            if curr_state == 'inactive':
                to_activate.append(name)

        if to_activate:
            self.get_logger().info(f'Activating controllers: {to_activate}...')
            req_switch = SwitchController.Request()
            req_switch.activate_controllers = to_activate
            req_switch.deactivate_controllers = []
            req_switch.strictness = SwitchController.Request.BEST_EFFORT
            req_switch.activate_asap = True
            req_switch.timeout.sec = 2
            req_switch.timeout.nanosec = 0

            res_switch = self._call_service_sync(switch_client, req_switch, timeout_sec=3.0)
            if res_switch is None or not res_switch.ok:
                msg = res_switch.message if res_switch else 'timeout'
                self.get_logger().warn(f'SwitchController failed: {msg}')
            else:
                self.get_logger().info(f'Successfully activated controllers: {to_activate}')

    def destroy_node(self):
        if hasattr(self, '_cm_clients'):
            for client in self._cm_clients.values():
                self.destroy_client(client)
            self._cm_clients.clear()
        if hasattr(self, '_lifecycle_clients'):
            for client in self._lifecycle_clients.values():
                self.destroy_client(client)
            self._lifecycle_clients.clear()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReadinessGate()
    success = False
    try:
        success = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
import argparse
import json
import math
import sys
import os


class StateLogger(LifecycleNode):
    """Managed lifecycle node for logging synchronized robot state and scans to JSONL.

    Lifecycle States:
    - Unconfigured: parameters declared, internal buffers empty, subscriptions inactive.
    - Inactive: subscriptions and TF listeners active; output file closed; no disk writes.
    - Active: output file opened at configured path; writing state entries on each scan.
    - Finalized: file flushed and closed, subscriptions and TF listeners cleaned up.
    """

    def __init__(self, node_name: str = 'state_logger', **kwargs):
        super().__init__(node_name)

        # Declare parameters with sensible defaults or kwargs overrides
        self.declare_parameter('odom_topic', kwargs.get('odom_topic', '/odom'))
        self.declare_parameter('cmd_vel_topic', kwargs.get('cmd_vel_topic', '/cmd_vel'))
        self.declare_parameter('scan_topic', kwargs.get('scan_topic', '/scan'))
        self.declare_parameter('target_frame', kwargs.get('target_frame', 'map'))
        self.declare_parameter('base_frame', kwargs.get('base_frame', 'base_link'))
        self.declare_parameter('output', kwargs.get('output_file', 'state.jsonl'))
        self.declare_parameter('stamped_cmd_vel', kwargs.get('stamped_cmd_vel', True))
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Internal state
        self.output_file = self.get_parameter('output').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.file = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self._is_active = False

        # Communication handles (created in on_configure)
        self.tf_buffer = None
        self.tf_listener = None
        self.odom_sub = None
        self.cmd_vel_sub = None
        self.scan_sub = None

        # Allow dynamic updates to output file when Inactive
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().info("StateLogger initialized in Unconfigured state.")

    @property
    def current_state(self) -> str:
        """Return the current lifecycle state label."""
        return self._state_machine.current_state[1]

    def _on_set_parameters(self, params):
        for p in params:
            if p.name in ('output', 'target_frame', 'base_frame'):
                if self._is_active:
                    self.get_logger().warn(f"Cannot change '{p.name}' parameter while node is Active. Deactivate first.")
                    return SetParametersResult(successful=False, reason="Node is Active")
                if p.name == 'output':
                    self.output_file = p.value
                    self.get_logger().info(f"Target output file updated to: {self.output_file}")
                elif p.name == 'target_frame':
                    self.target_frame = p.value
                elif p.name == 'base_frame':
                    self.base_frame = p.value
        return SetParametersResult(successful=True)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring StateLogger: binding topics and initializing TF buffer...")
        self.output_file = self.get_parameter('output').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        stamped_cmd_vel = self.get_parameter('stamped_cmd_vel').value

        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10
        )
        cmd_vel_msg_type = TwistStamped if stamped_cmd_vel else Twist
        self.cmd_vel_sub = self.create_subscription(
            cmd_vel_msg_type, cmd_vel_topic, self.cmd_vel_cb, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data
        )

        self.get_logger().info(f"Subscriptions created: odom={odom_topic}, cmd_vel={cmd_vel_topic}, scan={scan_topic}")
        self.get_logger().info(f"Tracking transform: {self.target_frame} -> {self.base_frame}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.output_file = self.get_parameter('output').value
        self.get_logger().info(f"Activating StateLogger: opening output file {self.output_file}")
        try:
            out_dir = os.path.dirname(os.path.abspath(self.output_file))
            os.makedirs(out_dir, exist_ok=True)
            self.file = open(self.output_file, 'w')
        except Exception as e:
            self.get_logger().error(f"Failed to open output file {self.output_file}: {e}")
            return TransitionCallbackReturn.FAILURE

        ret = super().on_activate(state)
        if ret == TransitionCallbackReturn.SUCCESS:
            self._is_active = True
            return ret
        else:
            self._close_file()
            return ret

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating StateLogger: flushing and closing file...")
        self._is_active = False
        self._close_file()
        return super().on_deactivate(state)

    def _release_resources(self):
        self._is_active = False
        self._close_file()

        if self.tf_listener is not None:
            try:
                self.tf_listener.unregister()
            except Exception:
                pass
            self.tf_listener = None
        self.tf_buffer = None

        if self.odom_sub:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub = None
        if self.cmd_vel_sub:
            self.destroy_subscription(self.cmd_vel_sub)
            self.cmd_vel_sub = None
        if self.scan_sub:
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None

        self.latest_odom = None
        self.latest_cmd_vel = None

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up StateLogger: destroying subscriptions and TF...")
        self._release_resources()
        return super().on_cleanup(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down StateLogger...")
        self._release_resources()
        return super().on_shutdown(state)

    def _close_file(self):
        if self.file is not None and not self.file.closed:
            try:
                self.file.flush()
                self.file.close()
                self.get_logger().info(f"Closed output file {self.output_file}")
            except Exception as e:
                self.get_logger().warn(f"Error closing file {self.output_file}: {e}")
            finally:
                self.file = None

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def cmd_vel_cb(self, msg):
        if hasattr(msg, 'twist'):
            self.latest_cmd_vel = msg.twist
        else:
            self.latest_cmd_vel = msg

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg: LaserScan):
        # Drive logging from scan callback to avoid duplicate entries
        # Only log data when in Active lifecycle state
        if not self._is_active or self.file is None or self.file.closed or self.latest_odom is None:
            return

        # Time
        stamp = msg.header.stamp
        time_sec = stamp.sec + stamp.nanosec * 1e-9

        # Odom state (fallback & diagnostics)
        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        odom_yaw = self.get_yaw_from_quaternion(q)

        # Resolve position in target frame (e.g. map) if available
        x = pos.x
        y = pos.y
        yaw = odom_yaw

        if self.tf_buffer is not None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                yaw = self.get_yaw_from_quaternion(t.transform.rotation)
            except Exception:
                # Try with namespace prefix on base_frame if root lookup failed
                try:
                    ns_prefix = self.get_namespace().strip('/')
                    if ns_prefix and not self.base_frame.startswith(ns_prefix):
                        alt_base = f"{ns_prefix}/{self.base_frame}"
                        t = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            alt_base,
                            rclpy.time.Time()
                        )
                        x = t.transform.translation.x
                        y = t.transform.translation.y
                        yaw = self.get_yaw_from_quaternion(t.transform.rotation)
                except Exception:
                    pass

        # Cmd vel state
        if self.latest_cmd_vel is not None:
            vx = self.latest_cmd_vel.linear.x
            wz = self.latest_cmd_vel.angular.z
        else:
            vx = 0.0
            wz = 0.0

        # JSON compliant ranges
        safe_ranges = [
            r if not math.isinf(r) and not math.isnan(r) else None 
            for r in msg.ranges
        ]

        # Construct JSON with map frame and raw odom
        data = {
            "timestamp": time_sec,
            "x": x,
            "y": y,
            "yaw": yaw,
            "odom_x": pos.x,
            "odom_y": pos.y,
            "odom_yaw": odom_yaw,
            "vx": vx,
            "wz": wz,
            "scan": safe_ranges
        }

        # Write to JSONL
        self.file.write(json.dumps(data) + '\n')
        self.file.flush()

    def destroy_node(self):
        self._is_active = False
        self._close_file()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description="Log robot state and lidar to JSONL (Lifecycle Managed)")
    parser.add_argument('--node_name', type=str, default='state_logger', help='Node name')
    parser.add_argument('--odom_topic', type=str, default='/odom', help='Odometry topic')
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel', help='Command velocity topic')
    parser.add_argument('--scan_topic', type=str, default='/scan', help='LaserScan topic')
    parser.add_argument('--target_frame', type=str, default='map', help='Global target frame for x, y, yaw')
    parser.add_argument('--base_frame', type=str, default='base_link', help='Robot base frame to resolve')
    parser.add_argument('--output', type=str, default='state.jsonl', help='Output JSONL file')
    parser.add_argument('--unstamped_cmd_vel', action='store_true', help='Subscribe to Twist instead of TwistStamped')
    parser.add_argument('--autostart', dest='autostart', action='store_true', default=True,
                        help='Automatically configure and activate on start (default: True for standalone CLI)')
    parser.add_argument('--no-autostart', dest='autostart', action='store_false',
                        help='Stay in Unconfigured state awaiting lifecycle manager')

    parsed_args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args or sys.argv)

    node = StateLogger(
        node_name=parsed_args.node_name,
        odom_topic=parsed_args.odom_topic,
        cmd_vel_topic=parsed_args.cmd_vel_topic,
        scan_topic=parsed_args.scan_topic,
        output_file=parsed_args.output,
        target_frame=parsed_args.target_frame,
        base_frame=parsed_args.base_frame,
        stamped_cmd_vel=not parsed_args.unstamped_cmd_vel
    )

    if parsed_args.autostart:
        node.get_logger().info("Autostart enabled: configuring and activating...")
        cfg_ret = node.trigger_configure()
        if cfg_ret == TransitionCallbackReturn.SUCCESS:
            node.trigger_activate()
        else:
            node.get_logger().error("Autostart configuration failed; skipping activation.")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

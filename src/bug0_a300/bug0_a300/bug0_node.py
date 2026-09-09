#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


CONTROL_RATE_HZ = 10.0

# Clearpath A300 Observer (~0.99 x 0.67 m footprint). The scan this node consumes is
# the 3D lidar flattened into a LaserScan in base_link (see bringup.launch.py), so
# every distance below is measured from the chassis centre and includes the
# robot's own half-length / half-width.
GOAL_TOLERANCE = 0.25
GOAL_LINEAR_SPEED = 0.50
GOAL_APPROACH_GAIN = 0.5
GOAL_APPROACH_MIN_SPEED = 0.10
WALL_LINEAR_SPEED = 0.35
MAX_ANGULAR_SPEED = 0.80
HEADING_KP = 1.5

ROBOT_RADIUS = 0.55
CLEARANCE_MARGIN = 0.15
CORRIDOR_HALF_WIDTH = ROBOT_RADIUS + CLEARANCE_MARGIN
GOAL_HEADING_CLEAR_DISTANCE = 2.0
CORRIDOR_NEAR_IGNORE = 0.05

FRONT_STOP_DISTANCE = 0.85
FRONT_SECTOR_DEG = 35.0
FRONT_CLEAR_DISTANCE = 2.0

SIDE_CHOICE_CENTER_DEG = 40.0
SIDE_CHOICE_HALF_DEG = 40.0

DESIRED_WALL_DISTANCE = 0.70
WALL_DIST_KP = 1.6
WALL_ALIGN_KP = 1.1
WALL_BEAM_HALF_DEG = 8.0
WALL_LOST_DISTANCE = 2.4
# Turn radius while re-acquiring a lost wall is ~0.5 m (one robot radius, as on
# the TurtleBot4); slower turning made the A300 loop away from the obstacle.
REACQUIRE_WALL_TURN_SPEED = 0.50

MIN_WALL_FOLLOW_TIME = 0.0
MIN_DISTANCE_FROM_HIT_POINT = 0.60
MIN_GOAL_PROGRESS_TO_LEAVE = 0.40

HEADING_TOLERANCE_DEG = 5.0
# The flattened scan is already expressed in base_link, so no sensor yaw offset.
DEFAULT_LIDAR_YAW_OFFSET_DEG = 0.0
DEBUG_LOG_PERIOD = 0.5

Pose2D = Tuple[float, float, float]


class BugState(Enum):
    GO_TO_GOAL = 1
    WALL_FOLLOW = 2
    DONE = 3


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Bug0Node(Node):
    """
    Bug0 controller using the same pose idea that Nav2 uses.

    It does not subscribe to /amcl_pose. Instead it reads the TF transform:

        map -> base_link     or     map -> base_footprint

    That transform is already the composition of:

        map -> odom          from AMCL or SLAM localization
        odom -> base_link    from wheel odometry / robot_localization

    Therefore the pose is globally corrected by localization and still smooth
    between localization updates.

    A300 port: topics are relative so the node runs inside the robot namespace
    (e.g. /a300_00000) with /tf and /tf_static remapped into it. The scan is the
    3D lidar (sensors/lidar3d_1/points) flattened to a LaserScan in base_link by
    pointcloud_to_laserscan; see launch/bringup.launch.py.
    """

    def __init__(self):
        super().__init__('bug0_node')

        self.declare_parameter('goal_x', -7.0)
        self.declare_parameter('goal_y', -1.0)
        self.declare_parameter('wall_side', 'auto')
        self.declare_parameter('scan_topic', 'sensors/lidar3d_1/scan_2d')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('use_twist_stamped', True)
        self.declare_parameter('lidar_yaw_offset_deg', DEFAULT_LIDAR_YAW_OFFSET_DEG)
        self.declare_parameter('require_progress_to_leave', True)
        self.declare_parameter('force_goal_heading_before_wall_follow', True)

        # Nav2-style pose source. Use base_footprint if that is your Nav2 robot_base_frame.
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.10)

        self.declare_parameter('debug', True)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.wall_side_param = str(self.get_parameter('wall_side').value).lower()
        self.use_twist_stamped = bool(self.get_parameter('use_twist_stamped').value)
        self.lidar_yaw_offset = math.radians(float(self.get_parameter('lidar_yaw_offset_deg').value))
        self.require_progress = bool(self.get_parameter('require_progress_to_leave').value)
        self.force_goal_heading_before_wall_follow = bool(
            self.get_parameter('force_goal_heading_before_wall_follow').value
        )
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        self.debug = bool(self.get_parameter('debug').value)

        if self.wall_side_param not in ('left', 'right', 'auto'):
            self.get_logger().warn("wall_side must be 'left', 'right', or 'auto'. Using 'auto'.")
            self.wall_side_param = 'auto'

        scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan: Optional[LaserScan] = None
        self.pose: Optional[Pose2D] = None

        self.state = BugState.GO_TO_GOAL
        self.active_wall_side = 'left'
        self.wall_start_time = None
        self.wall_hit_pose: Optional[Tuple[float, float]] = None
        self.wall_hit_goal_distance = float('inf')

        self.last_cmd: Tuple[float, float] = (0.0, 0.0)
        self.last_debug_time = 0.0
        self.last_wait_log_time = 0.0
        self.scan_info_logged = False
        self.tf_info_logged = False
        self.tf_missing_warned = False
        self.dbg_wall = ''

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)

        if self.use_twist_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
            cmd_type = 'TwistStamped'
        else:
            self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            cmd_type = 'Twist'

        self.timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self.control_loop)

        self.get_logger().info(
            f'Bug0 started with Nav2 TF pose. '
            f'goal_{self.global_frame}=({self.goal_x:.2f}, {self.goal_y:.2f}), '
            f'tf={self.global_frame}->{self.base_frame}, '
            f'wall_side={self.wall_side_param}, cmd_type={cmd_type}, '
            f'lidar_yaw_offset={math.degrees(self.lidar_yaw_offset):.0f}deg, '
            f'goal_tolerance={GOAL_TOLERANCE:.2f}, '
            f'force_goal_heading_before_wall_follow={self.force_goal_heading_before_wall_follow}'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg
        if self.debug and not self.scan_info_logged:
            self.scan_info_logged = True
            self.get_logger().info(
                f'[BUG0] first scan n={len(msg.ranges)} '
                f'angle_min={math.degrees(msg.angle_min):.1f}deg '
                f'angle_max={math.degrees(msg.angle_max):.1f}deg '
                f'inc={math.degrees(msg.angle_increment):.3f}deg '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}] '
                f'frame={msg.header.frame_id}'
            )

    def update_pose_from_tf(self) -> None:
        """Read corrected global pose from TF: global_frame -> base_frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=self.tf_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.pose = None
            if not self.tf_missing_warned:
                self.tf_missing_warned = True
                self.get_logger().warn(
                    f'Cannot get TF {self.global_frame}->{self.base_frame}: '
                    f'{type(exc).__name__}: {exc}. '
                    f'AMCL/SLAM must publish {self.global_frame}->odom and odometry must publish odom->{self.base_frame}. '
                    f'Set the initial pose in RViz if using AMCL.'
                )
            return

        tr = t.transform.translation
        yaw = yaw_from_quaternion(t.transform.rotation)
        self.pose = (float(tr.x), float(tr.y), yaw)

        if self.tf_missing_warned:
            self.tf_missing_warned = False
            self.get_logger().info(f'TF {self.global_frame}->{self.base_frame} is available again.')

        if self.debug and not self.tf_info_logged:
            self.tf_info_logged = True
            self.get_logger().info(
                f'[BUG0] first TF pose_{self.global_frame}=({tr.x:+.2f},{tr.y:+.2f}) '
                f'yaw={math.degrees(yaw):+.0f}deg frame={self.base_frame}'
            )

    def valid_range(self, r: float) -> bool:
        if self.scan is None:
            return False
        return math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max

    def sector_min(self, center_angle: float, half_width: float) -> float:
        if self.scan is None:
            return float('inf')

        best = float('inf')
        target = wrap_to_pi(center_angle - self.lidar_yaw_offset)
        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if abs(wrap_to_pi(angle - target)) <= half_width and self.valid_range(r):
                best = min(best, float(r))
            angle += self.scan.angle_increment

        return best

    def corridor_is_clear(self, bearing: float, length: float) -> bool:
        if self.scan is None:
            return False

        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if self.valid_range(r):
                robot_angle = wrap_to_pi(angle + self.lidar_yaw_offset)
                rel = wrap_to_pi(robot_angle - bearing)
                along = r * math.cos(rel)
                lateral = r * math.sin(rel)
                if CORRIDOR_NEAR_IGNORE < along < length and abs(lateral) < CORRIDOR_HALF_WIDTH:
                    return False
            angle += self.scan.angle_increment

        return True

    def goal_info(self) -> Tuple[float, float]:
        if self.pose is None:
            return float('inf'), 0.0

        x, y, yaw = self.pose
        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = wrap_to_pi(desired_yaw - yaw)
        return distance, heading_error

    def goal_reached(self) -> bool:
        distance, _ = self.goal_info()
        return distance <= GOAL_TOLERANCE

    def goal_heading_is_free(self) -> bool:
        if self.scan is None or self.pose is None:
            return False

        goal_distance, heading_error = self.goal_info()
        required = min(goal_distance, GOAL_HEADING_CLEAR_DISTANCE)
        return self.corridor_is_clear(heading_error, required)

    def distance_from_wall_hit(self) -> float:
        if self.pose is None or self.wall_hit_pose is None:
            return 0.0
        x, y, _ = self.pose
        hx, hy = self.wall_hit_pose
        return math.hypot(x - hx, y - hy)

    def wall_follow_time_ok(self) -> bool:
        if self.wall_start_time is None:
            return False
        elapsed = (self.get_clock().now() - self.wall_start_time).nanoseconds * 1e-9
        return elapsed >= MIN_WALL_FOLLOW_TIME

    def can_leave_wall(self) -> bool:
        if not self.wall_follow_time_ok():
            return False

        if self.distance_from_wall_hit() < MIN_DISTANCE_FROM_HIT_POINT:
            return False

        if self.require_progress:
            goal_distance, _ = self.goal_info()
            if goal_distance >= self.wall_hit_goal_distance - MIN_GOAL_PROGRESS_TO_LEAVE:
                return False

        return self.goal_heading_is_free()

    def control_loop(self) -> None:
        self.update_pose_from_tf()

        if self.scan is None or self.pose is None:
            self.publish_cmd(0.0, 0.0)
            self.log_waiting_for_inputs()
            return

        if self.goal_reached():
            if self.state != BugState.DONE:
                self.get_logger().info('Goal reached. Stopping robot.')
            self.state = BugState.DONE
            self.publish_cmd(0.0, 0.0)
            return

        if self.state == BugState.GO_TO_GOAL:
            self.go_to_goal()
        elif self.state == BugState.WALL_FOLLOW:
            self.follow_wall_until_goal_heading_free()
        else:
            self.publish_cmd(0.0, 0.0)

        self.log_debug()

    def go_to_goal(self) -> None:
        distance, heading_error = self.goal_info()
        heading_tol = math.radians(HEADING_TOLERANCE_DEG)

        # First align the robot to the goal direction while staying in GO_TO_GOAL.
        # This prevents a bad initial pose near an obstacle from immediately entering
        # WALL_FOLLOW and looping around the obstacle before the robot has even faced
        # the goal. Linear velocity remains zero during this alignment step.
        if self.force_goal_heading_before_wall_follow and abs(heading_error) > heading_tol:
            angular_z = clamp(HEADING_KP * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            self.publish_cmd(0.0, angular_z)
            return

        front_min = self.sector_min(0.0, math.radians(FRONT_SECTOR_DEG))
        front_blocked = front_min < FRONT_STOP_DISTANCE

        if self.goal_heading_is_free() and not front_blocked:
            approach_speed = min(
                GOAL_LINEAR_SPEED,
                GOAL_APPROACH_GAIN * distance + GOAL_APPROACH_MIN_SPEED,
            )
            self.drive_toward_heading(heading_error, approach_speed)
        else:
            self.enter_wall_follow()

    def choose_wall_side(self) -> str:
        if self.wall_side_param in ('left', 'right'):
            return self.wall_side_param

        center = math.radians(SIDE_CHOICE_CENTER_DEG)
        half = math.radians(SIDE_CHOICE_HALF_DEG)
        front_left = self.sector_min(+center, half)
        front_right = self.sector_min(-center, half)

        if not math.isfinite(front_left) and not math.isfinite(front_right):
            return 'left'

        chosen = 'left' if front_left <= front_right else 'right'
        self.get_logger().info(
            f'auto wall side front_left={front_left:.2f} front_right={front_right:.2f} '
            f'following={chosen}'
        )
        return chosen

    def enter_wall_follow(self) -> None:
        self.state = BugState.WALL_FOLLOW
        self.wall_start_time = self.get_clock().now()
        self.active_wall_side = self.choose_wall_side()

        goal_distance, _ = self.goal_info()
        self.wall_hit_goal_distance = goal_distance

        if self.pose is not None:
            x, y, _ = self.pose
            self.wall_hit_pose = (x, y)

        self.get_logger().info(
            f'Goal corridor blocked. Following {self.active_wall_side} wall. '
            f'distance_to_goal_at_hit={goal_distance:.2f}'
        )
        self.publish_cmd(0.0, 0.0)

    def wall_measurements(self):
        beam_half = math.radians(WALL_BEAM_HALF_DEG)
        side = 1.0 if self.active_wall_side == 'left' else -1.0

        d_perp = self.sector_min(side * math.pi / 2.0, beam_half)
        d_diag = self.sector_min(side * math.pi / 4.0, beam_half)

        if not math.isfinite(d_perp):
            return float('inf'), None
        if not math.isfinite(d_diag):
            return d_perp, None

        c45 = math.cos(math.pi / 4.0)
        s45 = math.sin(math.pi / 4.0)
        wall_dx = d_diag * c45
        wall_dy = d_diag * s45 - d_perp
        phi = math.atan2(wall_dy, wall_dx)
        wall_distance = d_perp * math.cos(phi)

        return wall_distance, phi

    def follow_wall_until_goal_heading_free(self) -> None:
        if self.can_leave_wall():
            self.leave_wall_follow()
            return

        side = 1.0 if self.active_wall_side == 'left' else -1.0
        turn_away_from_front = -side * MAX_ANGULAR_SPEED
        turn_toward_wall = side * REACQUIRE_WALL_TURN_SPEED

        front_min = self.sector_min(0.0, math.radians(FRONT_SECTOR_DEG))

        if front_min < FRONT_STOP_DISTANCE:
            self.dbg_wall = f'front_block F={front_min:.2f}'
            self.publish_cmd(0.0, turn_away_from_front)
            return

        wall_distance, wall_angle = self.wall_measurements()

        if wall_distance > WALL_LOST_DISTANCE:
            self.dbg_wall = f'lost d={wall_distance:.2f}'
            self.publish_cmd(WALL_LINEAR_SPEED * 0.7, turn_toward_wall)
            return

        dist_error = wall_distance - DESIRED_WALL_DISTANCE

        if wall_angle is not None:
            angular_z = side * (WALL_DIST_KP * dist_error + WALL_ALIGN_KP * wall_angle)
            self.dbg_wall = f'd={wall_distance:.2f} a={math.degrees(wall_angle):+4.0f}'
        else:
            angular_z = side * WALL_DIST_KP * dist_error
            self.dbg_wall = f'd={wall_distance:.2f} a=--'

        if front_min < FRONT_CLEAR_DISTANCE:
            angular_z += 0.6 * turn_away_from_front

        angular_z = clamp(angular_z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        speed_scale = 1.0 - 0.5 * min(abs(angular_z) / MAX_ANGULAR_SPEED, 1.0)
        self.publish_cmd(WALL_LINEAR_SPEED * speed_scale, angular_z)

    def leave_wall_follow(self) -> None:
        self.state = BugState.GO_TO_GOAL
        self.wall_start_time = None
        self.wall_hit_pose = None
        self.wall_hit_goal_distance = float('inf')
        self.get_logger().info('Corridor to goal is clear. Going to goal.')
        self.publish_cmd(0.0, 0.0)

    def drive_toward_heading(self, heading_error: float, speed: float) -> None:
        heading_tol = math.radians(HEADING_TOLERANCE_DEG)
        angular_z = clamp(HEADING_KP * heading_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        linear_x = 0.0 if abs(heading_error) > heading_tol else speed
        self.publish_cmd(linear_x, angular_z)

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        self.last_cmd = (float(linear_x), float(angular_z))

        if self.use_twist_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)

    def log_waiting_for_inputs(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_wait_log_time < 2.0:
            return
        self.last_wait_log_time = now

        missing = []
        if self.scan is None:
            missing.append('scan')
        if self.pose is None:
            missing.append(f'TF {self.global_frame}->{self.base_frame}')

        if missing:
            self.get_logger().warn(f'Waiting for {", ".join(missing)}.')

    def log_debug(self) -> None:
        if not self.debug or self.scan is None or self.pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_debug_time < DEBUG_LOG_PERIOD:
            return
        self.last_debug_time = now

        x, y, yaw = self.pose
        dist, hdg = self.goal_info()
        front = self.sector_min(0.0, math.radians(FRONT_SECTOR_DEG))
        left = self.sector_min(math.pi / 2.0, math.radians(WALL_BEAM_HALF_DEG))
        right = self.sector_min(-math.pi / 2.0, math.radians(WALL_BEAM_HALF_DEG))
        goal_free = self.goal_heading_is_free()
        v, w = self.last_cmd

        msg = (
            f'[BUG0] {self.state.name:11s} | '
            f'pose_{self.global_frame}=({x:+.2f},{y:+.2f}) yaw={math.degrees(yaw):+4.0f} | '
            f'goal d={dist:4.2f} hdg={math.degrees(hdg):+4.0f} | '
            f'scan F={front:4.2f} L={left:4.2f} R={right:4.2f} | '
            f'corridor_free={int(goal_free)} | '
            f'cmd v={v:+.2f} w={w:+.2f}'
        )

        if self.state == BugState.WALL_FOLLOW:
            msg += (
                f' | wall[{self.active_wall_side}] {self.dbg_wall} '
                f't_ok={int(self.wall_follow_time_ok())} '
                f'dhit={self.distance_from_wall_hit():.2f} '
                f'goal_at_hit={self.wall_hit_goal_distance:.2f} '
                f'can_leave={int(self.can_leave_wall())}'
            )

        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A second SIGINT (terminal + ros2 launch both forward Ctrl-C) can land
        # while tearing down; swallow it so the node exits quietly.
        try:
            if rclpy.ok():
                node.publish_cmd(0.0, 0.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, Exception):
            pass


if __name__ == '__main__':
    main()
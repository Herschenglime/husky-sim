#!/usr/bin/env python3
# Copyright 2026.
"""Drop laser returns that land on the robot itself.

Why this exists
---------------
With the sensor arch fitted, the arch legs sit about 0.40 m from the 2D lidar,
inside the a200's 1.1 x 0.9 m footprint. 32 of 720 beams return the robot's own
structure and 22 of those fall inside the footprint polygon. Nav2's
collision_monitor runs a "FootprintApproach" check with min_points 12: it reads
those as an imminent collision and scales every command to zero. The failure is
silent and looks like a planner problem - the robot accepts a goal, plans a
valid path, publishes it, reports no error, and never moves.

Removing the arch makes it go away, which is why the navigation config used to
ship without one, but the real robot has the arch and the arch carries the
camera. Filtering the scan is the fix that keeps both.

The rule
--------
Discard any return that falls inside the robot's own footprint. That is
config-independent - it needs no hand-measured bearings, and it keeps working
if the arch, bumpers, or lidar placement change - and it is the same rule Nav2's
costmap already applies to its obstacle layer as `footprint_clearing_enabled`.

Nothing useful is lost. A return inside the footprint is either the robot, or
something the robot is already touching; in neither case does projecting it
forward in time to predict a collision mean anything. Returns outside the
footprint - every real obstacle the robot could still avoid - pass through
untouched.

Beams are marked by setting the range to +inf, which is how LaserScan spells
"nothing there": every Nav2 consumer already discards values outside
[range_min, range_max].

The test is on where the return *lands*, not on which bearings the footprint
covers. The lidar is mounted at x=0.328 m, which is inside the footprint, so
every ray leaves the sensor already within it - masking whole bearings would
mask the entire scan. Testing endpoints also keeps a real obstacle seen along a
bearing the arch happens to occupy, which is the answer you want anyway.
"""

import math

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanSelfFilter(Node):

    def __init__(self):
        super().__init__('scan_self_filter')

        self.declare_parameter('scan_in', 'sensors/lidar2d_0/scan')
        self.declare_parameter('scan_out', 'sensors/lidar2d_0/scan_filtered')
        self.declare_parameter('base_frame', 'base_link')
        # a200 footprint, matching clearpath_nav2_demos' costmaps.
        self.declare_parameter('footprint', [0.55, 0.45, 0.55, -0.45,
                                             -0.55, -0.45, -0.55, 0.45])
        self.declare_parameter('margin', 0.02)

        scan_in = self.get_parameter('scan_in').value
        scan_out = self.get_parameter('scan_out').value
        self.base_frame = self.get_parameter('base_frame').value
        self.margin = float(self.get_parameter('margin').value)

        flat = list(self.get_parameter('footprint').value)
        if len(flat) < 6 or len(flat) % 2:
            raise ValueError('footprint must be an even number of at least 3 x,y pairs')
        self.poly = np.array(flat, dtype=float).reshape(-1, 2)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # The lidar is bolted on, so its pose in base_link never changes and the
        # per-beam bearings never change: solve both once, then each scan is one
        # vectorised point-in-polygon test over 720 points.
        self.geometry = None
        self.poly_inflated = None
        self.warned = False
        self.reported = None

        self.pub = self.create_publisher(LaserScan, scan_out, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, scan_in, self.on_scan,
                                            qos_profile_sensor_data)
        self.get_logger().info(f'filtering {scan_in} -> {scan_out} '
                               f'against the {self.poly.shape[0]}-point footprint '
                               f'in {self.base_frame} (margin {self.margin} m)')

    def inflated(self):
        """Grow the polygon by `margin` about its centroid.

        A beam that grazes the very edge of the footprint is as likely to be the
        robot as not, and the consequence of keeping one is a stalled robot
        while the consequence of dropping one is nothing. So err outward.
        """
        c = self.poly.mean(axis=0)
        d = self.poly - c
        norms = np.linalg.norm(d, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        return c + d + (d / norms) * self.margin

    @staticmethod
    def inside(poly, pts):
        """Even-odd point-in-polygon, vectorised over all beams at once."""
        x, y = pts[:, 0], pts[:, 1]
        result = np.zeros(len(pts), dtype=bool)
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            straddles = (y0 > y) != (y1 > y)
            with np.errstate(divide='ignore', invalid='ignore'):
                xint = (x1 - x0) * (y - y0) / (y1 - y0) + x0
            result ^= straddles & (x < xint)
        return result

    def solve_geometry(self, msg):
        """Cache the lidar pose in base_link and the per-beam bearings."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            if not self.warned:
                self.get_logger().warn(f'waiting for {self.base_frame} -> '
                                       f'{msg.header.frame_id}: {e}')
                self.warned = True
            return False

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        n = len(msg.ranges)
        bearings = msg.angle_min + np.arange(n) * msg.angle_increment + yaw
        self.geometry = (t.x, t.y, np.cos(bearings), np.sin(bearings), n)
        self.poly_inflated = self.inflated()
        self.get_logger().info(
            f'lidar at x={t.x:.3f} y={t.y:.3f} yaw={math.degrees(yaw):.1f} deg '
            f'in {self.base_frame}; {n} beams')
        return True

    def self_returns(self, msg):
        """Boolean mask of beams whose return lands inside the footprint."""
        lx, ly, cos_b, sin_b, _ = self.geometry
        r = np.asarray(msg.ranges, dtype=float)
        finite = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        pts = np.column_stack((lx + r * cos_b, ly + r * sin_b))
        pts[~finite] = 1e6  # far outside any footprint
        return self.inside(self.poly_inflated, pts) & finite

    def on_scan(self, msg):
        if self.geometry is None or self.geometry[4] != len(msg.ranges):
            if not self.solve_geometry(msg):
                return

        mask = self.self_returns(msg)
        if self.reported is None or abs(int(mask.sum()) - self.reported) > 4:
            self.reported = int(mask.sum())
            self.get_logger().info(f'dropping {self.reported} of {len(msg.ranges)} '
                                   f'returns landing inside the footprint')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[mask] = np.inf
        out.ranges = ranges.tolist()
        if msg.intensities:
            intensities = np.array(msg.intensities, dtype=np.float32)
            intensities[mask] = 0.0
            out.intensities = intensities.tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanSelfFilter()
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

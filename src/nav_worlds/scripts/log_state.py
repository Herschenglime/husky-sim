#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import argparse
import json
import math
import sys
import os

class StateLogger(Node):
    def __init__(self, odom_topic, cmd_vel_topic, scan_topic, output_file):
        super().__init__('state_logger')
        
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        
        self.output_file = output_file
        try:
            self.file = open(self.output_file, 'w')
        except Exception as e:
            self.get_logger().error(f"Failed to open {self.output_file}: {e}")
            sys.exit(1)

        # State storage
        self.latest_odom = None
        self.latest_cmd_vel = None

        # Subscriptions
        # Use sensor data QoS for scan just in case it's published with that profile
        from rclpy.qos import qos_profile_sensor_data
        
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.get_logger().info(f"State logger initialized. Writing to {self.output_file}")
        self.get_logger().info(f"Topics: odom={odom_topic}, cmd_vel={cmd_vel_topic}, scan={scan_topic}")

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def cmd_vel_cb(self, msg: Twist):
        self.latest_cmd_vel = msg

    def get_yaw_from_quaternion(self, q):
        # standard euler from quaternion for yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg: LaserScan):
        # We drive the logging from the scan callback since it's typically lower 
        # frequency than odom and contains the large array. This avoids duplicate 
        # scan arrays in the log.
        if self.latest_odom is None:
            # wait until we have at least one odom
            return

        # Time
        stamp = msg.header.stamp
        time_sec = stamp.sec + stamp.nanosec * 1e-9

        # Odom state
        pos = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(q)

        # Cmd vel state
        if self.latest_cmd_vel is not None:
            vx = self.latest_cmd_vel.linear.x
            wz = self.latest_cmd_vel.angular.z
        else:
            vx = 0.0
            wz = 0.0

        # Replace infinite values in ranges with a high number or None to be JSON compliant
        # JSON standard doesn't strictly support NaN or Infinity (though python json dump does, 
        # it outputs non-standard unquoted Infinity). Let's convert them to None for safety.
        safe_ranges = [
            r if not math.isinf(r) and not math.isnan(r) else None 
            for r in msg.ranges
        ]

        # Construct JSON
        data = {
            "timestamp": time_sec,
            "x": pos.x,
            "y": pos.y,
            "yaw": yaw,
            "vx": vx,
            "wz": wz,
            "scan": safe_ranges
        }

        # Write to JSONL
        self.file.write(json.dumps(data) + '\n')
        self.file.flush()

    def destroy_node(self):
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            self.get_logger().info(f"Closed {self.output_file}")
        super().destroy_node()

def main(args=None):
    parser = argparse.ArgumentParser(description="Log robot state and lidar to JSONL")
    parser.add_argument('--odom_topic', type=str, default='/odom', help='Odometry topic')
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel', help='Command velocity topic')
    parser.add_argument('--scan_topic', type=str, default='/scan', help='LaserScan topic')
    parser.add_argument('--output', type=str, default='state.jsonl', help='Output JSONL file')
    
    # Ignore unknown args (useful if launch system passes extra args)
    parsed_args, unknown = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=sys.argv)
    
    node = StateLogger(
        odom_topic=parsed_args.odom_topic,
        cmd_vel_topic=parsed_args.cmd_vel_topic,
        scan_topic=parsed_args.scan_topic,
        output_file=parsed_args.output
    )
    
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

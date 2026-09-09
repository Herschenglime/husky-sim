#!/usr/bin/env python3
"""Send a point-to-point navigation goal and report the outcome.

    send_goal.py X Y [YAW_DEG] [--ns a200_0000] [--frame map] [--timeout 180]

Reports the result and how close the robot actually got, so a run can be scored
rather than eyeballed.
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalSender(Node):
    def __init__(self, ns, frame):
        super().__init__('send_goal')
        self.frame = frame
        action = f'/{ns}/navigate_to_pose' if ns else '/navigate_to_pose'
        self.client = ActionClient(self, NavigateToPose, action)
        self.action = action
        self.feedback = None

    def send(self, x, y, yaw_deg, timeout):
        if not self.client.wait_for_server(timeout_sec=20.0):
            print(f'FAIL: no action server at {self.action}')
            return 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f'goal ({x:.2f}, {y:.2f}, {yaw_deg:.0f} deg) in {self.frame} -> {self.action}')
        send_future = self.client.send_goal_async(
            goal, feedback_callback=lambda f: setattr(self, 'feedback', f.feedback))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=20.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print('FAIL: goal rejected')
            return 1
        print('goal accepted, navigating...')

        result_future = handle.get_result_async()
        start = time.time()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                handle.cancel_goal_async()
                rclpy.spin_once(self, timeout_sec=2.0)
                print(f'FAIL: timed out after {timeout:.0f} s')
                self.report_distance()
                return 1
        status = result_future.result().status
        elapsed = time.time() - start
        ok = status == GoalStatus.STATUS_SUCCEEDED
        print(f'{"SUCCEEDED" if ok else "FAILED"} after {elapsed:.0f} s (status {status})')
        self.report_distance()
        return 0 if ok else 1

    def report_distance(self):
        if self.feedback is not None:
            p = self.feedback.current_pose.pose.position
            print(f'  final pose ({p.x:.2f}, {p.y:.2f}); '
                  f'distance remaining {self.feedback.distance_remaining:.2f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw', type=float, nargs='?', default=0.0)
    ap.add_argument('--ns', default='a200_0000')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--timeout', type=float, default=180.0)
    args = ap.parse_args()
    rclpy.init()
    node = GoalSender(args.ns, args.frame)
    try:
        rc = node.send(args.x, args.y, args.yaw, args.timeout)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()

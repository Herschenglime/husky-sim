#!/usr/bin/env python3
"""Send a point-to-point navigation goal and report the outcome.

    send_goal.py X Y [YAW_DEG] [--ns a200_0000] [--frame map] [--timeout 180] [--world warehouse]

Reports the result and how close the robot actually got, so a run can be scored
rather than eyeballed.
"""
import argparse
import math
import sys
import time
import subprocess

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def spawn_marker(world, x, y):
    if not world:
        return

    # Remove old marker if it exists
    subprocess.run([
        'gz', 'service', '-s', f'/world/{world}/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000',
        '--req', 'name: "goal_marker", type: MODEL'
    ], capture_output=True)

    # Spawn new marker
    sdf = f"""<sdf version="1.7"><model name="goal_marker"><static>true</static><pose>{x} {y} 0.1 0 0 0</pose><link name="link"><visual name="visual"><geometry><sphere><radius>0.2</radius></sphere></geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model></sdf>"""
    subprocess.run([
        'gz', 'service', '-s', f'/world/{world}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000',
        '--req', f"sdf: '{sdf}'"
    ], capture_output=True)


def report_distance(feedback):
    if feedback is not None:
        p = feedback.current_pose.pose.position
        print(f'  final pose ({p.x:.2f}, {p.y:.2f}); '
              f'distance remaining {feedback.distance_remaining:.2f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw', type=float, nargs='?', default=0.0)
    ap.add_argument('--ns', default='a200_0000')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--world', default='', help='Gazebo world name for visual marker spawning')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--use-sim-time', action='store_true', default=None)

    # Filter out ROS 2 specific arguments injected when run via launch Node
    try:
        from rclpy.utilities import remove_ros_args
        clean_argv = remove_ros_args(args=sys.argv)[1:]
    except Exception:
        clean_argv = [a for a in sys.argv[1:] if not a.startswith('--ros-args') and not a.startswith('__node:=') and not a == '-r']
    args, _ = ap.parse_known_args(clean_argv)

    rclpy.init()

    navigator = BasicNavigator(node_name='send_goal', namespace=args.ns)

    if args.use_sim_time is not None:
        navigator.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, args.use_sim_time)
        ])

    print(f'Waiting for Nav2 to become active in namespace /{args.ns}...')
    navigator.waitUntilNav2Active(localizer='robot_localization')

    spawn_marker(args.world, args.x, args.y)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = args.frame
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(args.x)
    goal_pose.pose.position.y = float(args.y)
    yaw = math.radians(args.yaw)
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

    print(f'goal ({args.x:.2f}, {args.y:.2f}, {args.yaw:.0f} deg) in {args.frame}')
    accepted = navigator.goToPose(goal_pose)
    if not accepted:
        print('FAIL: goal rejected')
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print('goal accepted, navigating...')
    start = time.time()
    last_feedback_time = 0.0

    while not navigator.isTaskComplete():
        now = time.time()
        feedback = navigator.getFeedback()
        if feedback and (now - last_feedback_time >= 5.0):
            last_feedback_time = now
            print(f'  distance remaining: {feedback.distance_remaining:.2f} m')

        if now - start > args.timeout:
            print(f'FAIL: timed out after {args.timeout:.0f} s')
            navigator.cancelTask()
            report_distance(navigator.getFeedback())
            navigator.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    result = navigator.getResult()
    elapsed = time.time() - start
    ok = (result == TaskResult.SUCCEEDED)

    if ok:
        status_str = 'SUCCEEDED'
    elif result == TaskResult.CANCELED:
        status_str = 'CANCELED'
    else:
        status_str = 'FAILED'

    print(f'{status_str} after {elapsed:.0f} s (status {navigator.status})')
    report_distance(navigator.getFeedback())

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

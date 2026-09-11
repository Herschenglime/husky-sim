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

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def publish_goal_marker(navigator, x, y, frame='map', namespace='a200_0000', world=''):
    """Publish visualization marker for RViz and spawn visual sphere in Gazebo GUI."""
    # 1. Publish ROS 2 Marker for RViz
    marker_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE
    )
    topic = f'/{namespace}/goal_marker' if namespace else '/goal_marker'
    marker_pub = navigator.create_publisher(Marker, topic, marker_qos)
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = navigator.get_clock().now().to_msg()
    marker.ns = 'goal'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = 0.1
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.9
    marker_pub.publish(marker)

    # 2. Render Visual Marker in Gazebo Simulation GUI
    if world:
        try:
            from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
            from ros_gz_interfaces.msg import Entity

            remove_client = navigator.create_client(DeleteEntity, f'/world/{world}/remove')
            create_client = navigator.create_client(SpawnEntity, f'/world/{world}/create')

            if remove_client.wait_for_service(timeout_sec=0.5):
                req_del = DeleteEntity.Request()
                req_del.entity.name = 'goal_marker'
                req_del.entity.type = Entity.MODEL
                fut_del = remove_client.call_async(req_del)
                rclpy.spin_until_future_complete(navigator, fut_del, timeout_sec=0.5)

            if create_client.wait_for_service(timeout_sec=0.5):
                req_spawn = SpawnEntity.Request()
                req_spawn.entity_factory.name = 'goal_marker'
                req_spawn.entity_factory.allow_renaming = False
                req_spawn.entity_factory.pose.position.x = float(x)
                req_spawn.entity_factory.pose.position.y = float(y)
                req_spawn.entity_factory.pose.position.z = 0.1
                req_spawn.entity_factory.pose.orientation.w = 1.0
                req_spawn.entity_factory.sdf = """<sdf version="1.7"><model name="goal_marker"><static>true</static><link name="link"><visual name="visual"><geometry><sphere><radius>0.2</radius></sphere></geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model></sdf>"""
                fut_spawn = create_client.call_async(req_spawn)
                rclpy.spin_until_future_complete(navigator, fut_spawn, timeout_sec=1.0)
        except Exception:
            pass


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
    ap.add_argument('--use-sim-time', dest='use_sim_time', action='store_true', default=True,
                    help='Use simulation clock from /clock (default: True)')
    ap.add_argument('--no-sim-time', dest='use_sim_time', action='store_false',
                    help='Use wall clock instead of simulation clock')

    # Filter out ROS 2 specific arguments injected when run via launch Node
    try:
        from rclpy.utilities import remove_ros_args
        clean_argv = remove_ros_args(args=sys.argv)[1:]
    except Exception:
        clean_argv = [a for a in sys.argv[1:] if not a.startswith('--ros-args') and not a.startswith('__node:=') and not a == '-r']
    args, _ = ap.parse_known_args(clean_argv)

    rclpy.init()

    navigator = BasicNavigator(node_name='send_goal', namespace=args.ns)

    navigator.set_parameters([
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, args.use_sim_time)
    ])

    print(f'Waiting for Nav2 to become active in namespace /{args.ns}...')
    navigator.waitUntilNav2Active(localizer='robot_localization')
    for server_name in ['planner_server', 'controller_server']:
        try:
            navigator._waitForNodeToActivate(server_name)
        except Exception as e:
            navigator.get_logger().warn(f'Wait for {server_name} returned: {e}')

    publish_goal_marker(navigator, args.x, args.y, frame=args.frame, namespace=args.ns, world=args.world)

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
    clock = navigator.get_clock()
    start_sim = clock.now()
    start_wall = time.time()
    last_feedback_time = 0.0

    while not navigator.isTaskComplete():
        now_wall = time.time()
        feedback = navigator.getFeedback()
        if feedback and (now_wall - last_feedback_time >= 5.0):
            last_feedback_time = now_wall
            print(f'  distance remaining: {feedback.distance_remaining:.2f} m')

        now_sim = clock.now()
        elapsed_sim = (now_sim - start_sim).nanoseconds * 1e-9
        if elapsed_sim > args.timeout:
            print(f'FAIL: timed out after {args.timeout:.0f} s (sim time)')
            navigator.cancelTask()
            report_distance(navigator.getFeedback())
            navigator.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    result = navigator.getResult()
    elapsed_sim = (clock.now() - start_sim).nanoseconds * 1e-9
    elapsed_wall = time.time() - start_wall
    ok = (result == TaskResult.SUCCEEDED)

    if ok:
        status_str = 'SUCCEEDED'
    elif result == TaskResult.CANCELED:
        status_str = 'CANCELED'
    else:
        status_str = 'FAILED'

    print(f'{status_str} after {elapsed_sim:.1f} s sim ({elapsed_wall:.1f} s wall) (status {navigator.status})')
    report_distance(navigator.getFeedback())

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

"""
Bring-up for bug0_a300 on the Clearpath A300 Observer (+ optional bag recording).

Port of bug0_turtlebot4/launch/bringup.launch.py. Differences from the TurtleBot4
version are limited to the platform:

  * everything runs inside the robot namespace (default a300_00000) with /tf and
    /tf_static remapped into it, as Clearpath does;
  * the 3D lidar (sensors/lidar3d_1/points) is flattened into a 2D LaserScan in
    base_link by pointcloud_to_laserscan, published on sensors/lidar3d_1/scan_2d,
    and that scan is what bug0_node consumes (so lidar_yaw_offset_deg is 0);
  * cmd_vel is TwistStamped on <namespace>/cmd_vel (Clearpath twist_mux input);
  * recorded topics are the A300 equivalents of the TurtleBot4 ones.

    ros2 launch bug0_a300 bringup.launch.py goal_x:=-7.0 goal_y:=-1.0 wall_side:=auto

Set record_bag:=false to bring up the controller only, with no recording.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR3D_HORIZONTAL_SAMPLES = 1024


def _nodes(context, *args, **kwargs):
    lc = LaunchConfiguration
    ns = lc('namespace').perform(context).strip('/')
    use_sim_time = lc('use_sim_time').perform(context).lower() == 'true'
    scan_topic = lc('scan_topic').perform(context)
    cmd_vel_topic = lc('cmd_vel_topic').perform(context)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def absolute(topic: str) -> str:
        return topic if topic.startswith('/') else f'/{ns}/{topic}'

    return [
        # ---------------- 3D lidar -> 2D scan in base_link ----------------
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='lidar3d_to_scan',
            namespace=ns,
            output='screen',
            remappings=tf_remaps + [
                ('cloud_in', lc('cloud_topic').perform(context)),
                ('scan', scan_topic),
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': lc('base_frame').perform(context),
                'transform_tolerance': 0.05,
                'min_height': float(lc('scan_min_height').perform(context)),
                'max_height': float(lc('scan_max_height').perform(context)),
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 2.0 * math.pi / LIDAR3D_HORIZONTAL_SAMPLES,
                'scan_time': 0.05,
                'range_min': 0.3,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),

        # ---------------- bug0_node ----------------
        Node(
            package='bug0_a300',
            executable='bug0_node',
            name='bug0_node',
            namespace=ns,
            output='screen',
            remappings=tf_remaps,
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_x': float(lc('goal_x').perform(context)),
                'goal_y': float(lc('goal_y').perform(context)),
                'wall_side': lc('wall_side').perform(context),
                'scan_topic': scan_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'use_twist_stamped': lc('use_twist_stamped').perform(context).lower() == 'true',
                'lidar_yaw_offset_deg': float(lc('lidar_yaw_offset_deg').perform(context)),
                'require_progress_to_leave':
                    lc('require_progress_to_leave').perform(context).lower() == 'true',
                'force_goal_heading_before_wall_follow':
                    lc('force_goal_heading_before_wall_follow').perform(context).lower() == 'true',
                'global_frame': lc('global_frame').perform(context),
                'base_frame': lc('base_frame').perform(context),
                'tf_timeout_sec': float(lc('tf_timeout_sec').perform(context)),
                'debug': lc('debug').perform(context).lower() == 'true',
            }],
        ),

        # ---------------- bag_recorder_node ----------------
        Node(
            package='turtlebot4_bag_recorder',
            executable='bag_recorder_node',
            name='bag_recorder_node',
            namespace=ns,
            output='screen',
            condition=IfCondition(lc('record_bag')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'bag_base_dir': lc('bag_base_dir').perform(context),
                'bag_name': lc('bag_name').perform(context),
                'storage_id': lc('storage_id').perform(context),
                'topics': [
                    absolute(scan_topic),
                    f'/{ns}/tf',
                    f'/{ns}/platform/odom',
                    f'/{ns}/sensors/camera_0/color/image',
                    absolute(cmd_vel_topic),
                ],
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([

        # ---------------- platform ----------------
        DeclareLaunchArgument(
            'namespace', default_value='a300_00000',
            description='Robot namespace (clearpath robot.yaml system.ros2.namespace)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Set false when running on the physical robot'),
        DeclareLaunchArgument(
            'record_bag', default_value='true',
            choices=['true', 'false'],
            description='Also launch bag_recorder_node alongside the controller'),

        # ---------------- 3D lidar -> 2D scan ----------------
        DeclareLaunchArgument(
            'cloud_topic', default_value='sensors/lidar3d_1/points',
            description='3D lidar PointCloud2 (relative to namespace)'),
        DeclareLaunchArgument(
            'scan_min_height', default_value='0.10',
            description='Lowest point height kept, in base_frame (m); base_link is '
                        '~0.17 m above the floor, so this rejects floor returns'),
        DeclareLaunchArgument(
            'scan_max_height', default_value='1.50',
            description='Highest point height kept, in base_frame (m)'),

        # ---------------- controller args ----------------
        DeclareLaunchArgument('goal_x', default_value='-7.0'),
        DeclareLaunchArgument('goal_y', default_value='-1.0'),
        DeclareLaunchArgument('wall_side', default_value='auto'),
        DeclareLaunchArgument('scan_topic', default_value='sensors/lidar3d_1/scan_2d'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('use_twist_stamped', default_value='true'),
        DeclareLaunchArgument('lidar_yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.10'),
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('require_progress_to_leave', default_value='true'),
        DeclareLaunchArgument(
            'force_goal_heading_before_wall_follow', default_value='true'),

        # ---------------- bag_recorder_node args ----------------
        DeclareLaunchArgument(
            'bag_base_dir', default_value='/tmp/a300_bags',
            description='Directory where bags are stored'),
        DeclareLaunchArgument(
            'bag_name', default_value='',
            description='Bag folder name (empty = timestamped)'),
        DeclareLaunchArgument(
            'storage_id', default_value='sqlite3',
            description='rosbag2 storage plugin: sqlite3 or mcap'),

        OpaqueFunction(function=_nodes),
    ])

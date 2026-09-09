"""Record a platform's sensors to a rosbag.

    ros2 launch platform_bag_recorder record.launch.py platform:=go1
    ros2 launch platform_bag_recorder record.launch.py platform:=a200

Stop with Ctrl+C; the bag is finalized on the way out.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('platform', default_value='a200',
                          choices=['a200', 'go1'],
                          description='Which robot to record.'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Override the profile default namespace.'),
    DeclareLaunchArgument('bag_base_dir', default_value='~/bags',
                          description='Directory to write bags into.'),
    DeclareLaunchArgument('bag_name', default_value='',
                          description='Bag name; empty gives a timestamped one.'),
    DeclareLaunchArgument('storage_id', default_value='mcap',
                          choices=['mcap', 'sqlite3']),
    DeclareLaunchArgument('sync_rate_hz', default_value='0.0',
                          description='0 records every message as it arrives; '
                                      '>0 writes all topics synchronized at '
                                      'that rate.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false']),
]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [Node(
        package='platform_bag_recorder',
        executable='bag_recorder',
        name='bag_recorder',
        output='screen',
        parameters=[{
            'platform': LaunchConfiguration('platform'),
            'namespace': LaunchConfiguration('namespace'),
            'bag_base_dir': LaunchConfiguration('bag_base_dir'),
            'bag_name': LaunchConfiguration('bag_name'),
            'storage_id': LaunchConfiguration('storage_id'),
            'sync_rate_hz': LaunchConfiguration('sync_rate_hz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )])

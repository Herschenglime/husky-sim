from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bag_base_dir = LaunchConfiguration('bag_base_dir')
    bag_name = LaunchConfiguration('bag_name')
    storage_id = LaunchConfiguration('storage_id')
    sync_rate_hz = LaunchConfiguration('sync_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set true when recording from the Gazebo simulation'),
        DeclareLaunchArgument(
            'bag_base_dir', default_value='/tmp/tb4/',
            description='Directory where bags are stored'),
        DeclareLaunchArgument(
            'bag_name', default_value='',
            description='Bag folder name (empty = timestamped)'),
        DeclareLaunchArgument(
            'storage_id', default_value='sqlite3',
            description='rosbag2 storage plugin: sqlite3 or mcap'),
        DeclareLaunchArgument(
            'sync_rate_hz', default_value='0.0',
            description='Uniform recording rate for all topics '
                        '(0.0 = auto-detect the slowest publisher)'),

        Node(
            package='turtlebot4_bag_recorder',
            executable='bag_recorder_node',
            name='bag_recorder_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'bag_base_dir': bag_base_dir,
                'bag_name': bag_name,
                'storage_id': storage_id,
                'sync_rate_hz': sync_rate_hz,
                'topics': [
                    '/scan',
                    '/tf',
                    '/odom',
                    '/oakd/rgb/preview/image_raw',
                    '/cmd_vel',
                ],
            }],
        ),
    ])

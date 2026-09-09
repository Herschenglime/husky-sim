"""Sim + line follower.

Brings up line_follow_sim's warehouse world and Husky, then the follower node.
The follower publishes nothing until its first camera frame arrives, so it does
not matter that it starts before the sim is ready.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false'],
                          description='Open the Gazebo GUI (default: headless).'),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
        [get_package_share_directory('line_follower'), 'config', 'line_follower.yaml']),
        description='Follower parameters.'),
]


def generate_launch_description():
    pkg_sim = get_package_share_directory('line_follow_sim')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_sim, 'launch', 'line_follow_sim.launch.py')]),
        launch_arguments=[('gui', LaunchConfiguration('gui')),
                          ('rviz', LaunchConfiguration('rviz'))],
    )

    follower = Node(
        package='line_follower',
        executable='follower',
        name='line_follower',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # No ground-truth pose bridge here on purpose: ros_gz_bridge's
    # Pose_V -> TFMessage conversion drops the entity name, so the robot cannot
    # be picked out of the message. The scoring harness reads `gz topic`
    # directly instead.
    return LaunchDescription(ARGUMENTS + [sim, follower])

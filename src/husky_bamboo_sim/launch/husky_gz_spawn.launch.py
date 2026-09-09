"""Spawn Husky into Gazebo and start platform + sensor GZ–ROS bridges.

DEPRECATED: not used by the mission stack. ``clearpath_bamboo_sim.launch.py`` now
includes ``clearpath_gz simulation.launch.py`` as the sole Gazebo + spawn entry.
Kept for reference only.
"""

import os

from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _spawn_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    clearpath_config = ClearpathConfig(os.path.join(setup_path, 'robot.yaml'))
    namespace = clearpath_config.system.namespace
    robot_name = 'robot' if namespace in ('', '/') else f'{namespace}/robot'

    platform_launch = os.path.join(setup_path, 'platform/launch/platform-service.launch.py')
    sensors_launch = os.path.join(setup_path, 'sensors/launch/sensors-service.launch.py')

    sensor_prefix = f'/world/{world}/model/{robot_name}/link/base_link/sensor/'

    return [
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(platform_launch),
                launch_arguments=[('prefix', sensor_prefix)],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments=[('prefix', sensor_prefix)],
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                namespace=namespace,
                output='screen',
                arguments=[
                    '-name', robot_name,
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-Y', yaw,
                    '-topic', 'robot_description',
                ],
            ),
        ]),
    ]


def generate_launch_description():
    default_setup = os.path.join(os.path.expanduser('~'), '.clearpath')

    return LaunchDescription([
        DeclareLaunchArgument(
            'setup_path',
            default_value=default_setup,
            description='Clearpath config directory (must already be generated)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='bamboo_field'),
        DeclareLaunchArgument('x', default_value='-30.0'),
        DeclareLaunchArgument('y', default_value='-30.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_spawn_setup),
    ])

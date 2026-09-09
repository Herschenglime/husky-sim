"""Spawn the A300 platform without waiting on the robot_description topic.

Stock ``robot_spawn.launch.py`` passes ``-topic robot_description`` to
``ros_gz_sim create``. When DDS shared-memory ports are contended (common after
stale sim runs), discovery can take 60+ seconds while the controller spawner
times out.  Pre-processing the URDF with xacro and using ``-file`` avoids that
race entirely.
"""

from __future__ import annotations

import os
import shutil
import subprocess

from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('generate',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Generate parameters and launch files'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.15',
                 description='z component of the robot pose.'))


def _write_urdf(context) -> str:
    """Run xacro once at launch time and return the generated URDF path."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    namespace = ClearpathConfig(os.path.join(setup_path, 'robot.yaml')).system.namespace

    robot_urdf_xacro = os.path.join(setup_path, 'robot.urdf.xacro')
    config_control = os.path.join(setup_path, 'platform/config/control.yaml')
    output_path = os.path.join(setup_path, 'robot.urdf')

    xacro_bin = shutil.which('xacro')
    if not xacro_bin:
        raise RuntimeError('xacro not found on PATH; source /opt/ros/jazzy/setup.bash')

    cmd = [
        xacro_bin,
        robot_urdf_xacro,
        f'is_sim:={use_sim_time}',
        f'gazebo_controllers:={config_control}',
        f'namespace:={namespace}',
        'use_fake_hardware:=false',
        'use_manipulation_controllers:=true',
        'use_platform_controllers:=true',
    ]
    env = os.environ.copy()
    env['PATH'] = '/usr/bin:/opt/ros/jazzy/bin:' + env.get('PATH', '')
    with open(output_path, 'w', encoding='utf-8') as urdf_file:
        subprocess.run(cmd, check=True, stdout=urdf_file, env=env)
    return output_path


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    generate = LaunchConfiguration('generate')

    clearpath_config = ClearpathConfig(os.path.join(
        str(setup_path.perform(context)), 'robot.yaml'))

    namespace = clearpath_config.system.namespace
    if namespace in ('', '/'):
        robot_name = 'robot'
    else:
        robot_name = namespace + '/robot'

    urdf_path = _write_urdf(context)

    pkg_clearpath_viz = FindPackageShare('clearpath_viz')
    rviz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_robot.launch.py'])
    launch_file_platform_service = PathJoinSubstitution([
        setup_path, 'platform/launch', 'platform-service.launch.py'])
    launch_file_sensors_service = PathJoinSubstitution([
        setup_path, 'sensors/launch', 'sensors-service.launch.py'])

    # robot_state_publisher (platform-service) is the single, latched robot_description
    # source for gz_ros2_control; a second publisher makes controller_manager warn
    # "ResourceManager has already loaded a urdf".
    group_action_spawn_robot = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_platform_service]),
            launch_arguments=[
                ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_sensors_service]),
            launch_arguments=[
                ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-name', robot_name,
                       '-file', urdf_path,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw],
            output='screen',
        ),
    ])

    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path],
    )

    node_generate_semantic_description = Node(
        package='clearpath_generator_common',
        executable='generate_semantic_description',
        name='generate_semantic_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path],
    )

    node_generate_launch = Node(
        package='clearpath_generator_gz',
        executable='generate_launch',
        name='generate_launch',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path],
    )

    node_generate_param = Node(
        package='clearpath_generator_gz',
        executable='generate_param',
        name='generate_param',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', setup_path],
    )

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=[node_generate_semantic_description],
        ),
    )

    event_generate_semantic_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_semantic_description,
            on_exit=[node_generate_launch],
        ),
    )

    event_generate_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_launch,
            on_exit=[node_generate_param],
        ),
    )

    event_generate_param = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_param,
            on_exit=[group_action_spawn_robot],
        ),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    do_generate = GroupAction(
        actions=[
            node_generate_description,
            event_generate_description,
            event_generate_semantic_description,
            event_generate_launch,
            event_generate_param,
            rviz,
        ],
        condition=IfCondition(LaunchConfiguration('generate')),
    )

    do_not_generate = GroupAction(
        actions=[group_action_spawn_robot, rviz],
        condition=UnlessCondition(LaunchConfiguration('generate')),
    )

    return [do_generate, do_not_generate]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

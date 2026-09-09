"""Spawn the Clearpath Husky in the warehouse with the painted line track.

This is clearpath_gz's own simulation bringup - gz sim + clock bridge +
clearpath's robot_spawn, which regenerates robot.urdf.xacro from robot.yaml on
every run - with two additions:

  * GZ_SIM_RESOURCE_PATH extended so this package's worlds/ and models/ resolve,
    since warehouse_line.sdf and model://line_track live here rather than in
    clearpath_gz.
  * A top-up of the ros2_control controllers (see below).

We call ros_gz_sim's gz_sim.launch.py rather than clearpath_gz's wrapper for one
reason only: the wrapper has no headless option, and there is no display here.
Everything else follows clearpath_gz/launch/simulation.launch.py.

Headless by default: pass gui:=true for the Gazebo window.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from clearpath_config.clearpath_config import ClearpathConfig

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'], description='use_sim_time'),
    DeclareLaunchArgument('gui', default_value='false',
                          choices=['true', 'false'],
                          description='Open the Gazebo GUI (default: headless).'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse_line',
                          description='World name (must match <world name=...> in the sdf).'),
    # Default spawn pose puts the robot on the line, tangent to it: the
    # straightest stretch of the whole loop runs along y = 13.26 heading +x,
    # with the nearest prop (shelf_big_3) 3.8 m away.
    DeclareLaunchArgument('x', default_value='5.0', description='Robot spawn x.'),
    DeclareLaunchArgument('y', default_value='13.26', description='Robot spawn y.'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw.'),
    DeclareLaunchArgument('z', default_value='0.3', description='Robot spawn z.'),
]


def launch_setup(context, *args, **kwargs):
    pkg_line_follow_sim = get_package_share_directory('line_follow_sim')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    setup_path = LaunchConfiguration('setup_path').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world = LaunchConfiguration('world').perform(context)

    # clearpath_gz's own recipe, plus this package's worlds/ and models/ so
    # `model://line_track` and warehouse_line.sdf both resolve.
    ament_prefix_paths = os.getenv('AMENT_PREFIX_PATH', '').split(':')
    package_share_paths = [os.path.join(p, 'share') for p in ament_prefix_paths if p]
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_line_follow_sim, 'worlds'),
            os.path.join(pkg_line_follow_sim, 'models'),
            os.path.join(pkg_clearpath_gz, 'worlds'),
            os.path.join(pkg_clearpath_gz, 'meshes'),
            *package_share_paths,
        ])
    )

    world_sdf = os.path.join(pkg_line_follow_sim, 'worlds', f'{world}.sdf')
    gz_args = f'{world_sdf} -r -v 4'
    if not gui:
        gz_args += ' -s --headless-rendering'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])]),
        launch_arguments=[
            ('setup_path', setup_path),
            ('world', world),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ]
    )

    # ros2_control spawners.
    #
    # Which controllers clearpath_control spawns under sim varies by release:
    # clearpath-simulator <= 2.9.3 spawned neither of the a200's (its sim-mode
    # loop only takes controllers whose name contains 'controller' and contains
    # neither 'manager' nor 'platform' - joint_state_broadcaster and
    # platform_velocity_controller each fail that filter), so controller_manager
    # came up empty and the robot ignored cmd_vel. Newer releases spawn both.
    #
    # So this cannot be a plain spawner: on the newer release a second spawner
    # exits 1 with "can not be configured from 'active' state". ensure_controllers
    # asks controller_manager what is already running and spawns only the gaps,
    # which is correct on either release.
    #
    # Delayed because controller_manager only exists once the robot entity is in
    # the world; --timeout then covers the rest.
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    check_camera_configured(robot_yaml)
    namespace = ClearpathConfig(robot_yaml).system.namespace
    ensure_controllers = os.path.join(pkg_line_follow_sim, 'scripts', 'ensure_controllers.py')
    controller_spawner = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['python3', ensure_controllers,
                 '-c', f'/{namespace}/controller_manager',
                 '--timeout', '60', '--settle', '20',
                 'joint_state_broadcaster', 'platform_velocity_controller'],
            additional_env={'ROS_SUPER_CLIENT': 'True'},
            output='screen',
        )],
    )

    return [resource_path, gz_sim, clock_bridge, robot_spawn, controller_spawner]


def check_camera_configured(robot_yaml):
    """Fail at launch, not silently at runtime, if there is no camera.

    The floor-facing camera lives in the Clearpath robot config rather than in
    this package, so a fresh checkout comes up with lidar only. Without this
    check the sim starts perfectly, publishes no image topic, and the follower
    waits forever for a first frame - which looks like a broken follower.
    """
    with open(robot_yaml) as f:
        config = yaml.safe_load(f) or {}
    if (config.get('sensors') or {}).get('camera'):
        return
    installer = os.path.join(
        get_package_share_directory('line_follow_sim'), 'scripts', 'install_robot_config.sh')
    raise RuntimeError(
        f'\n\nNo camera configured in {robot_yaml}, so there will be no image '
        f'topic and the line follower can never see the track.\n'
        f'Install the robot config this sim expects:\n\n'
        f'    {installer}\n')


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

"""Bring up a Gazebo world with the Clearpath robot in it.

Replaces clearpath_gz's simulation.launch.py, which is otherwise exactly what is
wanted, for two reasons:

  * Its `world` argument carries a `choices` list of the six worlds that package
    ships. Anything else - a world carried here, such as depot - is rejected
    outright, and so is an absolute path.
  * There is no way to run without the GUI. clearpath_gz's gz_sim.launch.py
    builds the `gz_args` string itself and exposes no hook, so `-s
    --headless-rendering` cannot be threaded through it.

The two consumers of `world` need different things and must not be conflated:

  * gz_sim wants something it can turn into a file. It builds gz_args as
    "<world>.sdf", so it gets a path with the extension stripped.
  * robot_spawn wants the name of the world *inside* the SDF, because it calls
    the /world/<name>/create service to place the robot.

For clearpath's own worlds those coincide, which is why the distinction is easy
to miss until a world lives somewhere else.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Name of the world (must match <world name=...>).'),
    DeclareLaunchArgument('world_file', default_value='',
                          description='Path to a world SDF. Empty means use the '
                                      'clearpath_gz world of the same name.'),
    DeclareLaunchArgument('setup_path',
                          default_value=[os.environ.get('HOME', ''), '/clearpath/']),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false']),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                          description='Run the server only, with offscreen sensor '
                                      'rendering and no GUI.'),
]
for axis in ('x', 'y', 'yaw'):
    ARGUMENTS.append(DeclareLaunchArgument(axis, default_value='0.0'))
ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3'))


def headless_gz(world_sdf):
    """Server-only Gazebo, replicating what clearpath_gz's gz_sim.launch.py sets up.

    Three things have to come across or the robot never appears: the resource
    path (clearpath *overwrites* GZ_SIM_RESOURCE_PATH with its own worlds and
    meshes plus every sourced package), the clock bridge (without it nothing
    using sim time ever gets a clock and the whole stack sits waiting), and the
    world itself.

    `-s` runs the server alone; `--headless-rendering` is separate and still
    needed, because sensors that render - the realsense on the arch - produce
    nothing without a rendering context, and the failure is silent: the camera
    topic is advertised and simply never publishes.
    """
    pkg_gz = get_package_share_directory('clearpath_gz')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    packages_paths = [os.path.join(p, 'share')
                      for p in os.getenv('AMENT_PREFIX_PATH', '').split(':') if p]

    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_gz, 'worlds') + ':',
               os.path.join(pkg_gz, 'meshes') + ':',
               ':' + ':'.join(packages_paths)])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', f'{world_sdf} -r -s --headless-rendering -v 4')],
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    return [resource_path, gz_sim, clock_bridge]


def launch_setup(context, *args, **kwargs):
    pkg_gz = get_package_share_directory('clearpath_gz')
    world = LaunchConfiguration('world').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'

    # gz_sim appends '.sdf', so hand it the path without the extension.
    gz_world = world_file[:-4] if world_file.endswith('.sdf') else (world_file or world)

    if headless:
        actions = headless_gz(f'{gz_world}.sdf')
    else:
        actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments=[('world', gz_world),
                              ('setup_path', LaunchConfiguration('setup_path'))])]

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'robot_spawn.launch.py')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('setup_path', LaunchConfiguration('setup_path')),
                          ('world', world),
                          ('rviz', LaunchConfiguration('rviz')),
                          ('x', LaunchConfiguration('x')),
                          ('y', LaunchConfiguration('y')),
                          ('z', LaunchConfiguration('z')),
                          ('yaw', LaunchConfiguration('yaw'))],
    )
    return actions + [robot_spawn]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])

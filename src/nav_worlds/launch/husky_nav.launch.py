"""Point-to-point navigation for the Clearpath a200, in one command.

    ros2 launch nav_worlds husky_nav.launch.py world:=office

Brings up the Gazebo world with the robot, then SLAM (or AMCL against a saved
map) and the Nav2 stack. Send goals with:

    ros2 run nav_worlds send_goal.py X Y [YAW_DEG]

Worlds: warehouse, office, construction, solar_farm, orchard, pipeline come
from clearpath_gz; depot is carried in this package because clearpath_gz does
not ship one.

Two things this arranges that are easy to get wrong:

  * The robot config must not carry the sensor arch. Its legs sit ~0.40 m from
    the 2D lidar, inside the 0.5 m footprint half-length, so 22 of 720 beams
    return the robot's own structure. Nav2's collision_monitor reads those as an
    imminent collision and scales every command to zero - the robot accepts a
    goal, plans a path, and never moves. config/robot_nav.yaml drops the arch.
  * SLAM and AMCL both publish map -> odom, so exactly one of them may run.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo world to load.'),
    DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false'],
                          description='true: build a map as you go. '
                                      'false: localize in the map given by map_file.'),
    DeclareLaunchArgument('map_file', default_value='',
                          description='Map yaml for slam:=false. Defaults to the '
                                      'map of the chosen world under maps/.'),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                          description='Start rviz with the navigation view.'),
    DeclareLaunchArgument('setup_path',
                          default_value=[os.environ.get('HOME', ''), '/clearpath/'],
                          description='Clearpath setup directory.'),
]


def generate_launch_description():
    pkg = get_package_share_directory('nav_worlds')
    pkg_gz = get_package_share_directory('clearpath_gz')
    pkg_nav = get_package_share_directory('clearpath_nav2_demos')

    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    setup_path = LaunchConfiguration('setup_path')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'simulation.launch.py')),
        launch_arguments={'world': world,
                          'setup_path': setup_path,
                          'rviz': LaunchConfiguration('rviz')}.items(),
    )

    # Delayed: both need the robot's TF tree and scan, which only exist once the
    # robot entity is in the world and its controllers are up.
    slam_node = TimerAction(period=20.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'setup_path': setup_path}.items(),
        condition=IfCondition(slam))])

    localization = TimerAction(period=20.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'setup_path': setup_path,
                          'map': PathJoinSubstitution([pkg, 'maps', [world, '.yaml']])}.items(),
        condition=UnlessCondition(slam))])

    nav2 = TimerAction(period=26.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'setup_path': setup_path}.items())])

    return LaunchDescription(ARGUMENTS + [sim, slam_node, localization, nav2])

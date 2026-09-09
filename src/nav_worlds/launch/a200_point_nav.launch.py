"""A200 point-to-point navigation bringup (Simulation + SLAM/AMCL + Nav2).

Replaces the legacy bringup.sh script with a native, event-driven ROS 2 launch
file. Staged using readiness_gate to ensure simulation stability before launching
SLAM and Nav2.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo world name (e.g. warehouse, office, depot).'),
    DeclareLaunchArgument('world_file', default_value='',
                          description='Explicit path to a world SDF file. If empty, resolves automatically.'),
    DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false'],
                          description='true: build a map using SLAM. false: localize against a saved map.'),
    DeclareLaunchArgument('map', default_value='',
                          description='Full path to map yaml file when slam:=false.'),
    DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false'],
                          description='Run Gazebo headless without GUI.'),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false'],
                          description='Open RViz with navigation configuration.'),
    DeclareLaunchArgument('setup_path',
                          default_value=[os.environ.get('HOME', ''), '/clearpath/'],
                          description='Path to clearpath robot configuration directory.'),
    DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
    DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position'),
    DeclareLaunchArgument('z', default_value='0.3', description='Spawn Z position'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw orientation (rad)'),
]


def launch_setup(context, *args, **kwargs):
    pkg_nav_worlds = get_package_share_directory('nav_worlds')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    world = LaunchConfiguration('world').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)
    setup_path = LaunchConfiguration('setup_path').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)

    # Fall back to nav_worlds/config/ if robot.yaml does not exist in setup_path
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    if not os.path.exists(robot_yaml):
        fallback_setup = os.path.join(pkg_nav_worlds, 'config')
        if os.path.exists(os.path.join(fallback_setup, 'robot.yaml')):
            setup_path = fallback_setup
            robot_yaml = os.path.join(setup_path, 'robot.yaml')

    # Determine namespace from robot.yaml
    namespace = 'a200_0000'
    if os.path.exists(robot_yaml):
        try:
            with open(robot_yaml, 'r') as f:
                cfg = yaml.safe_load(f)
                namespace = cfg.get('system', {}).get('ros2', {}).get('namespace', 'a200_0000')
        except Exception:
            pass

    # Resolve world file if needed (e.g. depot.sdf from nav_worlds/worlds)
    local_world = os.path.join(pkg_nav_worlds, 'worlds', f'{world}.sdf')
    if not world_file and os.path.exists(local_world):
        world_file = local_world

    # 1. Simulation launch (sim.launch.py)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav_worlds, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': world,
            'world_file': world_file,
            'setup_path': setup_path,
            'headless': headless,
            'rviz': rviz,
            'use_sim_time': 'true',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items()
    )

    # 2. Scan self-filter (masks robot's sensor arch from 2D scan)
    scan_filtered_topic = f'/{namespace}/sensors/lidar2d_0/scan_filtered'
    scan_filter_node = Node(
        package='nav_worlds',
        executable='scan_self_filter.py',
        name='scan_self_filter',
        namespace=namespace,
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', f'/{namespace}/tf'),
            ('/tf_static', f'/{namespace}/tf_static'),
        ],
        output='screen'
    )

    # 3. Simulation readiness gate (verifies clock, lidar, active controllers, and odom TF)
    sim_gate_node = Node(
        package='nav_worlds',
        executable='readiness_gate.py',
        name='sim_gate',
        namespace=namespace,
        parameters=[{
            'namespace': namespace,
            'stage': 'sim',
            'use_sim_time': True,
            'timeout_sec': 120.0
        }],
        remappings=[
            ('/tf', f'/{namespace}/tf'),
            ('/tf_static', f'/{namespace}/tf_static'),
        ],
        output='screen'
    )

    # 4. Stage 1: SLAM or Localization + Map Gate
    slam_actions = []
    if slam:
        slam_actions.append(LogInfo(msg='[a200_point_nav] Launching SLAM toolbox...'))
        slam_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_nav2_demos, 'launch', 'slam.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'setup_path': setup_path,
                'scan_topic': scan_filtered_topic,
            }.items()
        ))
    else:
        if not map_file:
            map_file = os.path.join(pkg_nav_worlds, 'maps', f'{world}.yaml')
        slam_actions.append(LogInfo(msg=f'[a200_point_nav] Launching Localization with map: {map_file}'))
        slam_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'setup_path': setup_path,
                'scan_topic': scan_filtered_topic,
                'map': map_file,
            }.items()
        ))

    map_gate_node = Node(
        package='nav_worlds',
        executable='readiness_gate.py',
        name='map_gate',
        namespace=namespace,
        parameters=[{
            'namespace': namespace,
            'stage': 'map',
            'slam': slam,
            'initial_x': LaunchConfiguration('x'),
            'initial_y': LaunchConfiguration('y'),
            'initial_yaw': LaunchConfiguration('yaw'),
            'use_sim_time': True,
            'timeout_sec': 120.0
        }],
        remappings=[
            ('/tf', f'/{namespace}/tf'),
            ('/tf_static', f'/{namespace}/tf_static'),
        ],
        output='screen'
    )

    # 5. Stage 2: Nav2 stack (launched only once map is active)
    nav2_actions = [
        LogInfo(msg='[a200_point_nav] Map active. Launching Nav2 stack...'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'setup_path': setup_path,
                'scan_topic': scan_filtered_topic,
            }.items()
        ),
        LogInfo(msg=f'[a200_point_nav] READY. Send goals with: ros2 run nav_worlds send_goal.py X Y [YAW] --ns {namespace}')
    ]

    def on_map_gate_exit(event, context):
        if event.returncode == 0:
            return nav2_actions
        return [LogInfo(msg=f'[a200_point_nav] ERROR: Map gate failed with code {event.returncode}. Aborting Nav2 bringup.')]

    start_nav2_on_map = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=map_gate_node,
            on_exit=on_map_gate_exit
        )
    )

    def on_sim_gate_exit(event, context):
        if event.returncode == 0:
            return [
                *slam_actions,
                map_gate_node,
                start_nav2_on_map
            ]
        return [LogInfo(msg=f'[a200_point_nav] ERROR: Simulation readiness gate failed with code {event.returncode}. Aborting navigation bringup.')]

    start_slam_on_sim = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=sim_gate_node,
            on_exit=on_sim_gate_exit
        )
    )

    return [
        sim_launch,
        scan_filter_node,
        sim_gate_node,
        start_slam_on_sim
    ]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])

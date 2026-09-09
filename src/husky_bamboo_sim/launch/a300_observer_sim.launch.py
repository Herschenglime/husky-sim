"""A300 observer simulation with Clearpath stock worlds and full teleop.

Stages ``a300_observer.yaml`` into ``setup_path``, starts Gazebo with one of the
six stock Clearpath worlds, spawns the A300 observer platform, and optionally
launches keyboard teleop plus a debug health monitor.

Usage::

    ros2 launch husky_bamboo_sim a300_observer_sim.launch.py world:=warehouse
    ros2 launch husky_bamboo_sim a300_observer_sim.launch.py world:=orchard keyboard_teleop:=true
"""

from __future__ import annotations

import filecmp
import os
import re
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

VALID_WORLDS = (
    'construction',
    'office',
    'orchard',
    'pipeline',
    'solar_farm',
    'warehouse',
)

_DEFAULT_SETUP = os.path.join(os.path.expanduser('~'), 'clearpath')

# Open spawn poses per world (0,0 is inside geometry or off-terrain for several maps).
WORLD_SPAWN_POSES = {
    'warehouse': {'x': '0.0', 'y': '0.0', 'z': '0.3', 'yaw': '0.0'},
    # Large open room west of the central corridor (not the outdoor pad by the dock).
    'office': {'x': '-0.5', 'y': '1.0', 'z': '0.3', 'yaw': '0.0'},
    'construction': {'x': '15.0', 'y': '-3.0', 'z': '0.3', 'yaw': '2.36'},
    'orchard': {'x': '-8.0', 'y': '-8.0', 'z': '0.3', 'yaw': '0.0'},
    'pipeline': {'x': '0.0', 'y': '8.0', 'z': '0.3', 'yaw': '0.0'},
    'solar_farm': {'x': '-28.0', 'y': '-12.0', 'z': '0.5', 'yaw': '0.0'},
}

# Force NVIDIA EGL/GLX for Gazebo sensor rendering (gpu_lidar, cameras).
_NVIDIA_EGL_VENDOR = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
_GPU_ENV = {
    '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
    '__NV_PRIME_RENDER_OFFLOAD': '1',
    '__EGL_VENDOR_LIBRARY_FILENAMES': _NVIDIA_EGL_VENDOR,
    'LIBGL_ALWAYS_SOFTWARE': '0',
}


def _gpu_env_actions():
    return [SetEnvironmentVariable(name=k, value=v) for k, v in _GPU_ENV.items()]


def _prepare_robot_yaml(context, *args, **kwargs):
    """Stage the bundled A300 observer robot.yaml for the generator chain."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    os.makedirs(setup_path, exist_ok=True)
    target = os.path.join(setup_path, 'robot.yaml')
    pkg_share = get_package_share_directory('husky_bamboo_sim')
    source = os.path.join(pkg_share, 'config', 'clearpath', 'a300_observer.yaml')
    if not os.path.isfile(target) or not filecmp.cmp(source, target, shallow=False):
        shutil.copy2(source, target)
    return []


def _prepare_gui_config(context, *args, **kwargs):
    """Patch GUI config: unpause on launch and aim Gazebo teleop at the model cmd_vel topic."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    robot_yaml = os.path.join(setup_path, 'robot.yaml')
    clearpath_gz_share = get_package_share_directory('clearpath_gz')
    source = os.path.join(clearpath_gz_share, 'config', 'gui.config')
    target = os.path.join(setup_path, 'gui.config')

    namespace = 'a300_00000'
    if os.path.isfile(robot_yaml):
        namespace = ClearpathConfig(robot_yaml).system.namespace

    with open(source, encoding='utf-8') as src:
        content = src.read()

    content = content.replace(
        '<start_paused>true</start_paused>',
        '<start_paused>false</start_paused>',
    )

    # Gazebo Teleop publishes gz.msgs.Twist on Gazebo transport. The ros_gz_bridge
    # subscribes on /{namespace}/cmd_vel (not /model/.../robot/cmd_vel).
    teleop_topic = f'/{namespace}/cmd_vel'
    content = re.sub(
        r'(<plugin filename="Teleop">)\s*(?:<topic>[^<]*</topic>\s*)?',
        rf'\1\n    <topic>{teleop_topic}</topic>\n',
        content,
        count=1,
        flags=re.DOTALL,
    )

    with open(target, 'w', encoding='utf-8') as dst:
        dst.write(content)
    return []


def _gz_sim_setup(context, *args, **kwargs):
    """Start Gazebo with a stock Clearpath world, GPU rendering, and patched GUI config."""
    clearpath_gz_share = get_package_share_directory('clearpath_gz')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    packages_paths = [
        os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':') if p
    ]
    path_value = ':'.join([
        os.path.join(clearpath_gz_share, 'worlds'),
        os.path.join(clearpath_gz_share, 'meshes'),
        *packages_paths,
    ])

    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    world_name = LaunchConfiguration('world').perform(context)
    gui_config = os.path.join(setup_path, 'gui.config')
    auto_start = LaunchConfiguration('auto_start').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    gpu_rendering = LaunchConfiguration('gpu_rendering').perform(context)

    auto_start_option = ' -r' if auto_start == 'true' else ''
    server_only_option = ' -s' if gui == 'false' else ''
    gui_config_option = f' --gui-config {gui_config}' if gui == 'true' else ''
    # Headless EGL rendering on the sim server keeps GPU lidar/cameras off the GUI thread.
    headless_option = ' --headless-rendering' if gpu_rendering == 'true' else ''
    render_engine_option = (
        ' --render-engine-server ogre2 --render-engine-gui ogre2'
        if gpu_rendering == 'true' else ''
    )
    gz_args = (
        f'{world_name}.sdf{auto_start_option}{headless_option}{render_engine_option}'
        f' -v 2{gui_config_option}{server_only_option}'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    return [
        *_gpu_env_actions(),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=path_value),
        gz_sim,
        clock_bridge,
    ]


def _should_generate(context) -> str:
    """Skip generation when ~/clearpath already has platform launch files."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    generate = LaunchConfiguration('generate').perform(context)
    if generate != 'true':
        return 'false'
    platform_launch = os.path.join(setup_path, 'platform/launch/platform-service.launch.py')
    if os.path.isfile(platform_launch):
        return 'false'
    return 'true'


def _recovery_spawner_setup(context, *args, **kwargs):
    """Re-spawn controllers only if Clearpath's spawner timed out before gz_ros_control loaded."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    namespace = ClearpathConfig(os.path.join(setup_path, 'robot.yaml')).system.namespace
    delay = float(LaunchConfiguration('spawn_delay').perform(context)) + 75.0
    script = os.path.join(
        get_package_share_directory('husky_bamboo_sim'),
        '..', '..', 'lib', 'husky_bamboo_sim', 'ensure_controllers.sh',
    )
    ensure = ExecuteProcess(
        cmd=['bash', script, namespace, 'joint_state_broadcaster', 'platform_velocity_controller'],
        output='screen',
    )
    return [TimerAction(period=delay, actions=[ensure])]


def _spawn_pose(context) -> tuple[str, str, str, str]:
    """Return (x, y, z, yaw) strings for the active world or explicit args."""
    world_name = LaunchConfiguration('world').perform(context)
    if LaunchConfiguration('auto_spawn_pose').perform(context) == 'true':
        pose = WORLD_SPAWN_POSES.get(world_name, WORLD_SPAWN_POSES['warehouse'])
        return pose['x'], pose['y'], pose['z'], pose['yaw']
    return (
        LaunchConfiguration('x').perform(context),
        LaunchConfiguration('y').perform(context),
        LaunchConfiguration('z').perform(context),
        LaunchConfiguration('yaw').perform(context),
    )


def _write_amcl_params(context, namespace: str) -> str:
    """Clearpath's A300 AMCL config, seeded with the spawn pose as the initial pose.

    The generated maps share Gazebo's world frame, so the spawn pose is the true
    initial map pose; seeding AMCL with it avoids a manual "2D Pose Estimate".
    """
    src = os.path.join(get_package_share_directory('clearpath_nav2_demos'),
                       'config', 'a300', 'localization.yaml')
    with open(src, encoding='utf-8') as handle:
        params = yaml.safe_load(handle)
    x, y, _z, yaw = _spawn_pose(context)
    amcl = params['amcl']['ros__parameters']
    amcl['scan_topic'] = f'/{namespace}/sensors/lidar2d_0/scan'
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': float(x), 'y': float(y), 'z': 0.0, 'yaw': float(yaw)}
    out = os.path.join(tempfile.gettempdir(),
                       f'a300_amcl_{LaunchConfiguration("world").perform(context)}.yaml')
    with open(out, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(params, handle)
    return out


def _rviz_map_setup(context, *args, **kwargs):
    """RViz (Nav2 layout) + map for the active world.

    With localization:=true (default) Nav2's map_server + AMCL provide map->odom,
    seeded at the spawn pose. Otherwise a static map->odom transform equal to the
    spawn pose lines the map up with the robot (drifts with odometry).
    """
    if LaunchConfiguration('rviz_map').perform(context) != 'true':
        return []

    namespace = LaunchConfiguration('robot_namespace').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    pkg_share = get_package_share_directory('husky_bamboo_sim')
    map_yaml = os.path.join(pkg_share, 'maps', f'{world_name}.yaml')
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    localization = (os.path.isfile(map_yaml)
                    and LaunchConfiguration('localization').perform(context) == 'true')

    actions = []
    if localization:
        actions.append(GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'localization_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': _write_amcl_params(context, namespace),
                    'autostart': 'true',
                }.items(),
            ),
        ]))
    elif os.path.isfile(map_yaml):
        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
            remappings=tf_remaps,
        ))
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            namespace=namespace,
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,
                'use_sim_time': use_sim_time,
            }],
        ))
    else:
        print(f'[a300_observer_sim] no map for world "{world_name}" at {map_yaml}; '
              'RViz will start without a map layer')

    if not localization and LaunchConfiguration('static_map_tf').perform(context) == 'true':
        x, y, _z, yaw = _spawn_pose(context)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            namespace=namespace,
            output='screen',
            arguments=[
                '--x', x, '--y', y, '--z', '0.0', '--yaw', yaw,
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=tf_remaps,
        ))

    # Clearpath's nav2.rviz with the RobotModel reader set to Transient Local so it
    # picks up the latched robot_description even though RViz starts later.
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'a300_nav2.rviz')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('clearpath_viz'),
            'launch', 'view_navigation.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config': rviz_config,
        }.items(),
    ))

    delay = float(LaunchConfiguration('spawn_delay').perform(context))
    return [TimerAction(period=delay, actions=actions)]


def _robot_spawn_setup(context, *args, **kwargs):
    """Spawn the robot using world-appropriate pose after Gazebo has loaded."""
    pkg_share = get_package_share_directory('husky_bamboo_sim')
    x, y, z, yaw = _spawn_pose(context)

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'a300_robot_spawn.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': LaunchConfiguration('setup_path'),
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'generate': _should_generate(context),
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    )
    delay = float(LaunchConfiguration('spawn_delay').perform(context))
    return [TimerAction(period=delay, actions=[robot_spawn])]


def _debug_monitor_setup(context, *args, **kwargs):
    if LaunchConfiguration('debug_monitor').perform(context) != 'true':
        return []
    pkg_lib = os.path.join(
        get_package_share_directory('husky_bamboo_sim'),
        '..', '..', 'lib', 'husky_bamboo_sim', 'sim_debug_monitor.sh',
    )
    namespace = LaunchConfiguration('robot_namespace').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    delay = float(LaunchConfiguration('spawn_delay').perform(context)) + 35.0
    monitor = ExecuteProcess(
        cmd=['bash', pkg_lib, namespace, world_name, '5'],
        output='screen',
    )
    return [TimerAction(period=delay, actions=[monitor])]


def generate_launch_description():
    keyboard_teleop = Node(
        package='husky_bamboo_sim',
        executable='sim_keyboard_teleop.py',
        name='sim_keyboard_teleop',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'namespace': LaunchConfiguration('robot_namespace'),
            'linear_speed': 0.6,
            'angular_speed': 1.0,
        }],
        condition=IfCondition(LaunchConfiguration('keyboard_teleop')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'setup_path',
            default_value=_DEFAULT_SETUP,
            description='Clearpath config directory (a300_observer.yaml staged as robot.yaml)',
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='a300_00000',
            description='ROS namespace from robot.yaml (serial a300-00000)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            choices=list(VALID_WORLDS),
            description='Stock Clearpath Gazebo world',
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'generate',
            default_value='false',
            choices=['true', 'false'],
            description='Regenerate Clearpath launch/params (needs system python3-apt)',
        ),
        DeclareLaunchArgument(
            'spawn_delay',
            default_value='35.0',
            description='Seconds to wait for Gazebo world load before robot spawn',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Pass -r to gz sim so physics runs on launch',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set false to run gz sim server-only (-s, no GUI)',
        ),
        DeclareLaunchArgument(
            'gpu_rendering',
            default_value='true',
            description='Use NVIDIA GPU + headless EGL rendering for sensors (recommended)',
        ),
        DeclareLaunchArgument(
            'auto_spawn_pose',
            default_value='true',
            description='Use per-world spawn pose (set false to use x/y/z/yaw args)',
        ),
        DeclareLaunchArgument(
            'keyboard_teleop',
            default_value='true',
            description='Launch terminal keyboard teleop (WASD + QE)',
        ),
        DeclareLaunchArgument(
            'debug_monitor',
            default_value='true',
            description='Launch periodic sim health monitor',
        ),
        DeclareLaunchArgument(
            'rviz_map',
            default_value='false',
            description='Launch RViz (Nav2 layout) with the world map and sensor topics',
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='true',
            description='With rviz_map, run Nav2 map_server + AMCL (2D lidar) seeded '
                        'at the spawn pose to provide map->odom',
        ),
        DeclareLaunchArgument(
            'static_map_tf',
            default_value='true',
            description='With rviz_map and localization:=false, publish static '
                        'map->odom at the spawn pose instead of AMCL',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_prepare_robot_yaml),
        OpaqueFunction(function=_prepare_gui_config),
        OpaqueFunction(function=_gz_sim_setup),
        OpaqueFunction(function=_robot_spawn_setup),
        OpaqueFunction(function=_recovery_spawner_setup),
        OpaqueFunction(function=_rviz_map_setup),
        TimerAction(period=8.0, actions=[keyboard_teleop]),
        OpaqueFunction(function=_debug_monitor_setup),
    ])

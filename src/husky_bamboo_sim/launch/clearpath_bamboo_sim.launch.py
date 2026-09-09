"""Bamboo-field sim layer: single Clearpath Gazebo entry point.

Uses ``ros_gz_sim`` + ``robot_spawn.launch.py`` directly (``simulation.launch.py``
rejects custom worlds like ``bamboo_field``; ``clearpath_gz gz_sim.launch.py``
overwrites ``GZ_SIM_RESOURCE_PATH`` without the bamboo worlds directory).

Do NOT launch ``clearpath_gz simulation.launch.py`` separately while this file
is included from ``bamboo_field_mission.launch.py``.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Force NVIDIA EGL/GLX for Gazebo Sim server headless sensor rendering (gpu_lidar,
# cameras). Without these, Ogre2 may probe EGL_MESA_device_software before GLX.
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
    """Stage Clearpath stock a200_observer.yaml as robot.yaml for the generate chain."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    os.makedirs(setup_path, exist_ok=True)
    target = os.path.join(setup_path, 'robot.yaml')
    pkg_share = get_package_share_directory('husky_bamboo_sim')
    source = os.path.join(pkg_share, 'config', 'clearpath', 'a200_observer.yaml')
    shutil.copy2(source, target)
    return []


def _prepare_gui_config(context, *args, **kwargs):
    """Patch Clearpath GUI config: unpause on launch and aim teleop at the spawned model."""
    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    os.makedirs(setup_path, exist_ok=True)

    clearpath_gz_share = get_package_share_directory('clearpath_gz')
    source = os.path.join(clearpath_gz_share, 'config', 'gui.config')
    target = os.path.join(setup_path, 'gui.config')

    with open(source, encoding='utf-8') as src:
        content = src.read()

    # Stock clearpath gui.config sets start_paused=true, which re-pauses after -r.
    content = content.replace(
        '<start_paused>true</start_paused>',
        '<start_paused>false</start_paused>',
    )

    teleop_topic = f'/model/{robot_namespace}/robot/cmd_vel'
    if '<plugin filename="Teleop">' in content and '<topic>' not in content.split(
        '<plugin filename="Teleop">', maxsplit=1
    )[1].split('</plugin>', maxsplit=1)[0]:
        content = content.replace(
            '<plugin filename="Teleop">',
            f'<plugin filename="Teleop">\n    <topic>{teleop_topic}</topic>',
            1,
        )

    with open(target, 'w', encoding='utf-8') as dst:
        dst.write(content)
    return []


def _gz_sim_setup(context, *args, **kwargs):
    """Start Gazebo with bamboo world on merged GZ_SIM_RESOURCE_PATH.

    clearpath_gz gz_sim.launch.py overwrites GZ_SIM_RESOURCE_PATH without the
    bamboo worlds directory, so we call ros_gz_sim directly with an absolute
    world path and prepend bamboo assets to the resource path first.
    """
    pkg_share = get_package_share_directory('husky_bamboo_sim')
    clearpath_gz_share = get_package_share_directory('clearpath_gz')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    packages_paths = [
        os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH', '').split(':') if p
    ]
    path_value = ':'.join([
        os.path.join(pkg_share, 'worlds'),
        os.path.join(clearpath_gz_share, 'worlds'),
        os.path.join(clearpath_gz_share, 'meshes'),
        *packages_paths,
    ])
    os.environ['GZ_SIM_RESOURCE_PATH'] = path_value

    setup_path = LaunchConfiguration('setup_path').perform(context).rstrip('/')
    world_name = LaunchConfiguration('world').perform(context)
    world_sdf = os.path.join(pkg_share, 'worlds', f'{world_name}.sdf')
    gui_config = os.path.join(setup_path, 'gui.config')
    auto_start = LaunchConfiguration('auto_start').perform(context)
    headless_gui = LaunchConfiguration('headless_gui').perform(context)
    gui = LaunchConfiguration('gui').perform(context)

    auto_start_option = ' -r' if auto_start == 'true' else ''
    # Headless EGL is required for GPU sensor rendering on the sim server.
    render_option = ' --headless-rendering' if headless_gui == 'true' or gui == 'false' else ''
    server_only_option = ' -s' if gui == 'false' else ''
    gui_config_option = f' --gui-config {gui_config}' if gui == 'true' else ''
    gz_args = (
        f'{world_sdf}{auto_start_option}{render_option}{server_only_option}'
        f' -v 4{gui_config_option}'
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
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=path_value),
        gz_sim,
        clock_bridge,
    ]


def generate_launch_description():
    clearpath_gz_share = get_package_share_directory('clearpath_gz')
    default_setup = os.path.join(os.path.expanduser('~'), '.clearpath')

    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    world = LaunchConfiguration('world')
    generate = LaunchConfiguration('generate')

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(clearpath_gz_share, 'launch', 'robot_spawn.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'setup_path': setup_path,
            'world': world,
            'rviz': LaunchConfiguration('rviz'),
            'generate': generate,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    # Large bamboo world: let Gazebo load before Clearpath spawn + bridges.
    robot_spawn_delayed = TimerAction(
        period=35.0,
        actions=[robot_spawn],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'setup_path',
            default_value=default_setup,
            description='Clearpath config directory (stock a200_observer.yaml staged as robot.yaml)',
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='a201_0000',
            description='ROS namespace from robot.yaml (must match Clearpath sim robot)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='bamboo_field',
            description='Gazebo world name (bamboo_field.sdf on GZ_SIM_RESOURCE_PATH)',
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'generate',
            default_value='true',
            choices=['true', 'false'],
            description='Generate Clearpath platform/sensor launch files before spawn',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Pass -r to gz sim so physics runs on launch',
        ),
        DeclareLaunchArgument(
            'headless_gui',
            default_value='false',
            description='Pass --headless-rendering to gz sim (lighter GUI for large worlds)',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set false to run gz sim server-only (-s, no GUI)',
        ),
        DeclareLaunchArgument('x', default_value='-30.0'),
        DeclareLaunchArgument('y', default_value='-30.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        *_gpu_env_actions(),
        OpaqueFunction(function=_prepare_robot_yaml),
        OpaqueFunction(function=_prepare_gui_config),
        OpaqueFunction(function=_gz_sim_setup),
        robot_spawn_delayed,
    ])

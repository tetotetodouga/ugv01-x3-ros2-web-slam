"""Software-only Nav2: one map->odom owner and an isolated velocity output."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('ugv01_room_explore'))
    mode = LaunchConfiguration('mode').perform(context)
    params = LaunchConfiguration('params_file').perform(context)
    if mode not in ('mapping', 'nav_slam', 'nav_map'):
        raise RuntimeError('mode must be mapping, nav_slam or nav_map')
    nodes = []
    if mode in ('mapping', 'nav_slam') and LaunchConfiguration('start_slam').perform(context).lower() == 'true':
        slam_share = Path(get_package_share_directory('slam_toolbox'))
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / 'launch/online_async_launch.py')),
            launch_arguments={
                'slam_params_file': str(share / 'config/slam_async.yaml'),
                'use_sim_time': 'false',
            }.items(),
        ))
    elif mode == 'nav_map':
        map_path = Path(LaunchConfiguration('map').perform(context)).expanduser()
        if not map_path.is_file() or map_path.suffix not in ('.yaml', '.yml'):
            raise RuntimeError('nav_map requires an existing map YAML file')
        nodes.extend([
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[params, {'yaml_filename': str(map_path)}], output='screen'),
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 parameters=[params], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'autostart': True, 'use_sim_time': False,
                              'node_names': ['map_server', 'amcl'], 'bond_timeout': 4.0}]),
        ])
    if mode == 'mapping':
        return nodes
    lifecycle = []
    for package, executable, name in [
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
    ]:
        lifecycle.append(name)
        remaps = [('cmd_vel', '/cmd_vel_nav_raw')]
        if name == 'velocity_smoother':
            remaps.append(('cmd_vel_smoothed', '/cmd_vel_nav'))
        nodes.append(Node(package=package, executable=executable, name=name,
                          output='screen', parameters=[params], remappings=remaps))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'use_sim_time': False,
                     'node_names': lifecycle, 'bond_timeout': 4.0}],
    ))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('ugv01_room_explore'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nav_slam'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/nav2.yaml')),
        OpaqueFunction(function=setup),
    ])

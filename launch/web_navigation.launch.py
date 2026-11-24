"""Hardware runs once; the dashboard owns mapping/localization/Nav2 processes."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('ugv01_room_explore'))
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_model', default_value='LD19'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/web_mapping.launch.py')),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'lidar_model': LaunchConfiguration('lidar_model'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'start_slam': 'false',
            }.items(),
        ),
        ExecuteProcess(
            cmd=['python3', '-u', str(share / 'web/server.py')],
            additional_env={
                'UGV_MANAGED_STACK': '1',
                'WEB_PORT': LaunchConfiguration('web_port'),
            },
            output='screen',
        ),
    ])

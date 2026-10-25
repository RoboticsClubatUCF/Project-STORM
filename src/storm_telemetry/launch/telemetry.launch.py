"""
telemetry.launch.py: rover health publishers.

Included by storm_teleop/teleop.launch.py. Standalone:
    ros2 launch storm_telemetry telemetry.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('storm_telemetry'), 'config', 'telemetry.yaml')

    can_interface = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='SocketCAN interface to monitor.',
    )

    can_health_node = Node(
        package='storm_telemetry',
        executable='can_health_node',
        parameters=[config, {'interface': LaunchConfiguration('can_interface')}],
        output='screen',
    )

    system_monitor_node = Node(
        package='storm_telemetry',
        executable='system_monitor_node',
        parameters=[config],
        output='screen',
    )

    battery_monitor_node = Node(
        package='storm_telemetry',
        executable='battery_monitor_node',
        parameters=[config],
        output='screen',
    )

    spark_monitor_node = Node(
        package='storm_telemetry',
        executable='spark_monitor_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        can_interface,
        can_health_node,
        system_monitor_node,
        battery_monitor_node,
        spark_monitor_node,
    ])

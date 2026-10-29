"""
rover_base.launch.py: always-on rover layer. Runs on the rover at boot
(systemd/storm-rover-base.service).

foxglove_bridge, telemetry and teleop_launcher only. No drivetrain: teleop
starts only when the dashboard calls /storm/teleop/start.

    ros2 launch storm_teleop rover_base.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('storm_basestation'), 'launch', 'bridge.launch.py')),
    )

    telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('storm_telemetry'), 'launch', 'telemetry.launch.py')),
        launch_arguments={'can_interface': LaunchConfiguration('can_interface')}.items(),
    )

    launcher = Node(
        package='storm_teleop',
        executable='teleop_launcher.py',
        name='teleop_launcher',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='SocketCAN interface for the CAN health node.',
        ),
        bridge,
        telemetry,
        launcher,
    ])

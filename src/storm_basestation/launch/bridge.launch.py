"""
bridge.launch.py: foxglove_bridge for Lichtblick. Runs on the rover.

Included by storm_teleop/teleop.launch.py. Standalone:
    ros2 launch storm_basestation bridge.launch.py

Lichtblick: Open connection > Foxglove WebSocket > ws://<rover>:8765
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('storm_basestation'), 'config', 'foxglove_bridge.yaml')

    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([bridge])

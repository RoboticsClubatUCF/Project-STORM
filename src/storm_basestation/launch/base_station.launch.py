"""
base_station.launch.py: Runs joy_node on the base station laptop.

Rover runs the rest:  ros2 launch storm_teleop teleop.launch.py use_joy:=false

No build needed on the laptop:
    ros2 launch /path/to/src/storm_basestation/launch/base_station.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _joy_teleop_config():
    # Built workspace first, else the source tree next to this package
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('storm_teleop'), 'config', 'joy_teleop.yaml')
    except Exception:
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', 'storm_teleop', 'config', 'joy_teleop.yaml')


def generate_launch_description():
    device_id = DeclareLaunchArgument(
        'device_id', default_value='0',
        description='Joystick index (0 = first controller).',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            _joy_teleop_config(),
            {'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int)},
        ],
        output='screen',
    )

    return LaunchDescription([
        device_id,
        joy_node,
    ])

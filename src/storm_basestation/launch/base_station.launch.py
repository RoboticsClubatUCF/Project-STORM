"""
base_station.launch.py: Runs joy_node and joy_watchdog on the base station.

Rover runs the rest:  ros2 launch storm_teleop teleop.launch.py use_joy:=false

No build needed on the base station:
    ros2 launch /path/to/src/storm_basestation/launch/base_station.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _storm_teleop_file(*parts):
    # Built workspace first, else the source tree next to this package
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('storm_teleop'), *parts)
    except Exception:
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', 'storm_teleop', *parts)


def generate_launch_description():
    joy_teleop_config = _storm_teleop_file('config', 'joy_teleop.yaml')

    device_id = DeclareLaunchArgument(
        'device_id', default_value='0',
        description='Joystick index (0 = first controller).',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joy_teleop_config,
            {'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int)},
        ],
        remappings=[('joy', 'joy/raw')],
        output='screen',
    )

    # Forwards joy/raw to /joy only while the controller is alive
    joy_watchdog = ExecuteProcess(
        cmd=['python3', _storm_teleop_file('scripts', 'joy_watchdog.py'),
             '--ros-args', '--params-file', joy_teleop_config],
        output='screen',
    )

    return LaunchDescription([
        device_id,
        joy_node,
        joy_watchdog,
    ])

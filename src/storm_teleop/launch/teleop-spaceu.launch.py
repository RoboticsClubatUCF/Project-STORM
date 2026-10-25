"""
teleop-spaceu.launch.py: teleop.launch.py with turbo at full speed.

Same stack and args as teleop.launch.py; only the turbo scales change
(config/joy_teleop_spaceu.yaml). Acceleration limits are unchanged.

    ros2 launch storm_teleop teleop-spaceu.launch.py
    ros2 launch storm_teleop teleop-spaceu.launch.py use_joy:=false
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('storm_teleop')

    # Other args (use_joy, can_interface, ...) pass through from the command line
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'teleop.launch.py')),
        launch_arguments={
            'joy_overrides': os.path.join(pkg_dir, 'config', 'joy_teleop_spaceu.yaml'),
        }.items(),
    )

    return LaunchDescription([teleop])

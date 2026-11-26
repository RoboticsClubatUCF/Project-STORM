"""Joystick only; launch instead of autonomous navigation to avoid competing commands."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(share("storm_teleop"), "config", "joystick.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="joy",
                executable="joy_node",
                parameters=[params, {"use_sim_time": LC("use_sim_time")}],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[params, {"use_sim_time": LC("use_sim_time")}],
            ),
        ]
    )

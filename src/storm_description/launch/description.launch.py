"""Publish STORM description and joint transforms; no controller manager here."""

import os
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def setup(context):
    backend = LC("hardware").perform(context)
    if backend not in ("mock", "can"):
        raise RuntimeError("Description launch accepts hardware:=mock or hardware:=can")
    robot = xacro.process_file(
        os.path.join(share("storm_description"), "urdf", "storm.urdf.xacro"),
        mappings={
            "hardware": backend,
            "can_interface": LC("can_interface").perform(context),
        },
    ).toxml()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot, "use_sim_time": LC("use_sim_time")}
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware", default_value="mock", choices=["mock", "can"]
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=setup),
        ]
    )

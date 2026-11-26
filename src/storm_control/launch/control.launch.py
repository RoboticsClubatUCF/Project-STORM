"""Standalone controller manager for CAN or mock hardware, never Gazebo."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )
    drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager-timeout",
            "60",
            "--controller-ros-args",
            "--remap ~/odom:=odom --remap ~/cmd_vel:=cmd_vel",
        ],
        output="screen",
    )

    def activate(event, context):
        if event.returncode:
            raise RuntimeError("Joint state broadcaster failed")
        return [drive]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware", default_value="mock", choices=["mock", "can"]
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        share("storm_description"), "launch", "description.launch.py"
                    )
                ),
                launch_arguments={
                    "hardware": LC("hardware"),
                    "can_interface": LC("can_interface"),
                    "use_sim_time": LC("use_sim_time"),
                }.items(),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    os.path.join(share("storm_control"), "config", "controllers.yaml"),
                    {"use_sim_time": LC("use_sim_time")},
                ],
                remappings=[("~/robot_description", "robot_description")],
                output="screen",
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawner, on_exit=activate)
            ),
            spawner,
        ]
    )

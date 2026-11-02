"""Gazebo alone; it owns the only controller_manager in simulation."""

import os
import shlex
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def setup(context):
    pkg = share("storm_gazebo")
    world = LC("world").perform(context)
    if not os.path.isfile(world):
        raise RuntimeError("Gazebo world does not exist: " + world)
    args = (
        "-r "
        + (
            ""
            if LC("gui").perform(context).lower() == "true"
            else "-s --headless-rendering "
        )
        + shlex.quote(world)
    )
    robot = xacro.process_file(
        os.path.join(pkg, "urdf", "storm.gazebo.xacro"), mappings={"hardware": "gazebo"}
    ).toxml()
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "storm",
            "-topic",
            "robot_description",
            "-z",
            LC("spawn_z").perform(context),
        ],
        output="screen",
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "90"],
        output="screen",
    )
    drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager-timeout", "90"],
        output="screen",
    )

    def after(action):
        def handler(event, context):
            if event.returncode:
                raise RuntimeError("STORM spawn/controller activation failed")
            return [action]

        return handler

    return [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": args}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[
                {
                    "use_sim_time": True,
                    "config_file": os.path.join(pkg, "config", "bridge.yaml"),
                }
            ],
            output="screen",
        ),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after(jsb))),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=after(drive))),
        spawn,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    share("storm_gazebo"), "worlds", "storm_arena.sdf"
                ),
            ),
            DeclareLaunchArgument("spawn_z", default_value="0.03"),
            OpaqueFunction(function=setup),
        ]
    )

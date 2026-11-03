"""STORM simulation + optional Nav2, with arena and real MDRS terrain presets."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def setup(context):
    preset = LC("environment").perform(context)
    truth_pose = LC("truth_pose").perform(context) == "true"
    if truth_pose and LC("mode").perform(context) != "slam":
        raise RuntimeError("truth_pose requires mode:=slam")
    nav = share("storm_navigation")
    gz = share("storm_gazebo")
    world = LC("world").perform(context) or os.path.join(
        gz, "worlds", "urc_mdrs.sdf" if preset == "urc" else "storm_arena.sdf"
    )
    params = LC("params_file").perform(context) or os.path.join(
        nav, "config", "nav2_urc.yaml" if preset == "urc" else "nav2.yaml"
    )
    z = LC("spawn_z").perform(context) or ("0.15" if preset == "urc" else "0.03")

    def include(package, file, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share(package), "launch", file)),
            launch_arguments=arguments.items(),
        )

    actions = [
        include(
            "storm_gazebo",
            "simulation.launch.py",
            {"gui": LC("gui"), "world": world, "spawn_z": z, "truth_pose": LC("truth_pose")},
        )
    ]
    if truth_pose:
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ))
    if LC("navigation").perform(context).lower() == "true":
        slam_params = os.path.join(nav, "config", "slam.yaml")
        if truth_pose:
            slam_params = RewrittenYaml(
                source_file=slam_params,
                param_rewrites={"use_scan_matching": "false", "do_loop_closing": "false",
                                "transform_publish_period": "0.0"},
                convert_types=True,
            ).perform(context)
        actions.append(
            include(
                "storm_navigation",
                "navigation.launch.py",
                {
                    "mode": LC("mode"),
                    "map": LC("map"),
                    "params_file": params,
                    "slam_params_file": slam_params,
                    "use_sim_time": "true",
                },
            )
        )
    if LC("rviz").perform(context).lower() == "true":
        actions.append(
            include("storm_navigation", "rviz.launch.py", {"use_sim_time": "true"})
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment", default_value="arena", choices=["arena", "urc"]
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("navigation", default_value="true"),
            DeclareLaunchArgument("truth_pose", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "mode", default_value="slam", choices=["slam", "localization"]
            ),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("world", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("spawn_z", default_value=""),
            OpaqueFunction(function=setup),
        ]
    )

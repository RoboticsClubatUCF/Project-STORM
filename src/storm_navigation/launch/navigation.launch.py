"""Nav2 only: consume an existing robot, /odom, /scan, and TF."""

import os

import yaml
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
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def setup(context):
    mode = LC("mode").perform(context)
    use_time = LC("use_sim_time").perform(context).lower() == "true"
    params_path = LC("params_file").perform(context)
    if not os.path.isfile(params_path):
        raise RuntimeError("Nav2 params_file does not exist: " + params_path)
    with open(params_path) as stream:
        collision_guard = "collision_monitor" in yaml.safe_load(stream)
    params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            param_rewrites={"use_sim_time": str(use_time).lower()},
            convert_types=True,
        ),
        allow_substs=True,
    )
    localization = []
    if mode == "slam":
        localization = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        share("slam_toolbox"), "launch", "online_async_launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": str(use_time).lower(),
                    "slam_params_file": LC("slam_params_file").perform(context),
                }.items(),
            )
        ]
    elif mode == "localization":
        map_file = LC("map").perform(context)
        if not os.path.isfile(map_file):
            raise RuntimeError("Localization requires map:=/absolute/path/map.yaml")
        localization = [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    params,
                    {"yaml_filename": map_file, "use_sim_time": use_time},
                ],
                output="screen",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_time,
                        "autostart": True,
                        "node_names": ["map_server", "amcl"],
                    }
                ],
                output="screen",
            ),
        ]
    else:
        raise RuntimeError("mode must be slam or localization")
    specs = [
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_velocity_smoother", "velocity_smoother"),
        ("nav2_waypoint_follower", "waypoint_follower"),
    ]
    actions = list(localization)
    if collision_guard:
        specs.append(("nav2_collision_monitor", "collision_monitor"))
        actions.append(Node(
            package="storm_perception", executable="ground_filter", name="ground_filter",
            parameters=[{"use_sim_time": use_time}], output="screen"))
    for package, name in specs:
        remaps = []
        if name in ("controller_server", "behavior_server"):
            remaps = [("cmd_vel", "cmd_vel_nav")]
        if name == "velocity_smoother":
            remaps = [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed",
                       "cmd_vel_checked" if collision_guard else "cmd_vel")]
        actions.append(
            Node(
                package=package,
                executable=name,
                name=name,
                parameters=[params],
                remappings=remaps,
                output="screen",
            )
        )
    actions.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[
                {
                    "use_sim_time": use_time,
                    "autostart": True,
                    "node_names": [name for _, name in specs],
                }
            ],
            output="screen",
        )
    )
    return actions


def generate_launch_description():
    pkg = share("storm_navigation")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode", default_value="slam", choices=["slam", "localization"]
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "params_file", default_value=os.path.join(pkg, "config", "nav2.yaml")
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(pkg, "config", "slam.yaml"),
            ),
            OpaqueFunction(function=setup),
        ]
    )

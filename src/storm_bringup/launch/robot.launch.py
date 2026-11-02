"""Physical/mock base + optional navigation. Actual lidar driver runs separately."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC


def generate_launch_description():
    def include(package, name, args, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share(package), "launch", name)),
            launch_arguments=args.items(),
            condition=condition,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware", default_value="mock", choices=["mock", "can"]
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("navigation", default_value="false"),
            DeclareLaunchArgument(
                "mode", default_value="slam", choices=["slam", "localization"]
            ),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    share("storm_navigation"), "config", "nav2.yaml"
                ),
            ),
            include(
                "storm_control",
                "control.launch.py",
                {
                    "hardware": LC("hardware"),
                    "can_interface": LC("can_interface"),
                    "use_sim_time": "false",
                },
            ),
            include(
                "storm_navigation",
                "navigation.launch.py",
                {
                    "mode": LC("mode"),
                    "map": LC("map"),
                    "params_file": LC("params_file"),
                    "use_sim_time": "false",
                },
                IfCondition(LC("navigation")),
            ),
        ]
    )

"""Physical/mock base with optional VLP-16 and navigation."""

import os
from ipaddress import IPv4Address
from math import isfinite
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    def start_vlp16(context):
        ip = str(IPv4Address(LC("vlp16_ip").perform(context)))
        mount = LC("vlp16_mount").perform(context).split()
        if len(mount) != 6 or not all(isfinite(float(v)) for v in mount):
            raise ValueError("vlp16_mount requires measured x y z roll pitch yaw")
        return [include("storm_sensors", "lidar.launch.py", {
            "lidar": "vlp16",
            "device_ip": ip,
            "mount": " ".join(mount),
            "udp_port": LC("vlp16_udp_port"),
            "ring": LC("vlp16_ring"),
            "calibration": LC("vlp16_calibration"),
        })]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware", default_value="mock", choices=["mock", "can"]
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("navigation", default_value="false"),
            DeclareLaunchArgument("vlp16", default_value="false"),
            DeclareLaunchArgument("vlp16_ip", default_value=""),
            DeclareLaunchArgument("vlp16_mount", default_value=""),
            DeclareLaunchArgument("vlp16_udp_port", default_value="2368"),
            DeclareLaunchArgument("vlp16_ring", default_value="-1"),
            DeclareLaunchArgument("vlp16_calibration", default_value=""),
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
            OpaqueFunction(function=start_vlp16, condition=IfCondition(LC("vlp16"))),
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

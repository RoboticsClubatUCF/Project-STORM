"""Bring up one physical lidar without starting motion or navigation."""

from ipaddress import IPv4Address
from math import isfinite
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    lidar = value("lidar")
    frame = "vlp16_link" if lidar == "vlp16" else "hokuyo_link"
    mount = value("mount").split()
    if mount and (len(mount) != 6 or not all(isfinite(float(x)) for x in mount)):
        raise ValueError("mount requires six finite values: x y z roll pitch yaw")

    if lidar == "vlp16":
        ip = str(IPv4Address(value("device_ip")))
        port = int(value("udp_port"))
        ring = int(value("ring"))
        if not 1 <= port <= 65535 or not -1 <= ring <= 15:
            raise ValueError("Expected UDP port 1..65535 and VLP-16 ring -1..15")
        calibration = value("calibration") or str(
            Path(get_package_share_directory("velodyne_pointcloud"))
            / "params" / "VLP16db.yaml"
        )
        if not Path(calibration).is_file():
            raise ValueError(f"Missing Velodyne calibration: {calibration}")
        nodes = [
            Node(
                package="velodyne_driver", executable="velodyne_driver_node",
                parameters=[{"model": "VLP16", "device_ip": ip, "port": port,
                             "frame_id": frame, "gps_time": False}],
                output="screen",
            ),
            Node(
                package="velodyne_pointcloud", executable="velodyne_transform_node",
                parameters=[{"model": "VLP16", "calibration": calibration,
                             "min_range": 1.0, "max_range": 100.0}],
                output="screen",
            ),
            Node(
                package="velodyne_laserscan", executable="velodyne_laserscan_node",
                parameters=[{"ring": ring}],
                output="screen",
            ),
        ]
    else:
        serial_port = value("serial_port")
        if not serial_port.startswith("/dev/"):
            raise ValueError("Set serial_port to the Hokuyo device under /dev")
        nodes = [
            Node(
                package="urg_node", executable="urg_node_driver",
                parameters=[{"serial_port": serial_port, "laser_frame_id": frame}],
                output="screen",
            )
        ]

    if mount:
        flags = ("--x", "--y", "--z", "--roll", "--pitch", "--yaw")
        args = [item for pair in zip(flags, mount) for item in pair]
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=[*args, "--frame-id", "base_link", "--child-frame-id", frame],
            output="screen",
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lidar", choices=["vlp16", "hokuyo"],
                              description="Physical lidar that publishes scan"),
        DeclareLaunchArgument("device_ip", default_value="",
                              description="Actual VLP-16 IPv4 address"),
        DeclareLaunchArgument("udp_port", default_value="2368"),
        DeclareLaunchArgument("ring", default_value="-1",
                              description="Velodyne scan ring, -1 for driver selection"),
        DeclareLaunchArgument("calibration", default_value="",
                              description="Override the upstream VLP-16 calibration YAML"),
        DeclareLaunchArgument("serial_port", default_value="",
                              description="Hokuyo serial device, preferably /dev/serial/by-id"),
        DeclareLaunchArgument("mount", default_value="",
                              description="Measured base_link to lidar x y z roll pitch yaw"),
        OpaqueFunction(function=setup),
    ])

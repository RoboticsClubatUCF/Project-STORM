"""Enforce measurement limits without replacing missing returns with free space."""
import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import dtype_from_fields


def limit_cloud(message, minimum=1., maximum=100.):
    result = copy.deepcopy(message)
    data = bytearray(message.data)
    dtype = dtype_from_fields(message.fields, point_step=message.point_step)
    dtype = dtype.newbyteorder(">" if message.is_bigendian else "<")
    points = np.ndarray((message.height, message.width), dtype=dtype, buffer=data,
                        strides=(message.row_step, message.point_step))
    radius = np.sqrt(sum(points[name].astype(float)**2 for name in ("x", "y", "z")))
    valid = np.isfinite(radius) & (radius >= minimum) & (radius <= maximum)
    for name in ("x", "y", "z"):
        points[name][~valid] = np.nan
    result.data = bytes(data)
    result.is_dense = False
    return result


def limit_depth(message, minimum=.5, maximum=20.):
    if message.encoding != "32FC1":
        raise ValueError("Expected 32FC1 Gazebo depth")
    result = copy.deepcopy(message)
    data = bytearray(message.data)
    values = np.ndarray((message.height, message.width), dtype=">f4" if message.is_bigendian else "<f4",
                        buffer=data, strides=(message.step, 4))
    valid = np.isfinite(values) & (values >= minimum) & (values <= maximum)
    values[~valid] = np.nan
    result.data = bytes(data)
    return result


class SensorLimits(Node):
    def __init__(self):
        super().__init__("sensor_limits")
        self.cloud_pub = self.create_publisher(PointCloud2, "/velodyne_points", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/simulation/vlp16/points", self.cloud, qos_profile_sensor_data)
        self.create_subscription(Image, "/simulation/camera/depth_unvalidated", self.depth, qos_profile_sensor_data)

    def cloud(self, message):
        try:
            self.cloud_pub.publish(limit_cloud(message))
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Invalid lidar sample withheld: {exc}")

    def depth(self, message):
        try:
            self.depth_pub.publish(limit_depth(message))
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Invalid depth sample withheld: {exc}")


def main():
    rclpy.init()
    node = SensorLimits()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

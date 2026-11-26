"""A 2D view of actual VLP-16 returns, not an additional virtual lidar."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def project_scan(points, minimum=1., maximum=100., bins=720):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    distance = np.linalg.norm(points, axis=1)
    horizontal = np.linalg.norm(points[:, :2], axis=1)
    elevation = np.arctan2(points[:, 2], horizontal)
    valid = (np.isfinite(points).all(axis=1) & (distance >= minimum) & (distance <= maximum)
             & (np.abs(elevation) <= math.radians(1.25)) & (horizontal >= minimum))
    selected = points[valid]
    result = np.full(bins, np.inf, dtype=float)
    angles = np.arctan2(selected[:, 1], selected[:, 0])
    indices = np.floor((angles + math.pi) * bins / (2 * math.pi)).astype(int) % bins
    np.minimum.at(result, indices, horizontal[valid])
    return result


class VlpScan(Node):
    def __init__(self):
        super().__init__("vlp_scan")
        self.publisher = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/velodyne_points", self.cloud, qos_profile_sensor_data)

    def cloud(self, cloud):
        points = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        message = LaserScan()
        message.header = cloud.header
        message.angle_min = -math.pi
        message.angle_increment = 2 * math.pi / 720
        message.angle_max = message.angle_min + 719 * message.angle_increment
        message.range_min, message.range_max = 1., 100.
        message.scan_time = .1
        message.ranges = project_scan(points).astype(np.float32).tolist()
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = VlpScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

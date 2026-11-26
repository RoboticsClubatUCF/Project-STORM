"""Transform VLP-16 samples into the rover frame and publish local ground/objects."""
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from storm_perception.ground import segment


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class GroundFilter(Node):
    def __init__(self):
        super().__init__("ground_filter")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("ground_tolerance", .08)
        self.declare_parameter("obstacle_height", .12)
        self.declare_parameter("max_ground_slope", 25.)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.ground = self.create_publisher(PointCloud2, "/perception/ground", qos_profile_sensor_data)
        self.obstacles = self.create_publisher(PointCloud2, "/perception/obstacles", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/velodyne_points", self.cloud, qos_profile_sensor_data)
        self.last = 0.
        self.last_warning = 0.

    def cloud(self, message):
        now = time.monotonic()
        if now - self.last < .18:
            return
        self.last = now
        try:
            age = self.get_clock().now().nanoseconds - Time.from_msg(message.header.stamp).nanoseconds
            if not 0 <= age <= 1_000_000_000:
                raise ValueError("Input cloud timestamp is stale or in the future")
            frame = self.get_parameter("target_frame").value
            transform = self.buffer.lookup_transform(frame, message.header.frame_id,
                Time.from_msg(message.header.stamp), timeout=Duration(seconds=0.))
            values = point_cloud2.read_points_numpy(message, field_names=("x", "y", "z"), skip_nans=True)
            values = np.asarray(values).reshape(-1, 3)
            t = transform.transform.translation
            points = values @ rotation(transform.transform.rotation).T + [t.x, t.y, t.z]
            ground, obstacles = segment(points,
                self.get_parameter("ground_tolerance").value,
                self.get_parameter("obstacle_height").value,
                self.get_parameter("max_ground_slope").value)
            message.header.frame_id = frame
            self.ground.publish(point_cloud2.create_cloud_xyz32(message.header, ground))
            self.obstacles.publish(point_cloud2.create_cloud_xyz32(message.header, obstacles))
        except (TransformException, ValueError) as exc:
            # Do not publish an empty "clear" scene when ground fitting or TF fails.
            if now - self.last_warning > 3:
                self.get_logger().warning(f"Obstacle observations withheld: {exc}")
                self.last_warning = now


def main():
    rclpy.init()
    node = GroundFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

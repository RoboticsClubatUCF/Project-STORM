"""ROS telemetry and leased, stamped simulator teleoperation."""

import math
import time

import numpy as np
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock as SteadyClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class RosBridge(Node):
    def __init__(self, app):
        super().__init__("storm_dashboard", parameter_overrides=[
            Parameter("use_sim_time", value=True)])
        self.app = app
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel_checked", 10)
        self.cancel_publisher = self.create_publisher(String, "/mission/cancel",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.previous_teleop = False
        self.zero_ticks = 0
        self.lifecycle_clients = {name: self.create_client(GetState, f"/{name}/get_state")
                        for name in ("bt_navigator", "controller_server", "planner_server", "collision_monitor")}
        self.futures = {}
        self.future_started = {}
        self.states = {}
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.controller_future = None
        self.generation = app.sim_generation
        self.wall_clock = SteadyClock(clock_type=ClockType.STEADY_TIME)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
            lambda msg: app.update("collision_state", {"action": msg.action_type, "polygon": msg.polygon_name}), 10)
        self.create_subscription(Odometry, "/simulation/ground_truth/odom", self.truth, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", lambda msg: app.update("clock", {
            "seconds": msg.clock.sec + msg.clock.nanosec / 1e9}), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/gps/fix", self.gnss, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", lambda msg: app.update("scan", {
            "rays": len(msg.ranges)}), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/velodyne_points", lambda msg: app.update("cloud", {
            "points": msg.width * msg.height}), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/perception/obstacles", lambda msg: app.update("objects", {
            "points": msg.width * msg.height}), qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", lambda msg: app.update("imu", {}), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", lambda msg: app.update("rgb", {
            "width": msg.width, "height": msg.height}), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=self.wall_clock)
        self.create_timer(1., self.health, clock=self.wall_clock)

    def cancel_mission(self, mission_id):
        message = String()
        message.data = mission_id
        self.cancel_publisher.publish(message)

    def truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(value) for value in values):
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.app.update("truth", {"x": p.x, "y": p.y, "yaw": yaw,
                                 "source": "Gazebo OdometryPublisher", "frame": msg.header.frame_id})

    def gnss(self, msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.app.update("gnss", {"latitude": msg.latitude, "longitude": msg.longitude,
                                     "valid": msg.status.status >= 0})

    def depth(self, msg):
        if msg.encoding != "32FC1" or msg.step < msg.width * 4 or len(msg.data) < msg.height * msg.step:
            self.app.update("depth", {"finite_pixels": 0, "encoding": msg.encoding})
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        values = np.ascontiguousarray(rows[:, :msg.width * 4]).view(">f4" if msg.is_bigendian else "<f4")
        valid = values[np.isfinite(values) & (values > 0)]
        self.app.update("depth", {"finite_pixels": int(valid.size), "total_pixels": msg.width * msg.height,
                                  "maximum_m": float(valid.max()) if valid.size else None,
                                  "minimum_m": float(valid.min()) if valid.size else None})

    def tick(self):
        with self.app.lock:
            enabled = self.app.teleop["enabled"]
        if enabled or self.previous_teleop or self.zero_ticks > 0:
            linear, angular = self.app.velocity()
            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "base_link"
            command.twist.linear.x = linear
            command.twist.angular.z = angular
            self.publisher.publish(command)
            self.app.update("manual_command", {"stamp": command.header.stamp.sec + command.header.stamp.nanosec / 1e9,
                "linear": linear, "angular": angular})
            if self.previous_teleop and not enabled:
                self.zero_ticks = 5
            elif self.zero_ticks:
                self.zero_ticks -= 1
        self.previous_teleop = enabled
        try:
            transform = self.buffer.lookup_transform("map", "base_footprint", Time())
            age = self.get_clock().now().nanoseconds - Time.from_msg(transform.header.stamp).nanoseconds
            if 0 <= age < 2_000_000_000:
                p = transform.transform.translation
                self.app.update("estimate", {"x": p.x, "y": p.y, "frame": "map"})
        except TransformException:
            pass

    def health(self):
        if self.generation != self.app.sim_generation:
            self.generation = self.app.sim_generation
            for future in self.futures.values():
                future.cancel()
            self.futures.clear()
            self.future_started.clear()
            self.states.clear()
            if self.controller_future:
                self.controller_future.cancel()
                self.controller_future = None
        if self.controller_future and self.controller_future.done():
            try:
                controllers = self.controller_future.result().controller
                if any(c.name == "diff_drive_controller" and c.state == "active" for c in controllers):
                    self.app.update("drive", {"active": True})
            except (RuntimeError, AttributeError):
                pass
            self.controller_future = None
        if self.controller_future is None and self.controller_client.service_is_ready():
            self.controller_future = self.controller_client.call_async(ListControllers.Request())
        for name, client in self.lifecycle_clients.items():
            future = self.futures.get(name)
            if future and not future.done() and time.monotonic() - self.future_started.get(name, 0) > 3:
                future.cancel()
                self.futures.pop(name)
                future = None
            if future and future.done():
                try:
                    self.states[name] = future.result().current_state.id
                except (RuntimeError, AttributeError):
                    self.states[name] = 0
                self.futures.pop(name)
            if name not in self.futures and client.service_is_ready():
                self.futures[name] = client.call_async(GetState.Request())
                self.future_started[name] = time.monotonic()
            if not client.service_is_ready():
                self.states[name] = 0
        self.app.update("lifecycle", {"states": dict(self.states)})
        if all(self.states.get(name) == 3 for name in self.lifecycle_clients):
            self.app.update("nav2", {"active": True})
        if self.states.get("collision_monitor") == 3:
            self.app.update("collision", {"active": True})
        if any("rviz" in name for name, _ in self.get_node_names_and_namespaces()):
            self.app.update("rviz", {"discovered": True})

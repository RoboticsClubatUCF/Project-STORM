#!/usr/bin/env python3
"""Spawn one Gazebo box, hold a forward request, and verify sensor-driven stopping."""
import argparse
import json
import math
import os
import subprocess
import tempfile
import time
import uuid
from pathlib import Path
from urllib.request import Request, urlopen

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    base = f"http://127.0.0.1:{args.port}"

    def state():
        with urlopen(base + "/api/state", timeout=10) as response:
            return json.load(response)

    def command(name, values=None):
        request = Request(base + "/api/commands", data=json.dumps({"command": name, "args": values or {}}).encode(),
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=45) as response:
            return json.load(response)["result"]

    current = state()
    if current["busy"] or current["processes"].get("mission", {}).get("running") or current["teleop"]["enabled"]:
        parser.error("Finish the active test, mission, or manual control first")
    if not all(current["telemetry"].get(key, {}).get("fresh") for key in ("truth", "collision", "objects")):
        parser.error("Open the console's simulator and wait for collision readiness")
    partition = current["processes"]["simulation"]["partition"]
    environment = {**os.environ, "GZ_PARTITION": partition}
    truth = current["telemetry"]["truth"]
    x = truth["x"] + 1.7 * math.cos(truth["yaw"])
    y = truth["y"] + 1.7 * math.sin(truth["yaw"])
    name = "storm_obstacle_test_" + uuid.uuid4().hex[:8]
    rclpy.init()
    node = rclpy.create_node("storm_obstacle_test")
    latest = {}
    node.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                             lambda message: latest.update(collision=message), 10)
    node.create_subscription(TwistStamped, "/cmd_vel", lambda message: latest.update(velocity=message), 10)
    node.create_subscription(PointCloud2, "/perception/obstacles",
                             lambda message: latest.update(objects=message.width * message.height),
                             rclpy.qos.qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                             lambda message: latest.update(grid=message),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    created = False
    try:
        with tempfile.TemporaryDirectory(prefix="storm-obstacle-") as folder:
            path = Path(folder) / "box.sdf"
            path.write_text(f'''<sdf version="1.9"><model name="{name}"><static>true</static>
              <link name="body">
              <collision name="collision"><geometry><box><size>0.6 0.6 1.0</size></box></geometry></collision>
              <visual name="visual"><geometry><box><size>0.6 0.6 1.0</size></box></geometry>
              <material><diffuse>0.9 0.25 0.08 1</diffuse></material></visual></link></model></sdf>''')
            subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", "urc_mission",
                            "-file", str(path), "-name", name, "-x", str(x), "-y", str(y), "-z", "0.5"],
                           env=environment, check=True, timeout=25)
            created = True
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        command("teleop.enable")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            command("teleop.drive", {"linear": .15, "angular": 0})
            rclpy.spin_once(node, timeout_sec=.08)
        final = state()["telemetry"]["truth"]
        collision, velocity, grid = latest.get("collision"), latest.get("velocity"), latest.get("grid")
        box_cells = 0
        if grid:
            for row in range(grid.info.height):
                for col in range(grid.info.width):
                    gx = grid.info.origin.position.x + (col + .5) * grid.info.resolution
                    gy = grid.info.origin.position.y + (row + .5) * grid.info.resolution
                    if math.hypot(gx-x, gy-y) < .6 and grid.data[row*grid.info.width+col] >= 99:
                        box_cells += 1
        result = {"travel_m": math.dist((truth["x"], truth["y"]), (final["x"], final["y"])),
                  "stop_polygon": collision.polygon_name if collision else None,
                  "action": collision.action_type if collision else None,
                  "output_mps": velocity.twist.linear.x if velocity else None,
                  "command_stamp": velocity.header.stamp.sec + velocity.header.stamp.nanosec/1e9 if velocity else None,
                  "obstacle_points": latest.get("objects", 0),
                  "box_occupied_cells": box_cells,
                  "occupied_cells": sum(value >= 99 for value in grid.data) if grid else 0}
        print(json.dumps(result, indent=2), flush=True)
        assert result["travel_m"] > .02, "Rover did not move before obstacle intervention"
        assert result["stop_polygon"] == "StopZone" and result["action"] == 1, "Stop polygon did not trigger"
        assert abs(result["output_mps"]) < .001, "Nonzero output despite nearby obstacle"
        assert result["box_occupied_cells"] > 0 and result["obstacle_points"] > 0, "Box not represented in local perception"
    finally:
        command("teleop.disable")
        if created:
            cleanup = subprocess.run(["gz", "service", "-s", "/world/urc_mission/remove",
                "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                "--req", f'name: "{name}" type: MODEL'], check=False, env=environment, capture_output=True, text=True, timeout=15)
            if cleanup.returncode or "true" not in cleanup.stdout:
                command("simulation.stop")
                print("Scene closed because box removal was not confirmed", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

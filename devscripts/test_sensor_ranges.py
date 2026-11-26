#!/usr/bin/env python3
"""Audit live published measurement bounds without using terrain truth as a sensor."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error("seconds must be in [1,60]")
    rclpy.init(args=[])
    node = rclpy.create_node("storm_sensor_range_audit")
    stats = {key: {"messages": 0, "finite_samples": 0, "minimum_m": None,
                   "maximum_m": None, "violations": 0} for key in ("lidar", "scan", "depth", "local_objects")}

    def record(key, values, minimum, maximum):
        finite = np.asarray(values)[np.isfinite(values)]
        item = stats[key]
        item["messages"] += 1
        item["finite_samples"] += int(finite.size)
        item["violations"] += int(((finite < minimum-1e-4) | (finite > maximum+1e-4)).sum())
        if finite.size:
            low, high = float(finite.min()), float(finite.max())
            item["minimum_m"] = min(item["minimum_m"], low) if item["minimum_m"] is not None else low
            item["maximum_m"] = max(item["maximum_m"], high) if item["maximum_m"] is not None else high

    def cloud(message, local=False):
        points = point_cloud2.read_points_numpy(message, field_names=("x", "y", "z"), skip_nans=True).reshape(-1, 3)
        values = np.linalg.norm(points[:, :2] if local else points, axis=1)
        record("local_objects" if local else "lidar", values, 0 if local else 1, 10 if local else 100)

    def depth(message):
        assert message.encoding == "32FC1"
        values = np.ndarray((message.height, message.width), dtype=">f4" if message.is_bigendian else "<f4",
                            buffer=message.data, strides=(message.step, 4))
        record("depth", values, .5, 20)

    def scan(message):
        assert message.range_min == 1 and message.range_max == 100
        record("scan", message.ranges, 1, 100)

    node.create_subscription(PointCloud2, "/velodyne_points", cloud, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/perception/obstacles", lambda msg: cloud(msg, True), qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth/image_raw", depth, qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        sources = [endpoint.node_name for endpoint in node.get_publishers_info_by_topic("/scan")]
        passed = (all(item["messages"] and not item["violations"] for item in stats.values())
                  and stats["lidar"]["finite_samples"] > 0 and stats["depth"]["finite_samples"] > 0
                  and sources == ["vlp_scan"])
        report = {"status": "passed" if passed else "failed", "stats": stats, "scan_sources": sources,
                  "scope": "Observed live bounds plus software gate tests; not a complete reflectivity/occlusion qualification"}
        print(json.dumps(report, indent=2), flush=True)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
        if not passed:
            parser.exit(1, "Sensor range audit failed\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

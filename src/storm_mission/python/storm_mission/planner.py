#!/usr/bin/env python3
"""Shared terrain planner and geographic mission executor."""

import argparse
import hashlib
import heapq
import itertools
import json
import math
import time
import uuid
from pathlib import Path

import numpy as np
from PIL import Image
from pyproj import Geod, Transformer


class MissionCancelled(Exception):
    """Cooperative cancellation without interrupting a ROS executor callback."""


def coordinates(lat, lon):
    if (isinstance(lat, bool) or isinstance(lon, bool)
            or not isinstance(lat, (int, float)) or not isinstance(lon, (int, float))
            or not math.isfinite(lat) or not math.isfinite(lon)
            or not -90 <= lat <= 90 or not -180 <= lon <= 180):
        raise ValueError("Expected finite WGS84 latitude and longitude in decimal degrees")
    return lat, lon


def load_waypoints(path):
    """Read the versioned, ordered mission format without interpreting executable data."""
    data = json.loads(path.read_text())
    if (not isinstance(data, dict) or set(data) != {"version", "frame", "waypoints"}
            or type(data["version"]) is not int or data["version"] != 1
            or data["frame"] != "WGS84"):
        raise ValueError("Mission requires version 1, frame WGS84, and waypoints")
    entries = data["waypoints"]
    if not isinstance(entries, list) or not entries:
        raise ValueError("Mission requires at least one waypoint")
    names = set()
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {"id", "latitude", "longitude"}:
            raise ValueError("Each waypoint requires only id, latitude, longitude")
        name = entry["id"]
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError("Waypoint ids must be nonempty, unique strings")
        names.add(name)
        coordinates(entry["latitude"], entry["longitude"])
    return entries


def valid_fix(fix):
    """Basic fix validity, independent of ROS imports for regression tests."""
    try:
        coordinates(fix.latitude, fix.longitude)
        return fix.status.status >= 0
    except ValueError:
        return False


def preview(terrain, start, targets, max_slope, max_leg):
    plans = []
    for target in targets:
        end = target["goal_xy"]
        route = terrain.route(start, end, max_slope)
        legs = subdivide(route, max_leg)
        plans.append({**target, "route_length_m": sum(
            math.dist(a, b) for a, b in itertools.pairwise(route)),
            "leg_count": len(legs), "legs_xy_yaw": legs})
        start = end
    return {"waypoints": plans, "route_length_m": sum(p["route_length_m"] for p in plans),
            "leg_count": sum(p["leg_count"] for p in plans)}


class Terrain:
    def __init__(self, directory):
        self.metadata = json.loads((directory / "provenance.json").read_text())
        raster = directory / "mdrs_dem.tif"
        if hashlib.sha256(raster.read_bytes()).hexdigest() != self.metadata["download_sha256"]:
            raise ValueError("Terrain raster checksum does not match its provenance")
        with Image.open(raster) as image:
            self.height = np.array(image, dtype=float)
        if self.height.ndim != 2 or not np.isfinite(self.height).all():
            raise ValueError("Terrain requires a finite, single-band elevation grid")
        self.rows, self.cols = self.height.shape
        self.dx, self.dy = self.metadata["resampled_grid_spacing_m"]
        self.west = self.metadata["extent"]["xmin"] - self.metadata["origin_utm_easting"]
        self.north = self.metadata["extent"]["ymax"] - self.metadata["origin_utm_northing"]

    def project(self, lat, lon):
        coordinates(lat, lon)
        # Gazebo's NavSat origin uses ENU with zero heading. Use that same
        # datum for goals. The DEM is indexed in the resulting world metres.
        altitude = self.metadata["origin_source_elevation_m"]
        transform = Transformer.from_pipeline(
            "+proj=pipeline +step +proj=cart +ellps=WGS84 "
            "+step +proj=topocentric +ellps=WGS84 "
            f"+lat_0={self.metadata['station_latitude']} "
            f"+lon_0={self.metadata['station_longitude']} +h_0={altitude}"
        )
        east, north, _ = transform.transform(lon, lat, altitude)
        return east, north

    def cell(self, xy):
        x, y = xy
        row = math.floor((self.north - y) / self.dy)
        col = math.floor((x - self.west) / self.dx)
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            raise ValueError(f"Point {xy} lies outside the terrain")
        return row, col

    def xy(self, cell):
        row, col = cell
        return self.west + (col + 0.5) * self.dx, self.north - (row + 0.5) * self.dy

    def route(self, start, goal, max_slope):
        first, last = self.cell(start), self.cell(goal)
        if first == last:
            return [start, goal]
        slope_limit = math.tan(math.radians(max_slope))

        def grade(a, b):
            return abs(self.height[a] - self.height[b]) / math.dist(self.xy(a), self.xy(b))

        queue = [(math.dist(self.xy(first), self.xy(last)), 0.0, first)]
        costs = {first: 0.0}
        previous = {}
        while queue:
            _, cost, cell = heapq.heappop(queue)
            if cost != costs[cell]:
                continue
            if cell == last:
                cells = [cell]
                while cell in previous:
                    cell = previous[cell]
                    cells.append(cell)
                return [start, *(self.xy(c) for c in reversed(cells)), goal]
            row, col = cell
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1),
                           (-1, 1), (1, -1), (1, 1)):
                nxt = row + dr, col + dc
                if not (0 <= nxt[0] < self.rows and 0 <= nxt[1] < self.cols):
                    continue
                slope = grade(cell, nxt)
                if slope > slope_limit:
                    continue
                if dr and dc:
                    sides = ((row + dr, col), (row, col + dc))
                    if any(max(grade(cell, side), grade(side, nxt)) > slope_limit for side in sides):
                        continue
                step = math.hypot(dc * self.dx, dr * self.dy)
                new_cost = cost + step * (1.0 + 4.0 * slope)
                if new_cost < costs.get(nxt, math.inf):
                    costs[nxt] = new_cost
                    previous[nxt] = cell
                    heuristic = math.dist(self.xy(nxt), self.xy(last))
                    heapq.heappush(queue, (new_cost + heuristic, new_cost, nxt))
        raise ValueError("No route found within the configured DEM slope limit")


def subdivide(route, maximum):
    legs = []
    for start, end in itertools.pairwise(route):
        distance = math.dist(start, end)
        if distance < 0.01:
            continue
        count = math.ceil(distance / maximum)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        legs.extend(
            (start[0] + (end[0] - start[0]) * i / count,
             start[1] + (end[1] - start[1]) * i / count, yaw)
            for i in range(1, count + 1)
        )
    return legs


def execute(args, terrain, targets, report, checkpoint):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.msg import Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("storm_mission", parameter_overrides=[
        rclpy.parameter.Parameter("use_sim_time", value=True)])
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    tf = Buffer(node=node)
    listener = TransformListener(tf, node)
    latest = {}
    cancel_requested = False
    monitor_cancel = True

    def on_cancel(message):
        nonlocal cancel_requested
        if message.data == args.mission_id:
            cancel_requested = True

    node.create_subscription(String, "/mission/cancel", on_cancel,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    route_publisher = node.create_publisher(
        NavPath, "mission/route", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid_topic = "/global_costmap/costmap"
    node.create_subscription(
        OccupancyGrid, grid_topic, lambda msg: latest.update(grid=msg),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    from rclpy.qos import qos_profile_sensor_data
    def on_fix(msg):
        latest.update(fix=msg, fix_received=time.monotonic())

    node.create_subscription(NavSatFix, "/gps/fix", on_fix, qos_profile_sensor_data)
    monitor_gnss = False

    def get_fix():
        fix = latest.get("fix")
        if (fix is not None and valid_fix(fix) and fresh(fix.header.stamp)
                and time.monotonic() - latest["fix_received"] <= 5.0):
            return fix
        return None

    def wait_for(predicate, seconds, reason):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if monitor_cancel and cancel_requested:
                raise MissionCancelled("Mission cancellation requested")
            if monitor_gnss and get_fix() is None:
                raise RuntimeError("GNSS fix lost, invalid, or stale; cancelling mission")
            value = predicate()
            if value is not None:
                return value
            rclpy.spin_once(node, timeout_sec=0.1)
        raise RuntimeError(reason)

    def pose():
        try:
            tr = tf.lookup_transform("map", "base_footprint", Time())
        except TransformException:
            return None
        if not fresh(tr.header.stamp):
            return None
        return tr.transform.translation.x, tr.transform.translation.y

    def fresh(stamp):
        now = node.get_clock().now().nanoseconds
        age = now - Time.from_msg(stamp).nanoseconds
        return now > 0 and 0 <= age <= 2_000_000_000

    def goal_inside_grid(x, y):
        grid = latest.get("grid")
        if grid is None or grid.header.frame_id != "map" or not fresh(grid.header.stamp):
            return None
        origin = grid.info.origin.position
        resolution = grid.info.resolution
        col = math.floor((x - origin.x) / resolution)
        row = math.floor((y - origin.y) / resolution)
        if 0 <= col < grid.info.width and 0 <= row < grid.info.height:
            value = grid.data[row * grid.info.width + col]
            if 0 <= value < 99 or (value == -1 and args.allow_unobserved):
                return True
        return None

    def reachable_step(x, y):
        here = pose()
        if here is None:
            return None
        if goal_inside_grid(x, y):
            return x, y, True
        distance = math.dist(here, (x, y))
        for step in np.arange(min(distance - 0.5, args.max_leg), 0.74, -0.5):
            fraction = step / distance
            candidate = (here[0] + fraction * (x - here[0]),
                         here[1] + fraction * (y - here[1]))
            if goal_inside_grid(*candidate):
                return *candidate, False
        return None

    pending = active = result = None

    def send_leg(x, y, yaw, index, total):
        nonlocal pending, active, result
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        print(f"Leg {index}/{total}: ({x:.2f}, {y:.2f})", flush=True)
        pending = client.send_goal_async(goal)
        active = wait_for(lambda: pending.result() if pending.done() else None,
                          10, "Nav2 did not acknowledge the leg")
        pending = None
        if not active.accepted:
            raise RuntimeError("Nav2 rejected the leg")
        result = active.get_result_async()

    def run_target(target):
        nonlocal active, result
        endpoint = target["goal_xy"]
        start = wait_for(pose, 15, "No fresh map to base_footprint transform")
        legs = subdivide(terrain.route(start, endpoint, args.max_slope), args.max_leg)
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = node.get_clock().now().to_msg()
        for x, y in [start, *((x, y) for x, y, _ in legs)]:
            point = PoseStamped()
            point.header = path.header
            point.pose.position.x, point.pose.position.y = float(x), float(y)
            point.pose.orientation.w = 1.0
            path.poses.append(point)
        route_publisher.publish(path)
        print(f"Waypoint {target['id']}: executing {len(legs)} legs", flush=True)
        for index, (x, y, yaw) in enumerate(legs, 1):
            for _ in range(20):
                gx, gy, reached = wait_for(
                    lambda x=x, y=y: reachable_step(x, y), 12,
                    "No observed free approach to next waypoint",
                )
                send_leg(gx, gy, yaw, index, len(legs))
                outcome = wait_for(lambda future=result: future.result() if future.done() else None,
                                   args.leg_timeout, "Nav2 leg timed out")
                active = result = None
                if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f"Nav2 leg {index} ended with status {outcome.status}")
                if reached:
                    break
            else:
                raise RuntimeError("Waypoint approach limit reached")
        final = wait_for(pose, 5, "No fresh final pose")
        error = math.dist(final, endpoint)
        if error > args.arrival_radius:
            raise RuntimeError(f"Map-frame arrival criterion not met: {error:.3f} m")
        gnss_error = None
        if "goal_gnss" in target:
            fix = wait_for(get_fix, 5, "No fresh valid GNSS fix for final scoring")
            lat, lon = target["goal_gnss"]
            _, _, gnss_error = Geod(ellps="WGS84").inv(fix.longitude, fix.latitude, lon, lat)
            if gnss_error > args.arrival_radius:
                raise RuntimeError(f"GNSS arrival criterion not met: {gnss_error:.3f} m")
        return {"map_error_m": error, "gnss_error_m": gnss_error, "leg_count": len(legs)}

    try:
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError("Nav2 NavigateToPose server unavailable")
        wait_for(
            lambda: True if any(
                endpoint.node_name == "storm_truth_bridge"
                for endpoint in node.get_publishers_info_by_topic("/odom")
            ) else None,
            10, "Execution requires the MDRS simulation with truth_pose:=true",
        )
        start = wait_for(pose, 15, "No fresh map to base_footprint transform")
        # Validate the entire ordered itinerary before sending the first action.
        preview(terrain, start, targets, args.max_slope, args.max_leg)
        if any("goal_gnss" in target for target in targets):
            wait_for(get_fix, 15, "No fresh valid GNSS fix before mission start")
            monitor_gnss = True
        report["status"] = "running"
        checkpoint()
        for target in targets:
            entry = {**target, "status": "running"}
            report["waypoints"].append(entry)
            checkpoint()
            entry.update(run_target(target), status="succeeded")
            print(f"Waypoint {target['id']} reached: map error {entry['map_error_m']:.3f} m, "
                  f"GNSS error {entry['gnss_error_m']}", flush=True)
            checkpoint()
        print("Mission arrival criterion met", flush=True)
    finally:
        # Cancellation must run even when the GNSS watchdog caused the failure.
        monitor_gnss = False
        monitor_cancel = False
        try:
            if pending is not None:
                active = wait_for(lambda: pending.result() if pending.done() else None,
                                  5, "Goal acknowledgement unknown; inspect Nav2 before restarting")
            if active is not None and active.accepted:
                cancel = active.cancel_goal_async()
                wait_for(lambda: cancel.result() if cancel.done() else None, 5,
                         "Nav2 cancellation unconfirmed")
                terminal = result if result is not None else active.get_result_async()
                outcome = wait_for(lambda: terminal.result() if terminal.done() else None, 5,
                                   "Nav2 goal termination unconfirmed")
                report["active_goal_terminated"] = outcome.status in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
        finally:
            del listener
            node.destroy_node()
            rclpy.shutdown()


def main():
    from ament_index_python.packages import get_package_share_directory

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--terrain", type=Path, default=Path(
        get_package_share_directory("storm_gazebo")) / "terrain/mdrs_mission")
    goals = parser.add_mutually_exclusive_group(required=True)
    goals.add_argument("--goal-xy", type=float, nargs=2, metavar=("X", "Y"))
    goals.add_argument("--goal-gnss", type=float, nargs=2, metavar=("LAT", "LON"))
    goals.add_argument("--mission", type=Path, help="Ordered WGS84 waypoint JSON file")
    parser.add_argument("--report", type=Path, help="Create a new JSON execution report (never resume)")
    parser.add_argument("--mission-id", default=uuid.uuid4().hex,
                        help="Cancellation token on /mission/cancel (unique for each execution)")
    parser.add_argument("--start-xy", type=float, nargs=2, default=(0.0, 0.0),
                        help="Planning preview only; execution uses live TF")
    parser.add_argument("--max-slope", type=float, default=15.0)
    parser.add_argument("--max-leg", type=float, default=9.0)
    parser.add_argument("--arrival-radius", type=float, default=1.0)
    parser.add_argument("--leg-timeout", type=float, default=90.0)
    parser.add_argument("--execute", action="store_true",
                        help="Send goals to the simulated rover; default prints a plan")
    parser.add_argument("--allow-unobserved", action="store_true",
                        help="Simulation only: allow DEM waypoints in unknown costmap cells")
    args = parser.parse_args()
    if args.report and not args.execute:
        parser.error("--report requires --execute")
    if not (0 < args.max_slope < 90 and 0 < args.max_leg <= 12
            and 0 < args.arrival_radius <= 1 and 0 < args.leg_timeout < math.inf
            and all(math.isfinite(value) for value in args.start_xy)):
        parser.error("Expected slope in (0,90), leg in (0,12], arrival radius in (0,1], and a finite timeout")
    report = {"version": 1, "status": "starting", "waypoints": [], "mission_id": args.mission_id}
    report_created = False

    def checkpoint():
        if args.report:
            temporary = args.report.with_name(args.report.name + ".tmp")
            temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
            temporary.replace(args.report)

    try:
        terrain = Terrain(args.terrain)
        if args.mission:
            targets = [{"id": entry["id"],
                        "goal_gnss": [entry["latitude"], entry["longitude"]],
                        "goal_xy": terrain.project(entry["latitude"], entry["longitude"])}
                       for entry in load_waypoints(args.mission)]
        elif args.goal_gnss:
            targets = [{"id": "goal", "goal_gnss": args.goal_gnss,
                        "goal_xy": terrain.project(*args.goal_gnss)}]
        else:
            targets = [{"id": "goal", "goal_xy": tuple(args.goal_xy)}]
        if not all(math.isfinite(v) for target in targets for v in target["goal_xy"]):
            raise ValueError("Goals must have finite map coordinates")
        report.update(datum={key: terrain.metadata[key] for key in (
            "station_latitude", "station_longitude", "origin_source_elevation_m")},
            arrival_radius_m=args.arrival_radius, targets=targets)
        if args.execute:
            if args.report:
                # Exclusive creation prevents overwriting an earlier mission record.
                with args.report.open("x") as stream:
                    json.dump(report, stream, indent=2, allow_nan=False)
                report_created = True
            execute(args, terrain, targets, report, checkpoint)
            report["status"] = "succeeded"
        else:
            plan = preview(terrain, tuple(args.start_xy), targets, args.max_slope, args.max_leg)
            # Preserve the original single-goal preview keys for existing tooling.
            if len(targets) == 1:
                plan.update(plan["waypoints"][0])
            print(json.dumps(plan, indent=2, allow_nan=False))
    except (OSError, ValueError, RuntimeError) as exc:
        report.update(status="failed", reason=str(exc))
        parser.exit(1, f"Mission stopped: {exc}\n")
    except (KeyboardInterrupt, MissionCancelled):
        report.update(status="cancelled", reason="Operator interrupted mission")
        parser.exit(130, "Mission interrupted\n")
    finally:
        if report_created:
            if report["waypoints"] and report["waypoints"][-1]["status"] == "running":
                report["waypoints"][-1]["status"] = report["status"]
            checkpoint()


if __name__ == "__main__":
    main()

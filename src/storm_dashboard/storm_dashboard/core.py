"""Application state and bounded simulator operations, shared by HTTP and CLI."""

import copy
import io
import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path
from typing import ClassVar

import numpy as np
from PIL import Image
from pyproj.enums import TransformDirection
from storm_mission.planner import coordinates, preview


class Conflict(ValueError):
    """An operation is incompatible with the active application state."""


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def terminate(process, timeout=12):
    """Stop only the process group created by this application."""
    records, parents = {}, {process.pid}
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit():
            try:
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                records[int(entry.name)] = (int(fields[1]), fields[19])
            except (OSError, ValueError, IndexError):
                pass
    while True:
        new = {pid for pid, (parent, _) in records.items() if parent in parents and pid not in parents}
        if not new:
            break
        parents.update(new)
    children = {pid: records[pid][1] for pid in parents if pid != process.pid and pid in records}

    def alive(pid, identity):
        try:
            fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            return fields[19] == identity and fields[0] not in ("Z", "X")
        except (OSError, IndexError):
            return False

    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
    # Gazebo children create new process groups. Stop only captured descendants
    # whose kernel start time still matches, even if launch has already exited.
    for sig in (signal.SIGTERM, signal.SIGKILL):
        remaining = [pid for pid, identity in children.items() if alive(pid, identity)]
        if not remaining:
            break
        for pid in remaining:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and any(alive(pid, children[pid]) for pid in remaining):
            time.sleep(.1)


def render_terrain(terrain):
    """Generate an offline, georeferenced relief raster from the checked DEM."""
    height = terrain.height
    gy, gx = np.gradient(height, terrain.dy, terrain.dx)
    light = np.clip((1 - gx * 1.5 + gy * 1.5) / np.sqrt(1 + gx * gx + gy * gy), .45, 1.35)
    level = (height - height.min()) / max(float(np.ptp(height)), 1)
    low, high = np.array([190, 173, 142]), np.array([104, 111, 86])
    rgb = (low[None, None, :] * (1 - level[:, :, None]) + high * level[:, :, None])
    rgb *= light[:, :, None]
    contour = np.floor(height / 5)
    edges = np.zeros(height.shape, dtype=bool)
    edges[1:] |= contour[1:] != contour[:-1]
    edges[:, 1:] |= contour[:, 1:] != contour[:, :-1]
    rgb[edges] *= .83
    output = io.BytesIO()
    Image.fromarray(np.uint8(np.clip(rgb, 0, 255))).save(output, format="PNG")
    return output.getvalue()


class Application:
    TESTS: ClassVar[dict] = {
        "sensors": {"name": "Sensor & scene check", "description": "Open Gazebo and RViz; verify truth, GNSS, lidar, RGB and finite depth."},
        "waypoints": {"name": "GNSS outbound + return", "description": "Fresh scene, two GNSS stops, independent arrival checks and report."},
        "deadman": {"name": "Teleop deadman stop", "description": "Drive briefly, expire the input lease, and measure the stopping distance."},
        "delivery": {"name": "Delivery transit practice", "description": "24 m approach and return with live obstacle checks. Navigation only; payload handling is not simulated."},
    }
    COMMANDS = ("status", "plan.add", "plan.set", "plan.clear", "plan.preview", "plan.export",
                "simulation.start", "simulation.stop", "mission.start", "mission.cancel",
                "teleop.enable", "teleop.disable", "teleop.drive", "stop", "test.run", "test.cancel")

    def __init__(self, terrain, mission_share, data_dir, spawn=subprocess.Popen):
        self.terrain = terrain
        self.mission_share = Path(mission_share)
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.spawn = spawn
        self.lock = threading.RLock()
        self.events = deque(maxlen=120)
        self.waypoints = []
        self.plan = None
        self.telemetry = {}
        self.trail = deque(maxlen=1800)
        self.processes = {}
        self.last_mission = None
        self.test = None
        self.test_abort = threading.Event()
        self.test_thread = None
        self.teleop = {"enabled": False, "linear": 0., "angular": 0., "expires": 0.}
        self.bridge = None
        self.closing = False
        self.sim_generation = 0
        self.map_png = render_terrain(terrain)
        self.event("STORM console ready. Plan offline or start the visible simulator.")
        saved = self.data_dir / "plan.json"
        if saved.exists():
            try:
                self.waypoints = self.targets(json.loads(saved.read_text()))
                self.event(f"Restored {len(self.waypoints)} saved waypoint(s).")
            except (OSError, ValueError, TypeError) as exc:
                self.event(f"Saved plan could not be loaded: {exc}")

    def event(self, message):
        with self.lock:
            self.events.append({"time": time.time(), "message": message})

    def update(self, name, value):
        with self.lock:
            self.telemetry[name] = {**value, "received": time.monotonic()}
            if name == "truth":
                point = [value["x"], value["y"]]
                if not self.trail or math.dist(point, self.trail[-1]) > .04:
                    self.trail.append(point)

    def fresh(self, name, age=3):
        value = self.telemetry.get(name)
        return value if value and time.monotonic() - value["received"] < age else None

    def navigation_blockers(self):
        """Explain execution readiness using the same gates as mission start."""
        blockers = []
        if not self.running("simulation"):
            blockers.append("simulator is stopped")
        for name in ("truth", "nav2", "drive", "collision", "objects"):
            if not self.fresh(name):
                blockers.append(f"{name} is not ready")
        lifecycle = self.fresh("lifecycle")
        if lifecycle:
            labels = {0: "unknown", 1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}
            for name, state in lifecycle["states"].items():
                if state != 3:
                    blockers.append(f"{name}: {labels.get(state, str(state))}")
        return blockers

    def busy(self):
        return bool((self.test and self.test["status"] in ("starting", "running", "cancelling"))
                    or (self.test_thread and self.test_thread.is_alive()))

    def running(self, name):
        item = self.processes.get(name)
        return bool(item and item["process"].poll() is None)

    def idle(self, internal=False):
        if self.closing:
            raise Conflict("Application is shutting down")
        if self.running("mission") or (self.busy() and not internal):
            raise Conflict("Cancel the active mission or test first")

    def bounds(self):
        t = self.terrain
        return [t.west, t.north - t.rows * t.dy, t.west + t.cols * t.dx, t.north]

    def geographic(self, x, y):
        # Invert the same ENU transformation used by the mission runner.
        from pyproj import Transformer
        m = self.terrain.metadata
        transform = Transformer.from_pipeline(
            "+proj=pipeline +step +proj=cart +ellps=WGS84 "
            "+step +proj=topocentric +ellps=WGS84 "
            f"+lat_0={m['station_latitude']} +lon_0={m['station_longitude']} "
            f"+h_0={m['origin_source_elevation_m']}")
        lon, lat, _ = transform.transform(x, y, 0, direction=TransformDirection.INVERSE)
        return lat, lon

    def targets(self, entries):
        if not isinstance(entries, list) or len(entries) > 100:
            raise ValueError("waypoints must be an array with at most 100 entries")
        targets, ids = [], set()
        for index, item in enumerate(entries):
            if not isinstance(item, dict):
                raise TypeError("Each waypoint must be an object")
            if set(item) <= {"id", "x", "y"} and {"x", "y"} <= set(item):
                x, y = finite(item["x"], "x"), finite(item["y"], "y")
                lat, lon = self.geographic(x, y)
            elif set(item) <= {"id", "latitude", "longitude"} and {"latitude", "longitude"} <= set(item):
                lat, lon = coordinates(item["latitude"], item["longitude"])
                x, y = self.terrain.project(lat, lon)
            else:
                raise ValueError("Supply x/y or latitude/longitude, with optional id")
            self.terrain.cell((x, y))
            name = item.get("id", f"WP-{index + 1:02d}")
            if not isinstance(name, str) or not name.strip() or len(name) > 80 or name in ids:
                raise ValueError("Waypoint ids must be unique nonempty strings of at most 80 characters")
            ids.add(name)
            targets.append({"id": name, "latitude": lat, "longitude": lon, "x": x, "y": y})
        return targets

    def plan_route(self):
        if not self.waypoints:
            raise ValueError("Add at least one waypoint")
        truth = self.fresh("truth")
        start = (truth["x"], truth["y"]) if truth else (0., 0.)
        targets = [{"id": w["id"], "goal_xy": (w["x"], w["y"]),
                    "goal_gnss": (w["latitude"], w["longitude"])} for w in self.waypoints]
        self.plan = preview(self.terrain, start, targets, 15., 9.)
        self.plan["start_xy"] = start
        self.plan["start_source"] = "Gazebo truth" if truth else "site datum (offline)"
        return self.plan

    def export_plan(self):
        return {"version": 1, "frame": "WGS84", "waypoints": [
            {key: w[key] for key in ("id", "latitude", "longitude")} for w in self.waypoints]}

    def set_plan(self, entries):
        waypoints = self.targets(entries)
        saved = [{key: w[key] for key in ("id", "latitude", "longitude")} for w in waypoints]
        temporary = self.data_dir / "plan.json.tmp"
        temporary.write_text(json.dumps(saved, indent=2, allow_nan=False))
        temporary.replace(self.data_dir / "plan.json")
        self.waypoints = waypoints

    def launch(self, name, arguments, directory, partition=None):
        directory.mkdir(parents=True, exist_ok=True)
        log_path = directory / f"{name}.log"
        with log_path.open("w") as output:
            environment = os.environ.copy()
            if partition:
                environment["GZ_PARTITION"] = partition
            process = self.spawn(arguments, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True, env=environment)
        self.processes[name] = {"process": process, "log": log_path, "directory": directory,
                                "argv": arguments, "started": time.time(), "partition": partition}
        return process

    def start_simulation(self, internal=False):
        self.idle(internal)
        if self.running("simulation"):
            raise Conflict("This console already owns a running simulation")
        if self.fresh("truth"):
            raise Conflict("Another simulator is publishing in this ROS domain. Stop it or use a separate domain")
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise Conflict("No graphical display found. Run inside a desktop session or WSLg")
        self.disable_teleop()
        self.telemetry.clear()
        self.trail.clear()
        self.sim_generation += 1
        directory = self.data_dir / ("scene-" + uuid.uuid4().hex[:10])
        partition = os.environ.get("GZ_PARTITION", "storm_console") + "_" + directory.name
        self.launch("simulation", ["ros2", "launch", "storm_bringup", "simulation.launch.py",
                    "environment:=urc_mission", "truth_pose:=true", "gui:=true", "rviz:=true"], directory, partition)
        self.event("Opening Gazebo + RViz with the MDRS mission terrain. Waiting for live data.")

    def disable_teleop(self):
        self.teleop.update(enabled=False, linear=0., angular=0., expires=0.)

    def drive(self, linear, angular):
        if not self.teleop["enabled"]:
            raise Conflict("Enable teleop before sending drive commands")
        if not self.fresh("truth") or not self.running("simulation"):
            self.disable_teleop()
            raise Conflict("No fresh Gazebo truth from this console's simulator")
        linear, angular = finite(linear, "linear"), finite(angular, "angular")
        if abs(linear) > .3 or abs(angular) > .6:
            raise ValueError("Teleop limits are 0.3 m/s and 0.6 rad/s")
        self.teleop.update(linear=linear, angular=angular, expires=time.monotonic() + .35)

    def velocity(self):
        with self.lock:
            if (not self.teleop["enabled"] or time.monotonic() > self.teleop["expires"]
                    or not self.fresh("truth") or self.running("mission")):
                return 0., 0.
            return self.teleop["linear"], self.teleop["angular"]

    def start_mission(self, internal=False, smoke=False, allow_unobserved=True):
        self.idle(internal)
        if self.teleop["enabled"]:
            raise Conflict("Disable teleop before mission execution")
        blockers = self.navigation_blockers()
        if blockers:
            raise Conflict("Navigation blocked: " + "; ".join(blockers))
        directory = self.data_dir / ("mission-" + uuid.uuid4().hex[:10])
        directory.mkdir(parents=True)
        if smoke:
            mission_path = self.mission_share / "missions/mdrs_smoke.json"
        else:
            self.plan_route()
            mission_path = directory / "waypoints.json"
            mission_path.write_text(json.dumps({"version": 1, "frame": "WGS84", "waypoints": [
                {key: w[key] for key in ("id", "latitude", "longitude")} for w in self.waypoints]}, indent=2))
        executable = self.mission_share.parent.parent / "lib/storm_mission/mission"
        args = [str(executable), "--mission", str(mission_path),
                "--execute", "--report", str(directory / "report.json"), "--mission-id", directory.name]
        if smoke:
            args.extend(["--max-leg", "0.6"])
        if allow_unobserved:
            args.append("--allow-unobserved")
        self.last_mission = str(directory / "report.json")
        self.launch("mission", args, directory)
        self.processes["mission"]["mission_id"] = directory.name
        self.event("Mission started with GNSS arrival checks at each stop.")

    def stop_process(self, name):
        with self.lock:
            item = self.processes.get(name)
        if item:
            if name == "mission" and self.bridge and item["process"].poll() is None:
                deadline = time.monotonic() + 12
                while item["process"].poll() is None and time.monotonic() < deadline:
                    self.bridge.cancel_mission(item["mission_id"])
                    time.sleep(.2)
                if item["process"].poll() is None:
                    self.event("Mission cancellation unconfirmed. Closing this console's simulator to stop motion.")
                    self.stop_process("simulation")
            terminate(item["process"])
            if name == "simulation":
                deadline = time.monotonic() + 4
                while time.monotonic() < deadline:
                    with self.lock:
                        truth = self.fresh("truth", age=.6)
                    if not truth:
                        with self.lock:
                            self.telemetry.clear()
                        break
                    time.sleep(.1)
            self.event(f"{name.capitalize()} process stopped.")

    def report(self):
        if not self.last_mission:
            return None
        try:
            return json.loads(Path(self.last_mission).read_text())
        except (OSError, ValueError):
            return None

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            telemetry = {key: {**value, "age_s": round(now - value["received"], 2),
                               "fresh": now - value["received"] < 3}
                         for key, value in self.telemetry.items()}
            processes = {key: {"running": self.running(key), "exit_code": value["process"].poll(),
                               "log": str(value["log"]), "argv": value["argv"], "partition": value.get("partition")}
                         for key, value in self.processes.items()}
            logs = []
            for name in ("mission", "simulation"):
                if name in self.processes:
                    try:
                        with self.processes[name]["log"].open("rb") as stream:
                            stream.seek(0, 2)
                            stream.seek(max(0, stream.tell() - 5000))
                            logs.append({"source": name, "text": stream.read().decode(errors="replace")})
                    except OSError:
                        pass
            return copy.deepcopy({"telemetry": telemetry, "processes": processes,
                "waypoints": self.waypoints, "plan": self.plan, "trail": list(self.trail),
                "events": list(self.events), "teleop": {"enabled": self.teleop["enabled"],
                "velocity": self.velocity()}, "test": self.test, "report": self.report(),
                "logs": logs, "busy": bool(self.busy()), "tests": self.TESTS,
                "navigation_blockers": self.navigation_blockers()})

    def command(self, name, args):
        if not isinstance(args, dict):
            raise TypeError("args must be an object")
        if name not in self.COMMANDS:
            raise ValueError("Unknown command. Read /api/schema for the allowlist")
        # Stops can wait for child exit without blocking the ROS telemetry lock.
        if name in ("mission.cancel", "simulation.stop", "stop", "test.cancel"):
            with self.lock:
                self.disable_teleop()
                if self.busy():
                    self.test_abort.set()
                    self.test["status"] = "cancelling"
            self.stop_process("mission")
            if name == "simulation.stop":
                self.stop_process("simulation")
            self.event("Stop requested. Teleop lease cleared and active mission cancellation requested.")
            return self.snapshot()
        with self.lock:
            if name == "status":
                return self.snapshot()
            if name.startswith("plan."):
                if name == "plan.export":
                    return self.export_plan()
                self.idle()
                if name == "plan.add":
                    entries = [{k: w[k] for k in ("id", "latitude", "longitude")} for w in self.waypoints]
                    entries.append(args)
                    self.set_plan(entries)
                elif name == "plan.set":
                    self.set_plan(args.get("waypoints"))
                elif name == "plan.clear":
                    self.set_plan([])
                if name == "plan.preview":
                    return copy.deepcopy(self.plan_route())
                self.plan = None
                self.event(f"Plan updated: {len(self.waypoints)} waypoint(s).")
            elif name == "simulation.start":
                self.start_simulation()
            elif name == "mission.start":
                allow = args.get("allow_unobserved", True)
                if type(allow) is not bool:
                    raise ValueError("allow_unobserved must be boolean")
                self.start_mission(allow_unobserved=allow)
            elif name == "teleop.enable":
                self.idle()
                if (not self.running("simulation") or not self.fresh("truth") or not self.fresh("drive")
                        or not self.fresh("collision") or not self.fresh("objects")):
                    raise Conflict("Wait for this console's live simulator before enabling teleop")
                self.teleop["enabled"] = True
                self.event("Teleop enabled. Hold WASD / arrow keys or a direction button to drive.")
            elif name == "teleop.disable":
                self.disable_teleop()
            elif name == "teleop.drive":
                self.idle()
                self.drive(args.get("linear", 0), args.get("angular", 0))
            elif name == "test.run":
                self.idle()
                test_id = args.get("id")
                if test_id not in self.TESTS:
                    raise ValueError("Unknown test id")
                if self.running("simulation"):
                    raise Conflict("Stop the current simulator first. Automated tests start a fresh visible scene")
                self.test_abort.clear()
                self.test = {"id": test_id, "status": "starting", "stage": "Opening Gazebo + RViz", "started": time.time()}
                self.test_thread = threading.Thread(target=self.run_test, args=(test_id,), daemon=True)
                self.test_thread.start()
            return self.snapshot()

    def run_test(self, test_id):
        saved_plan = None
        def wait(predicate, timeout, message):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if self.test_abort.is_set() or self.closing:
                    raise InterruptedError("Test cancelled")
                with self.lock:
                    if not self.running("simulation"):
                        raise RuntimeError("Simulation exited. Inspect the launch log")
                    value = predicate()
                if value:
                    return value
                time.sleep(.1)
            if message == "Simulator, Nav2 or RViz did not become ready":
                with self.lock:
                    details = self.navigation_blockers()
                    if not self.fresh("rviz"):
                        details.append("RViz is not ready")
                message += ": " + "; ".join(details)
            raise RuntimeError(message)

        try:
            with self.lock:
                if self.test_abort.is_set():
                    raise InterruptedError("Test cancelled")
                self.start_simulation(internal=True)
                self.test.update(status="running", stage="Waiting for simulator, Nav2 and RViz")
            wait(lambda: all(self.fresh(key) for key in ("truth", "nav2", "drive", "rviz", "collision", "objects")),
                 100, "Simulator, Nav2 or RViz did not become ready")
            if test_id == "sensors":
                with self.lock:
                    self.test["stage"] = "Checking live sensor streams and depth returns"
                wait(lambda: all(self.fresh(key) for key in ("gnss", "scan", "cloud", "rgb", "depth", "imu"))
                     and self.telemetry["depth"].get("finite_pixels", 0) > 0,
                     40, "Missing sensor stream or no finite depth returns")
                result = {"sensor_check": "passed", "visual_confirmation": "Inspect the open Gazebo and RViz windows"}
            elif test_id in ("waypoints", "delivery"):
                with self.lock:
                    if test_id == "delivery":
                        saved_plan = [{k: w[k] for k in ("id", "latitude", "longitude")} for w in self.waypoints]
                        self.set_plan([{"id": "pickup-approach", "x": 24., "y": 0.},
                                       {"id": "dropoff-approach", "x": 0., "y": 0.}])
                    self.test["stage"] = ("Delivery navigation: 24 m approach and return; no payload model"
                                          if test_id == "delivery" else "Executing GNSS outbound and return itinerary")
                    self.test["simulation_started_s"] = self.telemetry["clock"]["seconds"]
                    self.start_mission(internal=True, smoke=test_id == "waypoints", allow_unobserved=True)
                wait(lambda: not self.running("mission"), 600 if test_id == "delivery" else 200,
                     "Waypoint mission exceeded test deadline")
                result = self.report()
                if not result or result.get("status") != "succeeded" or len(result["waypoints"]) != 2:
                    raise RuntimeError("Waypoint mission failed. Inspect the mission report and log")
                with self.lock:
                    self.test["simulation_elapsed_s"] = (self.telemetry["clock"]["seconds"]
                                                         - self.test["simulation_started_s"])
                    if test_id == "delivery":
                        self.test["scope"] = "Navigation practice only; not pickup/delivery completion or URC scoring"
            else:
                with self.lock:
                    self.test["stage"] = "Driving at 0.15 m/s for two wall seconds"
                    self.teleop["enabled"] = True
                    start = self.fresh("truth")
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    with self.lock:
                        if self.test_abort.is_set():
                            raise InterruptedError("Test cancelled")
                        self.drive(.15, 0.)
                    time.sleep(.1)
                with self.lock:
                    self.test["stage"] = "Letting input expire and measuring stop"
                wait(lambda: time.monotonic() > self.teleop["expires"] + .8, 4, "Input lease did not expire")
                with self.lock:
                    stopped = self.fresh("truth")
                time.sleep(1.2)
                with self.lock:
                    final = self.fresh("truth")
                    self.disable_teleop()
                if not stopped or not final:
                    raise RuntimeError("Truth telemetry lost during stop measurement")
                drift = math.dist((stopped["x"], stopped["y"]), (final["x"], final["y"]))
                travelled = math.dist((start["x"], start["y"]), (stopped["x"], stopped["y"]))
                if drift > .12 or travelled < .03:
                    raise RuntimeError(f"Deadman check failed: travel {travelled:.3f} m, stop drift {drift:.3f} m")
                result = {"distance_m": travelled, "stop_drift_m": drift}
            with self.lock:
                if self.test_abort.is_set():
                    raise InterruptedError("Test cancelled")
                self.test.update(status="passed", stage="Complete — simulator windows remain open", result=result)
                self.event(f"{self.TESTS[test_id]['name']} passed.")
        except Exception as exc:  # noqa: BLE001 -- worker boundary must stop motion and record failures
            with self.lock:
                self.disable_teleop()
            self.stop_process("mission")
            with self.lock:
                self.test.update(status="cancelled" if isinstance(exc, InterruptedError) else "failed", stage=str(exc))
                self.event(f"Test stopped: {exc}")
        finally:
            with self.lock:
                if saved_plan is not None:
                    self.set_plan(saved_plan)
                    self.plan = None
                self.test["finished"] = time.time()
                path = self.data_dir / ("test-" + uuid.uuid4().hex[:10] + ".json")
                path.write_text(json.dumps(self.test, indent=2, allow_nan=False))
                self.test["report_path"] = str(path)

    def close(self):
        with self.lock:
            self.closing = True
            self.test_abort.set()
            self.disable_teleop()
        self.stop_process("mission")
        self.stop_process("simulation")
        if self.test_thread:
            self.test_thread.join(timeout=5)

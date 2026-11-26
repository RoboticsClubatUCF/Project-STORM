"""Loopback HTTP API and offline web assets. No shell command execution endpoint."""

import argparse
import json
import os
import signal
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import yaml
from ament_index_python.packages import get_package_share_directory as share
from storm_mission.planner import Terrain

from storm_dashboard.core import Application, Conflict


def schema():
    return {"version": 1, "command_endpoint": "POST /api/commands",
            "body": {"command": "name", "args": {}}, "commands": {
        "status": {}, "plan.add": {"x": "metres east", "y": "metres north", "id": "optional"},
        "plan.set": {"waypoints": "array of {id?,x,y} or {id?,latitude,longitude}"},
        "plan.clear": {}, "plan.preview": {}, "plan.export": {}, "simulation.start": {}, "simulation.stop": {},
        "mission.start": {"allow_unobserved": "boolean, default true; requires live local collision pipeline"}, "mission.cancel": {},
        "teleop.enable": {}, "teleop.disable": {},
        "teleop.drive": {"linear": "[-0.3,0.3] m/s", "angular": "[-0.6,0.6] rad/s",
                         "lease": "350 ms; refresh while held"},
        "stop": {}, "test.run": {"id": list(Application.TESTS)}, "test.cancel": {}},
        "reads": ["/api/state", "/api/site", "/api/schema", "/api/terrain.png"],
        "transport": "Loopback only. JSON mutations. Host/Origin checks. No CORS."}


def make_handler(app, web_dir):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            if not str(args[0]).startswith("GET /api/state"):
                super().log_message(format, *args)

        def trusted(self):
            allowed = {f"localhost:{self.server.server_port}", f"127.0.0.1:{self.server.server_port}"}
            host = self.headers.get("Host")
            origin = self.headers.get("Origin")
            return host in allowed and (origin is None or origin in {f"http://{value}" for value in allowed})

        def send(self, status, data, content_type="application/json"):
            if not isinstance(data, bytes):
                data = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not self.trusted():
                return self.send(403, {"error": "Local Host/Origin required"})
            path = urlsplit(self.path).path
            if path == "/api/state":
                return self.send(200, app.snapshot())
            if path == "/api/site":
                return self.send(200, {"name": "Mars Desert Research Station", "bounds": app.bounds(),
                    "sensor_limits": yaml.safe_load((Path(share("storm_gazebo")) / "config/sensor_limits.yaml").read_text()),
                    "datum": {"latitude": app.terrain.metadata["station_latitude"],
                              "longitude": app.terrain.metadata["station_longitude"]},
                    "map": "/api/terrain.png", "source": "USGS 3DEP elevation / 5 m contour bands",
                    "axes": "simulation world metres; east / north", "elevation_m": [
                        float(app.terrain.height.min()), float(app.terrain.height.max())]})
            if path == "/api/schema":
                return self.send(200, schema())
            if path == "/api/terrain.png":
                return self.send(200, app.map_png, "image/png")
            assets = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/app.js": ("app.js", "text/javascript"), "/style.css": ("style.css", "text/css")}
            if path in assets:
                file, content_type = assets[path]
                return self.send(200, (web_dir / file).read_bytes(), content_type)
            return self.send(404, {"error": "Unknown endpoint"})

        def do_POST(self):
            if not self.trusted():
                return self.send(403, {"error": "Local Host/Origin required"})
            if self.path != "/api/commands":
                return self.send(404, {"error": "Unknown endpoint"})
            if self.headers.get_content_type() != "application/json":
                return self.send(415, {"error": "application/json required"})
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 65536:
                    raise ValueError("JSON request must be between 1 and 65536 bytes")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict) or set(body) - {"command", "args"}:
                    raise ValueError("Expected command and optional args")
                result = app.command(body.get("command"), body.get("args", {}))
                return self.send(200, {"ok": True, "result": result})
            except Conflict as exc:
                return self.send(409, {"error": str(exc)})
            except (ValueError, TypeError, KeyError) as exc:
                return self.send(400, {"error": str(exc)})
            except Exception as exc:  # noqa: BLE001 -- HTTP boundary returns a logged structured error
                app.event(f"Command failed: {exc}")
                return self.send(500, {"error": str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description="STORM local mission console")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--offline", action="store_true", help="Map and planning only; no ROS or process control")
    parser.add_argument("--data-dir", type=Path, default=Path.home() / ".local/state/storm_dashboard")
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port must be in [1024,65535]")
    app = Application(Terrain(Path(share("storm_gazebo")) / "terrain/mdrs_mission"),
                      share("storm_mission"), args.data_dir)
    node = executor = None
    if not args.offline:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        from storm_dashboard.ros_bridge import RosBridge
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = RosBridge(app)
        app.bridge = node
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()
    else:
        original_command = app.command

        def offline_command(name, values):
            if name not in ("status", "plan.add", "plan.set", "plan.clear", "plan.preview", "plan.export"):
                raise Conflict("Offline mode supports map planning only")
            return original_command(name, values)
        app.command = offline_command
    http = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(app, Path(share("storm_dashboard")) / "web"))
    http.daemon_threads = True
    http.timeout = .5
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    url = f"http://localhost:{args.port}"
    print(f"STORM mission console: {url}\nROS domain: {os.environ.get('ROS_DOMAIN_ID', '0')}\n"
          f"Gazebo partition: {os.environ.get('GZ_PARTITION', '(default)')}\nReports: {args.data_dir}", flush=True)
    if args.open_browser:
        webbrowser.open(url)
    try:
        while not stopping.is_set():
            http.handle_request()
    finally:
        app.close()
        http.server_close()
        if executor:
            executor.shutdown(timeout_sec=2)
            node.destroy_node()
            rclpy.shutdown()

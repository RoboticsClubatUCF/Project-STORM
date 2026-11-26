import json
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest
from storm_dashboard.core import Application, Conflict
from storm_dashboard.server import ThreadingHTTPServer, make_handler
from storm_mission.planner import Terrain


class Process:
    pid = 999999
    returncode = None

    def poll(self):
        return self.returncode


@pytest.fixture
def app(tmp_path, monkeypatch):
    terrain = Terrain.__new__(Terrain)
    terrain.height = np.zeros((20, 20))
    terrain.rows, terrain.cols = terrain.height.shape
    terrain.dx = terrain.dy = 8.
    terrain.west, terrain.north = -80., 80.
    terrain.metadata = {"station_latitude": 38.40645833333333,
                        "station_longitude": -110.79190277777778, "origin_source_elevation_m": 1372.995}
    calls = []

    def spawn(argv, **kwargs):
        calls.append((argv, kwargs))
        return Process()
    monkeypatch.setenv("DISPLAY", ":0")
    application = Application(terrain, tmp_path, tmp_path / "records", spawn)
    application.calls = calls
    return application


def ready(app):
    app.command("simulation.start", {})
    app.update("truth", {"x": 0., "y": 0., "yaw": 0.})
    app.update("nav2", {"active": True})
    app.update("drive", {"active": True})
    app.update("collision", {"active": True})
    app.update("objects", {"points": 0})


def test_clicked_waypoint_roundtrip_and_ordered_preview(app):
    app.command("plan.add", {"id": "first", "x": 30., "y": 10.})
    app.command("plan.add", {"id": "second", "x": -10., "y": -20.})
    first = app.waypoints[0]
    assert app.terrain.project(first["latitude"], first["longitude"]) == pytest.approx((30, 10), abs=1e-4)
    plan = app.command("plan.preview", {})
    assert [w["id"] for w in plan["waypoints"]] == ["first", "second"]
    assert plan["route_length_m"] > 80
    assert plan["start_source"] == "site datum (offline)"


@pytest.mark.parametrize("values", [{"x": float("nan"), "y": 0}, {"x": 1000, "y": 0},
    {"x": True, "y": 0}, {"latitude": 91, "longitude": 0}, {"x": 0, "y": 0, "shell": "echo bad"}])
def test_reject_bad_map_inputs_without_partial_edit(app, values):
    app.command("plan.add", {"x": 0, "y": 0})
    before = app.snapshot()["waypoints"]
    with pytest.raises(ValueError):
        app.command("plan.add", values)
    assert app.waypoints == before


def test_duplicate_ids_rejected(app):
    with pytest.raises(ValueError, match="unique"):
        app.command("plan.set", {"waypoints": [{"id": "x", "x": 0, "y": 0}, {"id": "x", "x": 1, "y": 1}]})


def test_navigation_reports_inactive_lifecycle_and_refuses_mission(app):
    ready(app)
    app.update("lifecycle", {"states": {"controller_server": 2, "planner_server": 1}})
    with pytest.raises(Conflict, match="controller_server: inactive"):
        app.command("mission.start", {})
    blockers = app.snapshot()["navigation_blockers"]
    assert "planner_server: unconfigured" in blockers
    assert not any("mission" in call[0][0] for call in app.calls)


def test_plan_persistence_and_export(app):
    app.command("plan.add", {"id": "saved", "x": 12, "y": 4})
    exported = app.command("plan.export", {})
    assert exported["frame"] == "WGS84"
    restored = Application(app.terrain, app.mission_share, app.data_dir, app.spawn)
    assert restored.waypoints[0]["id"] == "saved"
    app.command("plan.clear", {})
    restored = Application(app.terrain, app.mission_share, app.data_dir, app.spawn)
    assert restored.waypoints == []


def test_visible_launch_uses_fixed_arguments_and_new_process_group(app):
    app.command("simulation.start", {})
    argv, kwargs = app.calls[0]
    assert "gui:=true" in argv and "rviz:=true" in argv
    assert kwargs["start_new_session"] is True
    assert "shell" not in kwargs
    with pytest.raises(Conflict):
        app.command("simulation.start", {})


def test_deadman_expires_and_mission_is_exclusive(app):
    ready(app)
    app.command("teleop.enable", {})
    app.command("teleop.drive", {"linear": .2, "angular": .3})
    assert app.velocity() == (.2, .3)
    app.teleop["expires"] = time.monotonic() - .1
    assert app.velocity() == (0, 0)
    with pytest.raises(Conflict, match="Disable teleop"):
        app.command("mission.start", {})
    app.command("teleop.disable", {})
    app.command("plan.add", {"x": 2, "y": 1})
    app.command("mission.start", {})
    with pytest.raises(Conflict):
        app.command("teleop.enable", {})
    with pytest.raises(Conflict):
        app.command("plan.clear", {})


def test_velocity_bounds_and_stale_truth(app):
    ready(app)
    app.command("teleop.enable", {})
    with pytest.raises(ValueError):
        app.command("teleop.drive", {"linear": .31, "angular": 0})
    app.command("teleop.drive", {"linear": .2, "angular": 0})
    app.telemetry["truth"]["received"] -= 10
    assert app.velocity() == (0, 0)
    with pytest.raises(Conflict):
        app.command("teleop.drive", {"linear": .2, "angular": 0})


def test_teleop_stops_on_disable(app):
    ready(app)
    app.command("teleop.enable", {})
    app.command("teleop.drive", {"linear": .2, "angular": 0})
    app.command("teleop.disable", {})
    assert app.velocity() == (0, 0)


@pytest.mark.parametrize("missing", ["objects", "collision"])
def test_missing_collision_pipeline_blocks_motion(app, missing):
    ready(app)
    app.command("plan.add", {"x": 2, "y": 1})
    app.telemetry.pop(missing)
    with pytest.raises(Conflict):
        app.command("mission.start", {})
    with pytest.raises(Conflict):
        app.command("teleop.enable", {})


def test_mission_uses_gnss_snapshot_and_respects_strict_policy(app):
    ready(app)
    app.command("plan.add", {"id": "goal", "x": 2, "y": 1})
    app.command("mission.start", {"allow_unobserved": False})
    argv, _ = app.calls[-1]
    assert "--allow-unobserved" not in argv
    path = argv[argv.index("--mission") + 1]
    from pathlib import Path
    assert json.loads(Path(path).read_text())["waypoints"][0]["id"] == "goal"


def test_http_contract_and_cross_origin_rejection(app, tmp_path):
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app, tmp_path))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(base + "/api/schema") as response:
            assert "teleop.drive" in json.load(response)["commands"]
        request = Request(base + "/api/commands", data=b'{"command":"plan.add","args":{"x":2,"y":3}}',
                          headers={"Content-Type": "application/json"})
        with urlopen(request) as response:
            assert json.load(response)["result"]["waypoints"][0]["x"] == 2
        request.add_header("Origin", "https://foreign.example")
        with pytest.raises(HTTPError) as error:
            urlopen(request)
        assert error.value.code == 403
        request = Request(base + "/api/commands", data=b'{}', headers={"Content-Type": "text/plain"})
        with pytest.raises(HTTPError) as error:
            urlopen(request)
        assert error.value.code == 415
    finally:
        server.shutdown()
        server.server_close()

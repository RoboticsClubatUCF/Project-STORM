import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from storm_mission import planner

mission = vars(planner)
Terrain = mission["Terrain"]
subdivide = mission["subdivide"]


def ridge():
    terrain = Terrain.__new__(Terrain)
    terrain.height = np.zeros((5, 7))
    terrain.height[:, 3] = 20.0
    terrain.rows, terrain.cols = terrain.height.shape
    terrain.dx = terrain.dy = 8.0
    terrain.west, terrain.north = 0.0, 40.0
    return terrain


def test_routes_around_steep_ridge_through_gap():
    terrain = ridge()
    terrain.height[0, 3] = 0.0
    start, target = terrain.xy((2, 1)), terrain.xy((2, 5))
    route = terrain.route(start, target, 15.0)
    assert terrain.xy((0, 3)) in route
    assert route[-1] == target
    previous = start
    for x, y, _ in subdivide(route, 6.0):
        assert math.dist(previous, (x, y)) <= 6.0 + 1e-9
        previous = (x, y)
    assert previous == target


def test_unbroken_steep_ridge_rejects_route():
    terrain = ridge()
    with pytest.raises(ValueError, match="No route"):
        terrain.route(terrain.xy((2, 1)), terrain.xy((2, 5)), 15.0)


def test_gnss_datum_projects_to_origin():
    terrain = ridge()
    terrain.metadata = {
        "station_latitude": 38.406458333,
        "station_longitude": -110.791902778,
        "origin_source_elevation_m": 1372.995,
    }
    assert terrain.project(38.406458333, -110.791902778) == pytest.approx((0, 0), abs=1e-8)


@pytest.mark.parametrize("lat,lon", [(math.nan, 0), (0, math.inf), (91, 0),
                                     (0, -181), (True, 0), ("38", 0)])
def test_invalid_gnss_coordinates_rejected(lat, lon):
    with pytest.raises(ValueError):
        mission["coordinates"](lat, lon)


def test_mission_preserves_order_and_rejects_duplicate_ids(tmp_path):
    entries = [{"id": "second", "latitude": 38, "longitude": -110},
               {"id": "first", "latitude": 38.001, "longitude": -110}]
    data = {"version": 1, "frame": "WGS84", "waypoints": entries}
    path = tmp_path / "route.json"
    path.write_text(json.dumps(data))
    assert mission["load_waypoints"](path) == entries
    entries[1]["id"] = "second"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="unique"):
        mission["load_waypoints"](path)


@pytest.mark.parametrize("data", [[], {}, {"version": 1, "frame": "UTM", "waypoints": []},
    {"version": 1, "frame": "WGS84", "waypoints": []},
    {"version": 1, "frame": "WGS84", "waypoints": [{"id": "x"}]}])
def test_invalid_mission_schema_rejected(tmp_path, data):
    path = tmp_path / "route.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        mission["load_waypoints"](path)


def test_multi_stop_plan_includes_return_and_preflights_later_targets():
    terrain = ridge()
    start, end = terrain.xy((2, 0)), terrain.xy((2, 1))
    targets = [{"id": "out", "goal_xy": end}, {"id": "back", "goal_xy": start}]
    plan = mission["preview"](terrain, start, targets, 15, 3)
    assert plan["route_length_m"] == pytest.approx(16)
    assert [p["id"] for p in plan["waypoints"]] == ["out", "back"]
    assert plan["waypoints"][-1]["legs_xy_yaw"][-1][:2] == pytest.approx(start)
    targets.append({"id": "blocked", "goal_xy": terrain.xy((2, 5))})
    with pytest.raises(ValueError, match="No route"):
        mission["preview"](terrain, start, targets, 15, 3)


def test_fix_requires_valid_status_and_coordinates():
    fix = SimpleNamespace(latitude=38., longitude=-110., status=SimpleNamespace(status=0))
    assert mission["valid_fix"](fix)
    fix.status.status = -1
    assert not mission["valid_fix"](fix)
    fix.status.status = 0
    fix.latitude = math.nan
    assert not mission["valid_fix"](fix)

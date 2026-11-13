from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def test_stamped_command_pipeline_and_footprint():
    root = Path(get_package_share_directory("storm_navigation"))
    for name in ("nav2.yaml", "nav2_urc.yaml"):
        c = yaml.safe_load((root / "config" / name).read_text())
        for node in ("controller_server", "behavior_server", "velocity_smoother"):
            assert c[node]["ros__parameters"]["enable_stamped_cmd_vel"] is True
        for key in ("local_costmap", "global_costmap"):
            cost = c[key][key]["ros__parameters"]
            footprint = yaml.safe_load(cost["footprint"])
            assert max(p[1] for p in footprint) >= 0.4318 + 0.025
            assert cost["robot_base_frame"] == "base_footprint"
        assert c["velocity_smoother"]["ros__parameters"]["max_velocity"][0] <= 0.5
        assert c["velocity_smoother"]["ros__parameters"]["velocity_timeout"] < 0.5


def test_saved_map_is_relocatable():
    root = Path(get_package_share_directory("storm_navigation")) / "maps"
    data = yaml.safe_load((root / "storm_arena.yaml").read_text())
    assert not Path(data["image"]).is_absolute()
    assert (root / data["image"]).is_file()


def test_outdoor_route_and_object_observations_are_separate():
    root = Path(get_package_share_directory("storm_navigation"))
    config = yaml.safe_load((root / "config/nav2_urc.yaml").read_text())
    assert config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"]
    for name in ("local_costmap", "global_costmap"):
        layer = config[name][name]["ros__parameters"]["obstacle_layer"]
        assert layer["objects"]["marking"] and not layer["objects"]["clearing"]
        assert layer["ground"]["clearing"] and not layer["ground"]["marking"]
    monitor = config["collision_monitor"]["ros__parameters"]
    assert monitor["cmd_vel_in_topic"] == "cmd_vel_checked"
    assert monitor["cmd_vel_out_topic"] == "cmd_vel"
    assert 0 < monitor["source_timeout"] <= 1

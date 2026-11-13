import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def test_model_respects_hardware_operating_limits():
    root = Path(get_package_share_directory("storm_gazebo"))
    limits = yaml.safe_load((root / "config/sensor_limits.yaml").read_text())
    robot = ET.fromstring(xacro.process_file(str(root / "urdf/storm.gazebo.xacro")).toxml())
    lidars = robot.findall(".//sensor[@type='gpu_lidar']")
    assert len(lidars) == 1, "Do not give STORM an independent, unselected virtual lidar"
    lidar = lidars[0]
    assert lidar.attrib["name"] == "vlp16"
    assert float(lidar.findtext("lidar/range/min")) == limits["vlp16"]["minimum_range_m"]
    assert float(lidar.findtext("lidar/range/max")) == limits["vlp16"]["maximum_range_m"]
    assert int(lidar.findtext("lidar/scan/vertical/samples")) == 16
    camera = robot.find(".//sensor[@name='rgbd']/camera")
    assert float(camera.findtext("depth_camera/clip/near")) == limits["zed"]["minimum_depth_m"]
    assert float(camera.findtext("depth_camera/clip/far")) == limits["zed"]["maximum_depth_m"]
    assert float(camera.findtext("clip/far")) > limits["zed"]["maximum_depth_m"], "RGB visibility is not depth measurement"
    assert int(camera.findtext("image/width")) == limits["zed"]["width_px"]

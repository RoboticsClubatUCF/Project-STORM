from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory


@pytest.mark.parametrize(
    "backend,plugin",
    [
        ("mock", "mock_components/GenericSystem"),
        ("can", "storm_hardware/SparkCanHardware"),
        ("gazebo", "gz_ros2_control/GazeboSimSystem"),
    ],
)
def test_robot_backend_and_kinematics(backend, plugin):
    path = (
        Path(get_package_share_directory("storm_description")) / "urdf/storm.urdf.xacro"
    )
    root = ET.fromstring(
        xacro.process_file(str(path), mappings={"hardware": backend}).toxml()
    )
    assert [p.text for p in root.findall(".//hardware/plugin")] == [plugin]
    control = root.find("ros2_control")
    joints = control.findall("joint")
    assert len(joints) == 6
    assert len({j.find("param[@name='can_id']").text for j in joints}) == 6
    for joint in joints:
        assert [i.attrib["name"] for i in joint.findall("command_interface")] == [
            "velocity"
        ]
        assert {i.attrib["name"] for i in joint.findall("state_interface")} == {
            "position",
            "velocity",
        }
    for joint in root.findall("joint[@type='continuous']"):
        y = float(joint.find("origin").attrib["xyz"].split()[1])
        assert (y > 0) == ("left" in joint.attrib["name"])
    links = {n.attrib["name"] for n in root.findall("link")}
    assert {"base_link", "base_footprint", "lidar_link"} <= links

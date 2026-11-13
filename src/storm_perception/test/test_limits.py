import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from storm_perception.limits import limit_cloud, limit_depth


def test_cloud_blind_zone_and_far_clipping_preserve_layout():
    points = np.array([[.5, 0, 0], [1, 0, 0], [100, 0, 0], [100.01, 0, 0], [0, 0, np.inf]], dtype=np.float32)
    source = point_cloud2.create_cloud_xyz32(Header(frame_id="lidar_link"), points)
    result = limit_cloud(source)
    values = point_cloud2.read_points_numpy(result, field_names=("x", "y", "z"))
    assert np.isnan(values[[0, 3, 4]]).all()
    assert values[1, 0] == 1 and values[2, 0] == 100
    assert result.width == source.width and result.point_step == source.point_step
    assert np.isfinite(point_cloud2.read_points_numpy(source)[0]).all(), "Input must be unchanged"


def test_depth_limits_mask_instead_of_fabricating_boundary_hits():
    source = Image(height=1, width=7, encoding="32FC1", step=28,
                   data=np.array([.2, .5, 2, 20, 20.01, np.inf, -1], dtype="<f4").tobytes())
    result = limit_depth(source)
    values = np.frombuffer(result.data, dtype="<f4")
    assert np.isnan(values[[0, 4, 5, 6]]).all()
    assert list(values[1:4]) == [.5, 2, 20]

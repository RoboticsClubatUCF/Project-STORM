import math

import numpy as np
from storm_perception.scan import project_scan


def test_range_and_vertical_limits_are_preserved():
    points = [[.5, 0, 0], [2, 0, 0], [100.1, 0, 0],
              [0, 3, 0], [0, 0, 5], [math.nan, 0, 0]]
    values = project_scan(points)
    assert sorted(values[np.isfinite(values)]) == [2, 3]


def test_empty_cloud_is_no_return_not_free_space():
    assert np.isinf(project_scan(np.empty((0, 3)))).all()

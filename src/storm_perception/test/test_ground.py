import numpy as np
import pytest
from storm_perception.ground import segment


def scene(slope=0):
    x, y = np.meshgrid(np.linspace(-6, 6, 35), np.linspace(-6, 6, 35))
    floor = np.column_stack((x.ravel(), y.ravel(), slope*x.ravel()))
    box = np.array([[2+x*.02, y*.02, slope*2+.4+z*.02]
                    for x in range(6) for y in range(6) for z in range(4)])
    return floor, box


@pytest.mark.parametrize("slope", [0., .15, -.15])
def test_ground_removed_objects_preserved_on_slopes(slope):
    ground, box = scene(slope)
    floor, obstacles = segment(np.vstack((ground, box)))
    assert len(floor) > 100
    assert len(obstacles) == len(box)
    assert np.all(obstacles[:, 0] >= 2)


def test_missing_ground_and_wall_only_fail_closed():
    with pytest.raises(ValueError):
        segment(np.zeros((10, 3)))
    x, z = np.meshgrid(np.linspace(-4,4,30), np.linspace(-.5,.5,30))
    wall = np.column_stack((x.ravel(), np.ones(x.size)*2, z.ravel()))
    with pytest.raises(ValueError):
        segment(wall)


def test_empty_obstacle_cloud_is_valid_only_with_supported_ground():
    ground, _ = scene()
    _, obstacles = segment(ground)
    assert obstacles.shape == (0, 3)

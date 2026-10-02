import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from nomad_perception.geometry import project_scan, rotation
from nomad_perception.observed_grid import ObservedGrid
from nomad_path_planning.rollout import AckermannRollout


def quaternion(pitch=0.0):
    return NS(x=0., y=math.sin(pitch/2), z=0., w=math.cos(pitch/2))


def test_scan_uses_full_rotation_and_sensor_translation():
    scan = NS(ranges=[2.0, math.inf, math.nan, -math.inf, 0.01],
              angle_min=0., angle_increment=math.pi/2, range_min=.12, range_max=12.)
    tf = NS(rotation=quaternion(math.pi/6), translation=NS(x=1., y=2., z=.38))
    origin, rays = project_scan(scan, tf, 8.)
    assert origin == (1., 2.) and len(rays) == 2
    assert np.allclose(rays[0][:2], [1+math.sqrt(3), 2.])
    assert rays[0][2] and not rays[1][2]
    assert np.allclose(rays[1][:2], [1., 10.])


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        rotation(NS(x=0., y=0., z=0., w=0.))


def test_base_center_tracks_rear_axle_bicycle_in_forward_and_reverse():
    for speed in (.3, -.3):
        planner = AckermannRollout(speed=speed, horizon=.6)
        path = planner.simulate((2., 3., .4), .4)
        rear_x, rear_y, yaw = 2.-.36*math.cos(.4), 3.-.36*math.sin(.4), .4
        for x, y, actual_yaw in path[1:]:
            rear_x += speed*.1*math.cos(yaw)
            rear_y += speed*.1*math.sin(yaw)
            yaw += speed/.72*math.tan(.4)*.1
            assert np.allclose([x-.36*math.cos(actual_yaw), y-.36*math.sin(actual_yaw)],
                               [rear_x, rear_y])
            assert math.isclose(actual_yaw, yaw)


def test_projected_hit_blocks_full_nomad_footprint_and_can_clear():
    grid = ObservedGrid(resolution=.2, size=10., inflation=.7)
    assert grid.integrate_rays((0., 0.), [(2., 0., True)], 1.)
    cx, cy = grid.cell(1.4, 0.)
    assert grid.maps(1.)[1][cy, cx] == 100
    for stamp in range(2, 8):
        grid.integrate_rays((0., 0.), [(4., 0., False)], float(stamp))
    assert grid.maps(7.)[1][cy, cx] == 0

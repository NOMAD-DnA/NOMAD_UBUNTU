from types import SimpleNamespace
import math
import numpy as np
from nomad_perception.traversability_output import CostViews, transform_points


def estimate(value, stamp):
    return SimpleNamespace(final_cost=value, cost_stamp_s=stamp)


def test_soft_scores_survive_local_clearance_and_ttl():
    views = CostViews(initial_size=8., max_size=8.)
    estimates = {(x, y): estimate(47, 10.) for x in range(-10, 11) for y in range(-10, 11)}
    raw, global_map, local_map = views.render(estimates, 10.5)
    cx, cy = views.grid.cell(.1, .1)
    assert (raw[cy, cx], global_map[cy, cx], local_map[cy, cx]) == (47, 47, 47)
    assert views.render(estimates, 12.01)[2][cy, cx] == -1
    assert views.render(estimates, 39.99)[1][cy, cx] == 47
    assert views.render(estimates, 40.01)[1][cy, cx] == -1


def test_hard_block_inflates_without_lidar_and_wins_over_scan_free():
    views = CostViews(initial_size=8., max_size=8.)
    views.grid.log_odds[:] = -3.
    views.grid.seen[:] = 10.
    _, global_map, local_map = views.render({(0, 0): estimate(100, 10.)}, 10.5)
    cx, cy = views.grid.cell(.1, .1)
    assert global_map[cy, cx] == local_map[cy, cx] == 100
    assert global_map[cy, cx+4] == 100
    assert global_map[cy, cx+5] == 5


def test_recent_clearance_uses_actual_cost_support_age():
    views = CostViews(initial_size=8., max_size=8.)
    estimates = {(x, y): estimate(17, 1.) for x in range(-10, 11) for y in range(-10, 11)}
    cx, cy = views.grid.cell(.1, .1)
    assert views.render(estimates, 2.)[2][cy, cx] == 17
    # Re-rendering retained estimates never marks them newly observed.
    assert views.render(estimates, 4.)[2][cy, cx] == -1


def test_growth_keeps_absolute_world_indices_and_is_bounded():
    views = CostViews(initial_size=10., max_size=20.)
    assert views.accept_points(np.array([[7., .1, 1.]]))[0]
    _, global_map, _ = views.render({(35, 0): estimate(55, 10.)}, 10.1)
    cx, cy = views.grid.cell(7.1, .1)
    assert global_map[cy, cx] == 55
    assert not views.accept_points(np.array([[100., 0., 0.]]))[0]
    assert views.grid.width <= 100 and views.grid.height <= 100


def test_optical_depth_becomes_odom_xy_and_height_through_full_rotation():
    # +cameraZ -> +odomX; +cameraX -> -odomY; +cameraY -> -odomZ.
    t = SimpleNamespace(rotation=SimpleNamespace(x=-.5,y=.5,z=-.5,w=.5),
                        translation=SimpleNamespace(x=10.,y=20.,z=2.))
    transformed = transform_points(np.array([[1., .3, 4.]]), t)
    np.testing.assert_allclose(transformed, [[14., 19., 1.7]], atol=1e-6)

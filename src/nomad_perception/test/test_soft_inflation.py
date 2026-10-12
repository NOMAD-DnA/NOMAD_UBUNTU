"""Regression checks for preserving passability while adding proximity costs."""
import numpy as np
import pytest

from nomad_perception.observed_grid import ObservedGrid
from nomad_perception.terrain import fuse_costs


def grid(radius=1.2):
    return ObservedGrid(resolution=.2, size=10, inflation=.7,
                        soft_inflation_radius=radius)


def unchanged_masks(before, after):
    np.testing.assert_array_equal(before >= 80, after >= 80)
    np.testing.assert_array_equal(before < 0, after < 0)
    assert np.all(after[before >= 0] >= before[before >= 0])


def test_overlap_takes_max_and_keeps_existing_costs_unknown_and_blocked():
    g = grid()
    first = np.zeros((g.height, g.width), bool)
    second = first.copy()
    first[25, 20] = second[25, 30] = True  # 2m gap between obstacle centres
    free = np.zeros(first.shape, np.int8)
    a, = g.add_proximity_costs(first, free)
    b, = g.add_proximity_costs(second, free)
    together, = g.add_proximity_costs(first | second, free)
    np.testing.assert_array_equal(together, np.maximum(a, b))
    assert 0 < together[25, 25] < 80
    free[25, 25], free[25, 24], free[25, 26] = 65, -1, 100
    result, = g.add_proximity_costs(first | second, free)
    assert result[25, 25] == 65
    unchanged_masks(free, result)


def test_costs_decay_and_end_at_outer_radius():
    g = grid()
    obstacle = np.zeros((g.height, g.width), bool)
    obstacle[25, 25] = True
    result, = g.add_proximity_costs(obstacle, np.zeros(obstacle.shape, np.int8))
    # Centre distances 1.0, 1.2, 1.4m; minus 0.1414m once -> costs 37, 6, 0.
    assert list(result[25, 30:33]) == [37, 6, 0]
    assert np.all(np.diff(result[25, 25:].astype(int)) <= 0)


@pytest.mark.parametrize('mode', ['depth', 'lidar', 'fused'])
@pytest.mark.parametrize('now', [10., 13., 50.])
def test_existing_blocked_and_unknown_masks_survive_all_modes_and_expiry(mode, now):
    g = grid()
    rng = np.random.default_rng(17)
    g.log_odds[:] = rng.choice([-3., 0., 2.], g.log_odds.shape, p=[.97, .01, .02])
    g.seen[:] = rng.choice([10., 8., -np.inf], g.seen.shape)
    g.terrain[:] = rng.choice([-1, 0, 35, 65, 100], g.terrain.shape,
                              p=[.05, .89, .02, .02, .02])
    g.terrain_seen[:] = 10.
    if mode == 'depth':
        g.soft_inflation_radius = 0
        before = g.navigation_maps_without_lidar_obstacles(now, 30., 2.)
        g.soft_inflation_radius = 1.2
        after = g.navigation_maps_without_lidar_obstacles(now, 30., 2.)
    else:
        raw, global_map, local_map = g.maps(now)
        before = (global_map, local_map)
        obstacles = raw >= 80
        if mode == 'fused':
            terrain = g.terrain_map(now, 30.)
            before = fuse_costs(*before, terrain, g.kernel)
            obstacles |= terrain >= 80
        after = g.add_proximity_costs(obstacles, *before)
    for old, new in zip(before, after):
        unchanged_masks(old, new)


@pytest.mark.parametrize('mode', ['depth', 'lidar'])
def test_open_corridor_stays_open_in_global_and_local_maps(mode):
    g = grid()
    g.log_odds.fill(-3)
    g.seen.fill(10.)
    g.terrain.fill(0)
    g.terrain_seen.fill(10.)
    if mode == 'depth':
        g.terrain[:, [20, 30]] = 100
        g.soft_inflation_radius = 0
        before = g.navigation_maps_without_lidar_obstacles(10., 30., 2.)
        g.soft_inflation_radius = 1.2
        after = g.navigation_maps_without_lidar_obstacles(10., 30., 2.)
    else:
        g.log_odds[:, [20, 30]] = 2
        raw, *before = g.maps(10.)
        after = g.add_proximity_costs(raw >= 80, *before)
    for old, new in zip(before, after):
        unchanged_masks(old, new)
        assert np.all(old[10:40, 25] == 0)
        assert np.all((new[10:40, 25] > 0) & (new[10:40, 25] < 80))


def test_no_obstacles_disabled_and_removed_sources_leave_no_residual_cost():
    g = grid()
    empty = np.zeros((g.height, g.width), bool)
    free = np.zeros(empty.shape, np.int8)
    obstacle = empty.copy()
    obstacle[25, 25] = True
    assert np.any(g.add_proximity_costs(obstacle, free)[0] > 0)
    np.testing.assert_array_equal(g.add_proximity_costs(empty, free)[0], free)
    g.soft_inflation_radius = 0
    np.testing.assert_array_equal(g.add_proximity_costs(obstacle, free)[0], free)


@pytest.mark.parametrize('radius', [-1, .7, .5, float('nan'), float('inf')])
def test_invalid_soft_radius_is_rejected(radius):
    with pytest.raises(ValueError, match='soft_inflation_radius'):
        grid(radius)


@pytest.mark.parametrize('lidar_enabled,camera_enabled', [(False, True), (True, True), (True, False)])
def test_published_navigation_messages_preserve_corridor(lidar_enabled, camera_enabled):
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from nomad_perception.sensor_input import SensorInput

    g = grid()
    g.log_odds.fill(-3)
    g.seen.fill(10.)
    g.terrain.fill(0)
    g.terrain_seen.fill(10.)
    g.terrain[:, [20, 30]] = 100
    if not camera_enabled:
        g.log_odds[:, [20, 30]] = 2
    messages = [[] for _ in range(6)]
    publishers = [NS(publish=items.append) for items in messages]
    node = NS(grid=g, now=lambda: 10., max_age=2., frame='odom',
              pose=None, local_memory_seconds=30., local_memory_radius=3.,
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=10))),
              camera=NS(mode='geometry', enabled=camera_enabled, ttl=30., neutral=35),
              navigation_lidar_enabled=lidar_enabled, map_pubs=publishers[:3],
              terrain_pub=publishers[3], navigation_pubs=publishers[4:])
    g.soft_inflation_radius = 0
    SensorInput.publish_maps(node)
    g.soft_inflation_radius = 1.2
    SensorInput.publish_maps(node)
    for index, published in enumerate(messages):
        before, after = (np.asarray(m.data).reshape(g.height, g.width) for m in published)
        unchanged_masks(before, after)
        if index in (0, 3):  # raw sensor and terrain evidence must stay unchanged
            np.testing.assert_array_equal(before, after)
        if index in (4, 5):
            assert np.all((after[10:40, 25] > 0) & (after[10:40, 25] < 80))

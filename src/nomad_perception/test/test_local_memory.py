"""Sensor age, moving-window and obstacle precedence regressions."""
import numpy as np
import pytest
from nomad_perception.observed_grid import ObservedGrid


def make_grid():
    return ObservedGrid(size=12., inflation=.6, soft_inflation_radius=1.2)


def local(g, now, mode='depth', xy=(.1, .1), seconds=30.):
    kw = dict(robot_xy=xy, local_memory_seconds=seconds, local_memory_radius=3.)
    return (g.navigation_maps_without_lidar_obstacles(now, 30., 2., **kw)[1]
            if mode == 'depth' else g.maps(now, 2., **kw)[2])


def value(g, layer, x=.1, y=.1):
    cx, cy = g.cell(x, y)
    return layer[cy, cx]


@pytest.mark.parametrize('mode', ['depth', 'lidar'])
def test_previously_seen_ground_survives_blind_spot_without_refreshing_age(mode):
    g = make_grid()
    if mode == 'depth':
        g.terrain.fill(0); g.terrain_seen.fill(10.)
    else:
        g.log_odds.fill(-3.); g.seen.fill(10.)
    # Observed at t=10, now under/behind a vehicle which moved forward.
    assert value(g, local(g, 15., mode, xy=(1., .1))) == 0
    assert value(g, local(g, 15., mode, seconds=0.)) == -1
    assert value(g, local(g, 25., mode)) == 0
    assert value(g, local(g, 40.01, mode)) == -1  # reads did not renew evidence
    assert value(g, local(g, 9., mode)) == -1  # future observations are not valid
    assert value(g, local(g, 15., mode, xy=None)) == -1
    assert value(g, local(g, 15., mode, xy=(float('nan'), 0.))) == -1


@pytest.mark.parametrize('mode', ['depth', 'lidar'])
def test_memory_window_moves_without_painting_unknown_ground(mode):
    g = make_grid()
    assert np.all(local(g, 15., mode) == -1)
    if mode == 'depth':
        g.terrain.fill(0); g.terrain_seen.fill(10.)
    else:
        g.log_odds.fill(-3.); g.seen.fill(10.)
    assert value(g, local(g, 15., mode)) == 0
    assert value(g, local(g, 15., mode), 4.1) == -1
    assert value(g, local(g, 15., mode, xy=(4.1, .1)), 4.1) == 0
    assert value(g, local(g, 15., mode, xy=(4.1, .1))) == -1
    g.ensure_bounds(-10., -10.)
    assert value(g, local(g, 15., mode)) == 0  # world coordinates survive map growth
    assert value(g, local(g, 15., mode, xy=(-10., -10.)), -10., -10.) == -1


@pytest.mark.parametrize('mode', ['depth', 'lidar'])
def test_new_obstacle_overrides_remembered_free_space_immediately(mode):
    g = make_grid()
    if mode == 'depth':
        g.terrain.fill(0); g.terrain_seen.fill(10.)
        g.integrate_surface_costs(np.array([[.1,.1]]), np.array([100],np.int8), 15.)
    else:
        g.log_odds.fill(-3.); g.seen.fill(10.)
        g.integrate_rays((-1.,.1),[(.1,.1,True,.3)],15.)
    result = local(g, 15., mode)
    assert value(g, result) == 100
    assert value(g, result, .5) == 100  # hard inflation still wins


def test_grass_memory_keeps_cost_but_not_unobserved_neighbours():
    g = make_grid()
    p = np.array([[.1,.1]])
    g.integrate_surface_costs(p, np.array([35],np.int8), 10., passable=True)
    assert value(g, local(g, 15.)) == 35
    assert value(g, local(g, 15.), .3) == -1
    assert value(g, local(g, 40.01)) == -1
    # New ambiguous classification revokes the older grass evidence.
    g.integrate_surface_costs(p, np.array([-1],np.int8), 16.)
    assert value(g, local(g, 16.)) == -1
    g.integrate_surface_costs(p, np.array([35],np.int8), 10., passable=True)
    assert value(g, local(g, 16.)) == -1


def test_retained_observations_do_not_skip_clearance_check():
    g = make_grid()
    g.integrate_surface_costs(np.array([[.1,.1]]), np.array([0],np.int8), 10.)
    # A single ordinary ground cell is not enough to fit the vehicle.
    assert value(g, local(g, 15.)) == -1


def test_publisher_uses_vehicle_position_and_memory_settings():
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from nomad_perception.sensor_input import SensorInput
    g = make_grid()
    g.terrain.fill(0); g.terrain_seen.fill(10.)
    messages = [[] for _ in range(6)]
    pubs = [NS(publish=m.append) for m in messages]
    node = NS(grid=g, now=lambda:15., max_age=2., frame='odom',
              pose=NS(pose=NS(position=NS(x=.1,y=.1))), pose_stamp=15.,
              fresh=lambda stamp:True, fault=None,
              local_memory_seconds=30., local_memory_radius=3.,
              get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=15))),
              camera=NS(mode='geometry',enabled=True,ttl=30.,neutral=35),
              navigation_lidar_enabled=False, map_pubs=pubs[:3],terrain_pub=pubs[3],
              navigation_pubs=pubs[4:])
    SensorInput.publish_maps(node)
    a=np.array(messages[5][-1].data).reshape(g.height,g.width)
    assert value(g,a)==0
    node.fresh=lambda stamp:False
    SensorInput.publish_maps(node)
    a=np.array(messages[5][-1].data).reshape(g.height,g.width)
    assert value(g,a)==-1

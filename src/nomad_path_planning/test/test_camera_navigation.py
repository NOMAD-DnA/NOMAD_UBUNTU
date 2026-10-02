from builtin_interfaces.msg import Time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from nomad_perception.observed_grid import ObservedGrid
from nomad_perception.sensor_input import SensorInput
from nomad_perception.camera_terrain import CameraTerrain
from nomad_path_planning.rollout import AckermannRollout


def ground_grid():
    grid = ObservedGrid(size=10, inflation=.70)
    grid.terrain[:] = 0
    grid.terrain_seen[:] = 10.
    return grid


def test_lidar_hill_hit_does_not_block_camera_navigation_but_depth_rock_does():
    grid = ground_grid()
    grid.integrate_rays((0., 0.), [(2., 0., True, .4)], 10.)
    cx, cy = grid.cell(2., 0.)
    assert grid.maps(10.)[1][cy, cx] == 100
    global_map, local_map = grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)
    assert global_map[cy, cx] == local_map[cy, cx] == 0
    planner = AckermannRollout(allow_unknown=False)
    metadata = dict(width=grid.width, height=grid.height, resolution=grid.resolution,
                    origin_x=grid.origin_x, origin_y=grid.origin_y)
    assert np.isfinite(planner.terrain_cost([(2., 0., 0.)],
                       dict(metadata, data=local_map.ravel())))
    grid.integrate_surface_costs(np.array([[2., 0.]]), np.array([100]), 10.1)
    _, local_map = grid.navigation_maps_without_lidar_obstacles(10.1, 30., 2.)
    assert not np.isfinite(planner.terrain_cost([(2., 0., 0.)],
                           dict(metadata, data=local_map.ravel())))
    # Preserve the original clearance radius around camera obstacles.
    assert local_map[cy, cx+3] == 100


def test_single_free_ray_cannot_clear_entire_footprint_and_ground_expires():
    grid = ObservedGrid(size=10)
    grid.integrate_rays((0., 0.), [(3., 0., False, 0.)], 10.)
    assert not np.any(grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)[1] == 0)
    grid = ground_grid()
    cx, cy = grid.cell(0., 0.)
    global_map, local_map = grid.navigation_maps_without_lidar_obstacles(13., 30., 2.)
    assert global_map[cy, cx] == 0 and local_map[cy, cx] == -1
    assert np.all(grid.navigation_maps_without_lidar_obstacles(41., 30., 2.)[0] == -1)
    # Unknown/neutral cells anywhere in the clearance disc veto local motion.
    grid.terrain[cy, cx+2] = 35
    assert grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)[1][cy, cx] == -1


@pytest.mark.parametrize('active,ground_stamp,cells,expected', [
    (True, 10., {1: 1}, True),
    (False, 10., {1: 1}, False),
    (True, -np.inf, {}, False),
    (True, 9., {1: 1}, False),
])
def test_navigation_health_uses_depth_even_without_any_scan(active, ground_stamp, cells, expected):
    node = SimpleNamespace(
        now=lambda: 10., last_now=10., fault=None, process_scan=Mock(),
        camera=SimpleNamespace(tick=Mock(), active=lambda: active, last_stamp=10.,
                               navigation_active=lambda: active and bool(cells) and ground_stamp == 10.,
                               ground_valid=bool(cells) and ground_stamp == 10.,
                               ground=SimpleNamespace(cells=cells, stamp=ground_stamp)),
        navigation_lidar_enabled=False, pose=object(), pose_stamp=10.,
        fresh=lambda stamp: stamp == 10., odom_wall=float('inf'), wall_timeout=3.,
        health_pub=Mock(), status_pub=Mock(), module_pub=Mock(), frame="odom",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10))), last_status=None,
        get_logger=lambda: Mock(), last_map_wall=float('inf'),
        scan_valid=False, scan_stamp=-np.inf, scan_wall=-np.inf,
    )
    SensorInput.tick(node)
    assert node.health_pub.publish.call_args.args[0].data == expected


def test_existing_scan_free_space_allows_departure_without_depth_under_vehicle():
    grid = ObservedGrid(size=10, inflation=.70)
    grid.log_odds[:] = -3.
    grid.seen[:] = 10.
    cx, cy = grid.cell(0., 0.)
    global_map, local_map = grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)
    assert global_map[cy, cx] == local_map[cy, cx] == 0
    # A LiDAR hit alone is unknown, not a blocked or cleared cell; no inflation.
    grid.log_odds[cy, cx+10] = 3.
    global_map, local_map = grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)
    assert global_map[cy, cx+10] == -1
    assert not np.any(global_map == 100) and not np.any(local_map == 100)
    assert local_map[cy, cx] == 0
    # Depth-confirmed uphill ground opens the hit, but a depth object blocks it.
    grid.terrain[cy, cx+10] = 0
    grid.terrain_seen[cy, cx+10] = 10.
    assert grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)[1][cy, cx+10] == 0
    grid.terrain[cy, cx+10] = 100
    assert grid.navigation_maps_without_lidar_obstacles(10., 30., 2.)[1][cy, cx+10] == 100
    # LiDAR observations are never silently refreshed by new depth frames.
    assert grid.navigation_maps_without_lidar_obstacles(13., 30., 2.)[1][cy, cx] == -1


@pytest.mark.parametrize('image_stamp,processed_stamp,input_wall,processed_wall,valid,expected', [
    (10., 9., 100., 100., True, True),  # live input, valid existing two-second map
    (9., 9., 100., 100., True, False),  # no new input
    (10., 7., 100., 100., True, False),  # worker stalled: new input cannot refresh old map
    (10., 9., 96., 100., True, False),  # wall-clock input stall
    (10., 9., 100., 96., True, False),  # wall-clock processing stall
    (10., 9., 100., 100., False, False),  # rejected depth/TF
])
def test_depth_input_and_processed_map_have_independent_expiry(
        monkeypatch, image_stamp, processed_stamp, input_wall, processed_wall, valid, expected):
    monkeypatch.setattr('nomad_perception.camera_terrain.time.monotonic', lambda: 100.)
    camera = SimpleNamespace(enabled=True, ground_valid=valid,
        depth=[SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(image_stamp), nanosec=0)))],
        last_stamp=processed_stamp, last_wall=processed_wall, depth_received_wall=input_wall,
        node=SimpleNamespace(fresh=lambda at: 0 <= 10.-at <= .6,
                             now=lambda: 10., max_age=2., wall_timeout=3.))
    assert CameraTerrain.navigation_active(camera) == expected

from types import SimpleNamespace as NS

import numpy as np
from builtin_interfaces.msg import Time

from nomad_perception.geometric_terrain import depth_points, surface_costs
from nomad_perception.observed_grid import ObservedGrid
from nomad_perception.terrain import fuse_costs
from nomad_perception.camera_terrain import CameraTerrain


def surface(height):
    y, x = np.mgrid[-.6:.6:.025, -.6:.6:.025]
    return np.column_stack((x.ravel(), y.ravel(), height(x, y).ravel()))


def test_flat_and_inclined_planes_separate_slope_from_roughness():
    flat = surface(lambda x, y: np.zeros_like(x))
    _, costs, metrics = surface_costs(flat)
    assert len(costs) and np.all(costs == 0)
    _, costs, metrics = surface_costs(surface(lambda x, y: x*np.tan(.2)))
    np.testing.assert_allclose(metrics[:, 0], .2, atol=1e-8)
    assert np.all(metrics[:, 1] < 1e-8)
    assert np.all((costs > 0) & (costs < 80))
    _, costs, _ = surface_costs(surface(lambda x, y: y*np.tan(.4)))
    assert np.all(costs == 100)


def test_rough_surface_and_step_are_hazards():
    _, costs, metrics = surface_costs(surface(lambda x, y: .08*np.sin(65*x)*np.cos(65*y)))
    assert np.any(metrics[:, 1] >= .04) and np.any(costs == 100)
    centres, costs, _ = surface_costs(surface(lambda x, y: np.where(x > 0, .3, 0.)))
    assert np.any(costs[np.abs(centres[:, 0]) < .2] == 100)


def test_vertical_surface_sparse_and_invalid_points():
    y, z = np.mgrid[-.5:.5:.02, 0:1:.02]
    wall = np.column_stack((np.zeros(y.size), y.ravel(), z.ravel()))
    _, costs, _ = surface_costs(wall)
    assert len(costs) and np.all(costs == 100)
    for points in [[], [[np.nan, 0, 0]], [[0, 0, 0], [.1, .1, 0]]]:
        assert len(surface_costs(points)[1]) == 0


def test_projection_uses_world_frame_and_never_fills_missing_depth():
    r = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    points = depth_points(np.array([[2., 0.], [np.nan, np.inf]]),
                          [100, 0, 0, 0, 100, 0, 0, 0, 1], r, np.array([1, 2, 3]), 1)
    np.testing.assert_allclose(points, [[3, 2, 3]])


def test_geometry_hazards_fuse_without_clearing_lidar_or_unknown():
    grid = ObservedGrid(size=4)
    centres = np.array([[.1, .1]])
    grid.integrate_surface_costs(centres, np.array([100], dtype=np.int8), 2.)
    grid.integrate_surface_costs(centres, np.array([0], dtype=np.int8), 1.)
    x, y = grid.cell(.1, .1)
    assert grid.terrain[y, x] == 100
    grid.ensure_bounds(-3, -3)
    x, y = grid.cell(.1, .1)
    terrain = grid.terrain_map(3, 30)
    assert terrain[y, x] == 100
    raw, global_map, local = grid.maps(3)
    fused, safe = fuse_costs(global_map, local, terrain, grid.kernel)
    assert raw[y, x] == -1 and fused[y, x] == safe[y, x] == 100
    assert safe[0, 0] == -1
    assert grid.terrain_map(33, 30)[y, x] == -1


def test_geometry_process_needs_no_rgb_and_updates_actual_grid():
    camera = CameraTerrain.__new__(CameraTerrain)
    header = NS(stamp=Time(sec=10), frame_id='optical')
    depth = NS(header=header, width=80, height=80, encoding='32FC1',
               is_bigendian=0, step=320, data=np.full((80, 80), 2., dtype='<f4').tobytes())
    info = NS(header=header, width=80, height=80, d=[],
              k=[100, 0, 40, 0, 100, 40, 0, 0, 1])
    # Optical forward -> odom X, optical down -> odom -Z.
    tf = NS(transform=NS(rotation=NS(x=-.5, y=.5, z=-.5, w=.5),
                        translation=NS(x=0., y=0., z=.3)))
    camera.node = NS(fault=None, fresh=lambda s: s == 10., frame='odom',
                     buffer=NS(lookup_transform=lambda *args: tf), grid=ObservedGrid(size=10))
    camera.mode, camera.depth, camera.info = 'geometry', [depth], {'depth': info}
    camera.last_stamp = -np.inf
    camera.stride, camera.max_range = 4, 6.
    camera.patch_radius, camera.geometry_min_samples = .35, 6
    camera.slope_limit, camera.roughness_limit, camera.step_limit = .35, .04, .18
    camera.slope_enabled = False
    camera.objects_only = False
    camera.process()
    assert camera.last_stamp == 10.
    assert camera.status.startswith('GEOMETRY:')
    assert np.any(camera.node.grid.terrain == 100)


def test_disabled_slope_has_no_cost_but_keeps_steps_roughness_and_walls():
    _, costs, _ = surface_costs(surface(lambda x, y: x*np.tan(.6)), slope_enabled=False)
    assert len(costs) and np.all(costs == 0)
    for points in [surface(lambda x, y: np.where(x > 0, .3, 0.)),
                   surface(lambda x, y: .08*np.sin(65*x)*np.cos(65*y))]:
        assert np.any(surface_costs(points, slope_enabled=False)[1] == 100)
    y, z = np.mgrid[-.5:.5:.02, 0:1:.02]
    wall = np.column_stack((np.zeros(y.size), y.ravel(), z.ravel()))
    assert np.all(surface_costs(wall, slope_enabled=False)[1] == 100)

from types import SimpleNamespace

import numpy as np
import pytest

from nomad_perception.terrain import classify, image_array, project_ground, fuse_costs
from nomad_perception.observed_grid import ObservedGrid
from nomad_path_planning.dstar_lite import DStarLite
from nomad_path_planning.rollout import AckermannRollout
from nomad_perception.camera_terrain import CameraTerrain


def test_colour_baseline_and_uncertain_pixels():
    # Render palette RGB: dry soil (130,99,67), yellow-green field (165,151,112).
    bgr = np.array([[[67, 99, 130], [112, 151, 165], [35, 130, 40],
                     [100, 100, 100], [2, 3, 5]]], dtype=np.uint8)
    assert classify(bgr).tolist() == [[1, 2, 2, 0, 0]]


def test_depth_big_endian_padding_and_rgb_channels():
    msg = SimpleNamespace(encoding='16UC1', width=2, height=2, step=6,
                          is_bigendian=1, data=b'\x03\xe8\x07\xd0XX\x00\x00\x0b\xb8XX')
    np.testing.assert_allclose(image_array(msg), [[1, 2], [0, 3]])
    msg = SimpleNamespace(encoding='rgb8', width=1, height=1, step=4,
                          is_bigendian=0, data=bytes([130, 99, 67, 0]))
    assert image_array(msg).tolist() == [[[67, 99, 130]]]


def test_projection_ground_vs_vertical_wall_and_invalid_depth():
    # Optical z forward, x right, y down; camera 0.3 m above a flat floor.
    r = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    t = np.array([2, 3, .3])
    k = [100, 0, 40, 0, 100, 30, 0, 0, 1]
    v, _ = np.mgrid[:80, :80]
    depth = np.full((80, 80), np.nan, dtype=np.float32)
    below = v > 30
    depth[below] = .3*100/(v[below]-30)
    labels = np.ones_like(depth, dtype=np.uint8)
    points, classes = project_ground(depth, labels, k, r, t)
    assert len(points) > 0 and np.all(classes == 1)
    np.testing.assert_allclose(points[:, 2], 0, atol=1e-6)
    assert np.all(points[:, 0] > 2)
    wall, _ = project_ground(np.full((80, 80), 2.), labels, k, r, t)
    assert len(wall) == 0
    empty, _ = project_ground(np.zeros((80, 80)), labels, k, r, t)
    assert len(empty) == 0


def test_terrain_memory_expansion_mixed_edges_and_expiry():
    grid = ObservedGrid(size=10, max_size=40)
    points = np.array([[.01, .01, 0]] * 10)
    assert grid.integrate_terrain(points, np.array([1]*8+[2]*2), 1) == 1
    cx, cy = grid.cell(0, 0)
    assert grid.terrain[cy, cx] == 65  # minority grass at a road edge wins
    assert grid.maps(1)[0][cy, cx] == -1  # camera never clears unknown
    grid.ensure_bounds(-10, -10)
    cx, cy = grid.cell(0, 0)
    assert grid.terrain_map(2, 30)[cy, cx] == 65
    grid.integrate_terrain(points, np.ones(10), .5)
    assert grid.terrain[cy, cx] == 65  # out-of-order observation ignored
    grid.integrate_terrain(points, np.ones(10), 3)
    assert grid.terrain[cy, cx] == 0
    assert grid.terrain_map(34, 30)[cy, cx] == -1


def test_camera_cannot_clear_obstacles_or_unknown_and_footprint_cost():
    g = np.zeros((7, 7), dtype=np.int8)
    g[2, 2], g[4, 4] = 100, -1
    local = g.copy()
    local[1, 1] = -1
    terrain = np.zeros_like(g)
    terrain[3, 3] = 65
    fused, safe = fuse_costs(g, local, terrain, np.ones((3, 3), np.uint8))
    assert fused[2, 2] == 100 and safe[2, 2] == 100
    assert fused[4, 4] == -1 and safe[1, 1] == -1
    assert fused[3, 2] == 65 and fused[0, 0] == 0
    fallback, _ = fuse_costs(g, local, np.full_like(g, -1), np.ones((1, 1), np.uint8))
    assert fallback[0, 0] == 35 and fallback[2, 2] == 100


def test_planners_prefer_dirt_detour_and_still_reject_obstacle():
    data = np.full((11, 21), 65, dtype=np.int8)
    data[3, 1:20] = 0
    data[3:6, 1] = 0
    data[3:6, 19] = 0
    planner = DStarLite(21, 11, data.ravel().tolist(), (1, 5), (19, 5), unknown_penalty=4.5)
    assert planner.compute_shortest_path()
    assert (10, 3) in planner.extract_path()  # longer dirt road over grass shortcut
    grid = dict(width=21, height=11, resolution=1., origin_x=0., origin_y=0.,
                data=data.ravel().tolist())
    lpp = AckermannRollout(allow_unknown=False)
    road = [(x+.5, 3.5, 0) for x in range(3, 18)]
    field = [(x+.5, 5.5, 0) for x in range(3, 18)]
    assert lpp.terrain_cost(road, grid) < lpp.terrain_cost(field, grid)
    grid['data'][3*21+10] = 100
    assert not np.isfinite(lpp.terrain_cost(road, grid))


@pytest.mark.parametrize('mode', ['unsynchronized', 'stale', 'calibration_mismatch'])
def test_bad_rgbd_does_not_update_terrain(mode):
    def message(t):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=t, nanosec=0),
                                                      frame_id='optical'), width=8, height=8)
    camera = CameraTerrain.__new__(CameraTerrain)
    # No grid/buffer: rejected frames must not reach map integration or TF lookup.
    camera.node = SimpleNamespace(fault=None, fresh=lambda t: 9.5 <= t <= 10.5)
    camera.last_stamp = -np.inf
    camera.sync_tolerance = .04
    camera.status = ''
    camera.depth = [message(10)]
    camera.rgb = [message(9 if mode == 'unsynchronized' else 10)]
    if mode == 'stale':
        camera.rgb = [message(1)]
        camera.depth = [message(1)]
    ri, di = message(10), message(10)
    ri.k = [100, 0, 4, 0, 100, 4, 0, 0, 1]
    di.k = [90, 0, 4, 0, 100, 4, 0, 0, 1]
    camera.info = {'rgb': ri, 'depth': di}
    camera.process()
    assert camera.last_stamp == -np.inf
    assert camera.status.startswith('REJECT' if mode == 'calibration_mismatch' else 'WAIT')

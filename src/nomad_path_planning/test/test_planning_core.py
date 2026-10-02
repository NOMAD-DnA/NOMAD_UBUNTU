"""Meaningful sensor-only mapping / planning regressions (no simulator needed)."""
import heapq
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from nomad_path_planning.dstar_lite import DStarLite
from nomad_perception.observed_grid import ObservedGrid
from nomad_path_planning.rollout import AckermannRollout


def at(grid, image, x, y):
    cx, cy = grid.cell(x, y)
    return image[cy, cx]


def test_near_goal_does_not_require_clearance_past_destination():
    data = np.zeros((60, 60), dtype=np.int8)
    data[:, 32:] = 100  # wall at x=1.2, after a goal at x=1.0
    grid = dict(width=60, height=60, resolution=.1, origin_x=-2., origin_y=-3.,
                data=data.ravel())
    planner = AckermannRollout(speed=.4, horizon=1.5, max_steer=.4, steer_samples=1)
    result = planner.plan((0., 0., 0.), [(0., 0.), (1., 0.)], grid)
    assert result['success']
    assert planner.horizon == 1.5
    assert not planner.plan((0., 0., 0.), [(0., 0.), (3., 0.)], grid)['success']
    data[:, 27:] = 100  # obstacle before the goal still blocks every candidate
    assert not planner.plan((0., 0., 0.), [(0., 0.), (1., 0.)], grid)['success']


def test_unknown_occlusion_and_obstacle_removal():
    grid = ObservedGrid(resolution=0.2, inflation=0.2)
    assert np.all(grid.maps(0)[0] == -1)
    grid.integrate((0, 0, 0), [2.0], 0, 1, .1, 10, 1)
    raw, _, _ = grid.maps(1)
    assert at(grid, raw, 1, 0) == 0
    assert at(grid, raw, 2, 0) == 100
    assert at(grid, raw, 3, 0) == -1
    for time in range(2, 7):
        grid.integrate((0, 0, 0), [math.inf], 0, 1, .1, 10, time)
    assert at(grid, grid.maps(6)[0], 2, 0) == 0
    # A newly appearing object must immediately block previously free space.
    grid.integrate((0, 0, 0), [2.0], 0, 1, .1, 10, 7)
    assert at(grid, grid.maps(7)[0], 2, 0) == 100


def test_invalid_scan_does_not_create_free_space():
    grid = ObservedGrid()
    assert not grid.integrate((0,0,0), [math.nan, -math.inf, 0.01], 0, 1, .1, 10, 1)
    assert np.all(grid.maps(1)[0] == -1)


def test_recent_observations_required_for_local_driving():
    grid = ObservedGrid()
    grid.integrate((0,0,0), [math.inf]*720, -math.pi, 2*math.pi/719, .1, 10, 1)
    raw, global_map, local_map = grid.maps(1)
    assert at(grid, local_map, 0, 0) == 0
    assert at(grid, local_map, 7.9, 0) == -1  # footprint crosses unobserved boundary
    assert at(grid, grid.maps(4)[2], 0, 0) == -1
    assert at(grid, grid.maps(4)[1], 0, 0) == 0  # global memory persists


def test_map_expansion_preserves_obstacles_world_coordinates():
    grid = ObservedGrid(size=10, max_size=30)
    grid.integrate((0,0,0), [2.0], 0, 1, .1, 3, 1, usable_range=3)
    assert grid.ensure_bounds(-9,-9)
    assert at(grid, grid.maps(1)[0], 2, 0) == 100
    assert at(grid, grid.maps(1)[0], -9, -9) == -1
    assert not grid.ensure_bounds(1000,1000)


def test_dstar_replans_after_first_obstacle_observation():
    width = height = 20
    data = [-1]*(width*height)
    planner = DStarLite(width,height,data,(2,10),(17,10))
    assert planner.compute_shortest_path()
    assert (9,10) in planner.extract_path()
    for y in range(7,14):
        data[y*width+9] = 100
    planner.update_costmap(data)
    assert planner.compute_shortest_path()
    path = planner.extract_path()
    assert path[-1] == (17,10)
    assert (9,10) not in path
    assert all(math.isfinite(planner.edge_cost(a,b)) for a,b in zip(path,path[1:]))


def test_dstar_low_unknown_penalty_prefers_seen_free_and_blocks_obstacles():
    width, height = 25, 11
    data = [-1] * (width * height)
    for x in range(3, 22):
        data[6 * width + x] = 0
    planner = DStarLite(width, height, data, (2, 5), (22, 5), unknown_penalty=1.1)
    assert planner.compute_shortest_path()
    assert (12, 6) in planner.extract_path()

    # The cached cell penalties must track new lidar observations on the same grid.
    for y in range(3, 8):
        data[y * width + 12] = 100
    planner.update_costmap(data)
    assert planner.compute_shortest_path()
    path = planner.extract_path()
    assert path[0] == (2, 5) and path[-1] == (22, 5)
    assert all(not (x == 12 and 3 <= y <= 7) for x, y in path)
    assert all(math.isfinite(planner.edge_cost(a, b)) for a, b in zip(path, path[1:]))


def test_dstar_cached_costs_match_independent_shortest_path_after_updates():
    width = height = 15
    start, goal = (1, 1), (13, 13)
    rng = np.random.default_rng(42)

    def raw_edge(data, a, b):
        def value(cell):
            return data[cell[1] * width + cell[0]]

        if value(a) >= 80 or value(b) >= 80:
            return math.inf
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        if dx == dy == 1 and (value((a[0], b[1])) >= 80
                              or value((b[0], a[1])) >= 80):
            return math.inf
        cost = lambda cell: 1.1 if value(cell) < 0 else 1.0 + 4.0 * value(cell) / 79.0
        return (math.sqrt(2) if dx == dy else 1.0) * (cost(a) + cost(b)) / 2

    def reference_cost(data):
        best = {start: 0.0}
        queue = [(0.0, start)]
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost != best[cell]:
                continue
            if cell == goal:
                return cost
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nxt = (cell[0] + dx, cell[1] + dy)
                    if (dx == dy == 0 or not 0 <= nxt[0] < width
                            or not 0 <= nxt[1] < height):
                        continue
                    next_cost = cost + raw_edge(data, cell, nxt)
                    if next_cost < best.get(nxt, math.inf):
                        best[nxt] = next_cost
                        heapq.heappush(queue, (next_cost, nxt))
        return math.inf

    for _ in range(4):
        data = rng.choice([-1, 0, 25, 100], width * height,
                          p=[0.55, 0.25, 0.10, 0.10]).tolist()
        data[start[1] * width + start[0]] = 0
        data[goal[1] * width + goal[0]] = 0
        planner = DStarLite(width, height, data, start, goal, unknown_penalty=1.1)
        for update in range(2):
            if update:
                for index in rng.choice(width * height, 12, replace=False):
                    data[int(index)] = int(rng.choice([-1, 0, 100]))
                data[start[1] * width + start[0]] = 0
                data[goal[1] * width + goal[0]] = 0
                planner.update_costmap(data)
            optimum = reference_cost(data)
            assert planner.compute_shortest_path() == math.isfinite(optimum)
            path = planner.extract_path()
            if math.isfinite(optimum):
                actual = sum(raw_edge(data, a, b) for a, b in zip(path, path[1:]))
                assert path[0] == start and path[-1] == goal
                assert math.isclose(actual, optimum, rel_tol=1e-9)
            else:
                assert not path


def test_forward_and_reverse_respect_unknown_space():
    grid = dict(width=100,height=100,resolution=.1,origin_x=-5.,origin_y=-5.,data=[-1]*10000)
    path=[(0.,0.),(1.,0.),(2.,0.)]
    forward = AckermannRollout(allow_unknown=False)
    reverse = AckermannRollout(speed=-.3,horizon=.6,allow_unknown=False)
    assert not forward.plan((0,0,0),path,grid)['success']
    assert not reverse.plan((0,0,0),path,grid)['success']
    grid['data'] = [0]*10000
    # Block all forward arcs but leave the rear free.
    for y in range(100):
        grid['data'][y*100+52] = 100
    assert not forward.plan((0,0,0),path,grid)['success']
    backward = reverse.plan((0,0,0),path,grid)
    assert backward['success']
    assert backward['best']['trajectory'][-1][0] < 0

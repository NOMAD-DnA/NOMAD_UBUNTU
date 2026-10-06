"""Reverse guidance must preserve the original directed grid problem."""
import heapq
import math
import random

import pytest

from nomad_path_planning.gpp.astar import AStar
from nomad_path_planning.gpp.base import BudgetExceeded, Grid, Request
from nomad_path_planning.gpp.dstar_lite import DStarLite
from nomad_path_planning.history_recovery import EntryGate


@pytest.mark.parametrize('resolution', [.2, .35, 1.])
@pytest.mark.parametrize('gated', [False, True])
def test_guided_astar_matches_directed_dijkstra(resolution, gated):
    rng = random.Random(7)
    data = [rng.choice([0, 0, -1, 20, 60, 100]) for _ in range(400)]
    data[21] = data[378] = 0
    r = Request(dict(width=20, height=20, resolution=resolution,
                     origin_x=0., origin_y=0., data=data),
                (1.5*resolution, 1.5*resolution, 0.),
                (18.5*resolution, 18.5*resolution))
    if gated:
        r.gates = (EntryGate(10.*resolution, 10.*resolution, 0., 4.*resolution),)
    grid = Grid(r, 4.5)
    start, goal = grid.cell(*r.start[:2]), grid.cell(*r.goal)
    graph = DStarLite(20, 20, data, start, goal, unknown_penalty=4.5)
    graph.edge_filter = lambda a, b: not any(
        gate.blocks(grid.world(a), grid.world(b)) for gate in r.gates)
    costs = {start: 0.}
    queue = [(0., start)]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost != costs[cell]:
            continue
        if cell == goal:
            break
        for nxt in graph.neighbors(cell):
            candidate = cost + graph.edge_cost(cell, nxt)
            if candidate < costs.get(nxt, math.inf):
                costs[nxt] = candidate
                heapq.heappush(queue, (candidate, nxt))
    result = AStar(seconds=5.).plan(r)
    assert result.path
    assert math.isfinite(grid.path_cost(result.path))
    cells = [grid.cell(*p[:2]) for p in result.path[1:-1]]
    actual = sum(graph.edge_cost(a, b) for a, b in zip(cells, cells[1:]))
    assert actual == pytest.approx(costs[goal])


def test_guidance_rebuilt_when_map_changes():
    r = Request(dict(width=12, height=12, resolution=.2,
                     origin_x=0., origin_y=0., data=[0]*144),
                (.3, .3, 0.), (2.1, 2.1))
    planner = AStar(seconds=5.)
    assert planner.plan(r).path
    for y in range(12):
        r.grid['data'][y*12+6] = 100
    assert not planner.plan(r).path
    r.grid['data'] = [0]*144
    assert planner.plan(r).path


def test_reverse_guidance_obeys_shared_deadline(monkeypatch):
    from nomad_path_planning.gpp import base
    ticks = iter(range(1000))
    monkeypatch.setattr(base.time, 'monotonic', lambda: next(ticks))
    r = Request(dict(width=12, height=12, resolution=.2,
                     origin_x=0., origin_y=0., data=[0]*144),
                (.3, .3, 0.), (2.1, 2.1))
    with pytest.raises(BudgetExceeded):
        AStar(seconds=.5).plan(r)

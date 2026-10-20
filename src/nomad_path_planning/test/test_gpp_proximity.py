import math
import numpy as np
import pytest
from nomad_path_planning.gpp.proximity import proximity_costs
from nomad_path_planning.gpp.base import Grid, Request
from nomad_path_planning.gpp.dstar_lite import DStarLite
from nomad_path_planning.gpp.registry import create


def test_distance_uses_source_centres_preserves_unknown_and_blocking():
    data = [100, 90, 0, -1, 0, 0]
    original = list(data)
    extra = proximity_costs(data, 6, 1, .5)
    assert extra == pytest.approx([0, 0, 1., 0, 0, 0])
    assert data == original
    grid = Grid(Request(dict(width=6,height=1,resolution=.5,origin_x=0,origin_y=0,
                             data=data,proximity=extra),(0,0,0),(1,0)))
    assert math.isinf(grid.value(.75,.25))
    assert grid.value(1.25,.25) == 2.
    assert grid.value(1.75,.25) == 4.5
    assert not any(proximity_costs(data,6,1,.5,weight=0))
    assert not any(proximity_costs([0]*6,6,1,.5))


def route_map():
    # Straight short passage at y=0 between two obstacle rows; open detours.
    a=np.zeros((31,51),dtype=int)
    a[13,15:36]=100
    a[17,15:36]=100
    return dict(width=51,height=31,resolution=.2,origin_x=0.,origin_y=0.,data=a.ravel().tolist())


@pytest.mark.parametrize('name',['dstar_lite','astar','weighted_astar','field_dstar'])
def test_soft_cost_selects_wider_detour_without_blocking_short_passage(name):
    g=route_map()
    request=Request(g,(1.1,3.1,0.),(9.1,3.1))
    plain=create(name,expansions=200000).plan(request)
    assert plain.path
    g['proximity']=proximity_costs(g['data'],51,31,.2)
    soft=create(name,expansions=200000).plan(request)
    assert soft.path
    assert max(abs(p[1]-3.1) for p in plain.path)<.3
    assert max(abs(p[1]-3.1) for p in soft.path)> .6
    assert math.isfinite(Grid(request).path_cost(plain.path))
    assert Grid(request).path_cost(soft.path)<Grid(request).path_cost(plain.path)


def test_incremental_updates_include_unchanged_cells_with_new_proximity():
    g=route_map(); a=g['data']; start=(5,15); goal=(45,15)
    graph=DStarLite(51,31,a,start,goal)
    assert graph.compute_shortest_path()
    extra=proximity_costs(a,51,31,.2)
    changed=graph.update_costmap(a,extra)
    assert changed and graph.compute_shortest_path()
    fresh=DStarLite(51,31,a,start,goal,proximity=extra)
    assert fresh.compute_shortest_path()
    assert graph.path_cost(graph.extract_path())==pytest.approx(fresh.path_cost(fresh.extract_path()))
    assert graph.update_costmap(a,extra)==[]
    # Removing the obstacle must also clear the surrounding preference.
    cleared=[0]*len(a)
    graph.update_costmap(cleared,proximity_costs(cleared,51,31,.2))
    assert graph.compute_shortest_path()
    assert graph.path_cost(graph.extract_path())==pytest.approx(40.)

"""Behavioral coverage of selectable strategies, not just registry names."""
import math
import pytest
from nomad_path_planning.gpp.registry import create, NAMES
from nomad_path_planning.gpp.base import Request,Grid,BudgetExceeded
from nomad_path_planning.gpp.reference import ReferenceAdapter
from nomad_path_planning.gpp.hybrid_astar import advance
from nomad_path_planning.lpp.registry import create as local
from nomad_path_planning.history_recovery import EntryGate


def request():
    return Request(dict(width=50,height=40,resolution=.2,origin_x=-2.,origin_y=-4.,
                        data=[0]*2000),(0.,0.,0.),(5.,1.))


@pytest.mark.parametrize('name',NAMES)
def test_open_goal_and_costmap_contract(name):
    r=request()
    result=create(name,seconds=4.).plan(r)
    assert result.path, result.detail
    assert math.dist(result.path[-1][:2],r.goal)<.3
    assert math.isfinite(Grid(r).path_cost(result.path))
    assert result.expansions>0


@pytest.mark.parametrize('name',NAMES)
def test_solid_wall_has_no_path(name):
    r=request(); r.goal=(3.,0.)
    for y in range(r.grid['height']): r.grid['data'][y*50+20]=100
    try: result=create(name,seconds=.15,expansions=1000).plan(r)
    except BudgetExceeded: return
    assert not result.path


@pytest.mark.parametrize('name',['dstar_lite','field_dstar'])
def test_incremental_cost_increase_then_clear(name):
    r=request(); r.goal=(4.,0.)
    planner=create(name,seconds=5.)
    assert planner.plan(r).path
    graph=planner.graph
    for y in range(17,24): r.grid['data'][y*50+20]=100
    changed=planner.plan(r)
    assert changed.path and planner.graph is graph
    assert math.isfinite(Grid(r).path_cost(changed.path))
    r.grid['data']=[0]*2000
    assert planner.plan(r).path


@pytest.mark.parametrize('name',['hybrid_astar','state_lattice'])
def test_kinematic_paths_respect_rear_axle_curvature(name):
    r=request(); result=create(name,seconds=4.).plan(r)
    limit=math.tan(.4)/.72
    for a,b in zip(result.path,result.path[1:]):
        ar=(a[0]-.36*math.cos(a[2]),a[1]-.36*math.sin(a[2]))
        br=(b[0]-.36*math.cos(b[2]),b[1]-.36*math.sin(b[2]))
        ds=math.dist(ar,br)
        dyaw=abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))
        assert dyaw<=limit*ds*1.02+1e-6


def test_lattice_primitives_end_on_position_and_orientation_lattice():
    p=create('state_lattice')
    for h in range(16):
        found=False
        for turn in (-1,0,1):
            for length in (.8,1.2,1.6):
                edge=p.primitive(h,turn,length,.05)
                if not edge: continue
                found=True; x,y,yaw=edge[-1]
                assert abs(x/.4-round(x/.4))<1e-8
                assert abs(y/.4-round(y/.4))<1e-8
                assert abs(math.sin(yaw-(h+turn)*math.pi/8))<1e-8
        assert found


def test_field_recurrence_uses_interior_interpolation():
    from nomad_path_planning.gpp.field_dstar import FieldGraph
    graph=FieldGraph(5,5,[0]*25,(1,1),(4,4))
    u,a,b=(1,1),(2,1),(2,2)
    graph.g[graph.index(a)]=1.; graph.g[graph.index(b)]=.5
    interpolated=graph.triangle_value(u,a,b)
    assert interpolated < min(2.,math.sqrt(2)+.5)-.02


@pytest.mark.parametrize('name',['hybrid_astar','state_lattice','astar','weighted_astar','field_dstar'])
def test_directional_recovery_gate_cannot_be_crossed(name):
    r=request(); r.gates=(EntryGate(2.,0.,0.,20.),)
    try: result=create(name,seconds=.15,expansions=800).plan(r)
    except BudgetExceeded: return
    assert not result.path


@pytest.mark.parametrize('gpp,lpp',[
    ('hybrid_astar','rpp'),('state_lattice','rpp'),('dstar_lite','rollout'),
    ('astar','rollout'),('weighted_astar','rollout'),('field_dstar','rpp')])
def test_first_five_combinations_have_valid_local_motion(gpp,lpp):
    r=request(); path=create(gpp,seconds=4.).plan(r)
    if gpp=='field_dstar': path=ReferenceAdapter(seconds=4.).adapt(r,path)
    output=local(lpp,speed=.4).plan(r.start,[p[:2] for p in path.path],r.grid)
    assert output and output['success']
    best=output['best']
    assert math.isfinite(Grid(r).path_cost(best['trajectory']))
    assert abs(best['steer'])<=.4
    assert 0<best.get('speed',.4)<=.4


def test_rpp_stops_on_obstacle_and_unknown_and_regulates_speed():
    r=request(); path=[(i*.1,0.) for i in range(51)]
    p=local('rpp',speed=.4)
    clear=p.plan(r.start,path,r.grid)
    assert clear['success']
    # Cost regulation is explicit in the candidate command.
    r.grid['data']=[35]*2000
    slow=p.plan(r.start,path,r.grid)
    assert slow['success'] and slow['best']['speed']<clear['best']['speed']
    for value in (-1,100):
        r.grid['data']=[value]*2000
        assert not p.plan(r.start,path,r.grid)['success']


def test_rpp_rejects_impossible_turn_instead_of_clipping_steering():
    r=request()
    assert not local('rpp',speed=.4).plan(r.start,[(0.,0.),(.15,1.),(.2,2.)],r.grid)['success']


def test_invalid_names_and_weights_are_errors():
    with pytest.raises(ValueError,match='Unknown GPP'): create('typo')
    with pytest.raises(ValueError,match='Unknown LPP'): local('typo')
    for value in (float('nan'),.5):
        with pytest.raises(ValueError): create('weighted_astar',weight=value)


def test_hybrid_long_unknown_goal_with_bounded_expansions():
    # A distant RViz goal is mostly in unknown space. Unit-cost Euclidean
    # guidance floods SE(2) states because unknown traversal costs 4.5x more.
    r=Request(dict(width=400,height=400,resolution=.2,origin_x=-40.,origin_y=-40.,
                   data=[-1]*160000),(-28.,0.,0.),(-2.,12.))
    for y in range(192,209):
        for x in range(55,90):
            r.grid['data'][y*400+x]=0
    result=create('hybrid_astar',seconds=5.,expansions=1500).plan(r)
    assert result.path
    assert math.dist(result.path[-1][:2],r.goal)<.3
    assert math.isfinite(Grid(r).path_cost(result.path))

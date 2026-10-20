import math
import numpy as np
from nomad_path_planning.corridor_selector import CorridorSelector, observed_grid
from nomad_path_planning.gpp.base import Grid, Request


def grid(a):
    return dict(width=a.shape[1],height=a.shape[0],resolution=.2,origin_x=0.,origin_y=0.,data=a.ravel().tolist())


def test_unknown_shortcut_excluded_and_goal_may_get_farther():
    a=np.full((80,80),-1);a[10:65,10:25]=0
    g=grid(a);s=(3.5,3.,math.pi/2);goal=(14.,1.)
    selector=CorridorSelector(seconds=2.)
    cs=selector.candidates(g,s,goal,0.)
    assert cs and all(c.point[1]>s[1] for c in cs)
    assert math.dist(cs[0].point,goal)>math.dist(s[:2],goal)
    private=observed_grid(g)
    assert private['data'][0]==100 and g['data'][0]==-1
    assert not math.isfinite(Grid(Request(private,s,goal)).value(*goal))


def test_unknown_gap_does_not_connect_observed_islands():
    a=np.full((70,70),-1);a[5:30,5:25]=0;a[35:65,5:25]=0
    cs=CorridorSelector(seconds=2.).candidates(grid(a),(3.,3.,math.pi/2),(3.,10.),0.)
    assert all(c.point[1]<6. for c in cs)
    assert not any(c.final for c in cs)


def test_wide_branch_preferred_and_active_held():
    a=np.full((80,80),100)
    a[10:50,10:28]=0;a[40:47,28:65]=0
    selector=CorridorSelector(seconds=2.)
    g=grid(a);s=(3.5,3.,math.pi/2);goal=(14.,10.)
    cs=selector.candidates(g,s,goal,0.)
    assert cs and cs[0].clearance>=.7
    selector.accept(cs[0]);point=cs[0].point
    cs2=selector.candidates(g,(3.5,3.2,math.pi/2),goal,1.)
    assert cs2[0].point==point
    selector.reject(point,1.)
    cs3=selector.candidates(g,s,goal,2.)
    assert all(math.dist(c.point,point)>=1.5 for c in cs3)


def test_final_goal_selected_only_when_known_reachable():
    a=np.zeros((60,60),dtype=int);selector=CorridorSelector(seconds=2.)
    cs=selector.candidates(grid(a),(3.,3.,0.),(6.,3.),0.)
    assert cs[0].final and cs[0].point==(6.,3.)
    a[15,30]=100
    assert not any(c.final for c in selector.candidates(grid(a),(3.,3.,0.),(6.,3.),1.))


def test_no_fake_start_and_budget_is_failure():
    a=np.zeros((60,60),dtype=int);a[15,15]=-1
    selector=CorridorSelector(seconds=2.)
    assert selector.candidates(grid(a),(3.1,3.1,0.),(6.,3.),0.)==[]
    assert selector.detail['state']=='start_not_observed_free'
    a[15,15]=0;selector.max_expansions=1
    assert selector.candidates(grid(a),(3.1,3.1,0.),(6.,3.),0.)==[]
    assert selector.detail['state']=='selection_budget_exhausted'


def test_rejected_candidates_are_reconsidered_after_short_cooldown():
    a=np.zeros((60,60),dtype=int);selector=CorridorSelector(seconds=2.)
    g=grid(a);start=(3.,3.,0.);goal=(6.,3.)
    selector.reject(goal,0.)
    assert not any(c.final for c in selector.candidates(g,start,goal,1.))
    assert selector.candidates(g,start,goal,3.1)[0].final


def test_long_kinematic_loop_is_not_an_acceptable_local_route():
    from nomad_path_planning.corridor_selector import short_path_length
    assert short_path_length([(0,0),(3,0),(5,0)],5.,8.)==(5.,True)
    length,valid=short_path_length([(0,0),(0,8),(8,8),(8,0),(5,0)],5.,8.)
    assert length==27. and not valid


def test_planning_risk_selects_lower_cost_observed_branch():
    a=np.zeros((100,100),dtype=int)
    g=grid(a);weights=np.ones_like(a,dtype=float);weights[50:,:]=201.
    g['cell_weights']=weights.ravel().tolist()
    start=(5.,10.,0.);goal=(18.,15.)
    baseline=CorridorSelector(seconds=3.).candidates(g,start,goal,0.)
    safer=CorridorSelector(seconds=3.,risk_weight=3.).candidates(g,start,goal,0.)
    assert baseline[0].point[1]>10.
    assert safer[0].point[1]<10. and safer[0].risk<.1
    assert g['data']==a.ravel().tolist()


def test_risk_never_opens_unknown_and_rejects_bad_weights():
    import pytest
    a=np.full((70,70),-1);a[5:30,5:25]=0
    g=grid(a);g['cell_weights']=[1.]*(70*70)
    cs=CorridorSelector(seconds=2.,risk_weight=3.).candidates(g,(3.,3.,math.pi/2),(3.,10.),0.)
    assert all(c.point[1]<6. for c in cs)
    g['cell_weights'][15*70+15]=float('nan')
    with pytest.raises(ValueError):CorridorSelector(risk_weight=1.).candidates(g,(3.,3.,0.),(3.,10.),0.)


def test_planning_margin_is_private_and_preserves_unknown():
    from nomad_path_planning.corridor_selector import clearance_grid
    a=np.zeros((12,12),dtype=int);a[6,6]=100;a[6,5]=-1
    g=grid(a);g['cell_weights']=[1.]*144;g['cell_weights'][6*12+6]=math.inf
    copy=clearance_grid(g,.4);d=np.array(copy['data']).reshape(12,12)
    assert d[6,5]==-1 and d[6,8]>=80 and d[6,9]==0
    assert math.isinf(copy['cell_weights'][6*12+8])
    assert g['data']==a.ravel().tolist() and g['cell_weights'][6*12+8]==1.
    # No fake free start if newly required margin encompasses the vehicle.
    assert CorridorSelector().candidates(copy,(1.5,1.3,0.),(2.,1.3),0.)==[]


def test_target_bearing_cost_discourages_sharp_subgoal_turn():
    g=grid(np.zeros((120,120),dtype=int));start=(10.,10.,0.);goal=(23.,22.)
    a=CorridorSelector(seconds=2.,goal_weight=1.).candidates(g,start,goal,0.)[0]
    b=CorridorSelector(seconds=2.,goal_weight=1.,bearing_weight=3.).candidates(g,start,goal,0.)[0]
    angle=lambda c:abs(math.atan2(c.point[1]-10.,c.point[0]-10.))
    assert angle(a)>.5 and angle(b)<.1


def test_partial_budget_returns_only_completed_candidates():
    g=grid(np.zeros((100,100),dtype=int));s=(5.,5.,0.);goal=(7.,5.)
    c=CorridorSelector(seconds=3.,max_expansions=1000,incremental=True)
    choices=c.candidates(g,s,goal,0.)
    assert choices and c.detail['state']=='partial_candidates'
    assert c.detail['expanded']==1000
    assert any(q.final for q in choices)
    c.max_expansions=1
    assert c.candidates(g,s,goal,1.)==[]


def test_revalidated_route_fast_path_rejects_new_obstacle_unknown_and_pose_error():
    from nomad_path_planning.corridor_selector import Candidate
    g=grid(np.zeros((60,60),dtype=int));s=(3.1,3.1,0.);goal=(10.,3.1)
    path=[(3.1+i*.1,3.1,0.) for i in range(31)]
    c=CorridorSelector(seconds=2.,incremental=True);c.accept(Candidate(path[-1][:2],0.,3.,1.,0.),path)
    assert c.candidates(g,s,goal,0.)[0].point==path[-1][:2]
    assert c.detail['state']=='active_path_revalidated' and c.detail['expanded']==0
    for cost in [100,-1]:
        blocked=dict(g);blocked['data']=list(g['data']);blocked['data'][15*60+22]=cost
        c.candidates(blocked,s,goal,1.)
        assert c.detail['state']!='active_path_revalidated'
    c.candidates(g,(3.1,3.1,math.pi),goal,2.)
    assert c.detail['state']!='active_path_revalidated'

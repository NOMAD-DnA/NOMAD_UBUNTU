import math
import pytest
from nomad_path_planning.observed_guard import known_prefix, footprint_known, observed_distance, speed_limit
from nomad_path_planning.gpp.multicost import compose
from nomad_path_planning.gpp.registry import create
from nomad_path_planning.gpp.base import Request

def grid():
    return dict(width=100,height=100,resolution=.1,origin_x=-5.,origin_y=-5.,data=[0]*10000)

def unknown_after(g,x=1.):
    for y in range(g['height']):
        for i in range(math.floor((x-g['origin_x'])/g['resolution']),g['width']):g['data'][y*g['width']+i]=-1

def test_prefix_checks_between_sparse_waypoints_and_preserves_yaw():
    g=grid();unknown_after(g)
    p=known_prefix((0.,0.,.2),[(0.,0.,.2),(3.,0.,.5)],g)
    assert .9<p[-1][0]<1. and p[-1][2]>.2
    assert all(0<=q[0]<1 for q in p)

def test_prefix_does_not_rejoin_beyond_unknown_hole():
    g=grid();g['data'][50*100+55]=-1
    p=known_prefix((0.,0.,0.),[(0.,0.,0.),(2.,0.,0.)],g)
    assert p[-1][0]<.5

def test_footprint_catches_side_unknown_and_outside():
    g=grid();g['data'][53*100+50]=-1
    assert not footprint_known(g,0,0,.65)
    assert not footprint_known(g,4.9,0,.65)
    assert footprint_known(g,-2,0,.65)

def test_existing_inflation_is_not_inflated_again():
    g=grid();g['data'][53*100+50]=80
    assert footprint_known(g,0,0,.65)
    assert not footprint_known(g,.05,.35,.65)

def test_actual_arc_and_reverse_stop_before_unknown():
    g=grid();unknown_after(g)
    d=observed_distance(g,(0,0,0),0,1)
    assert .3<d<.5
    assert observed_distance(g,(0,0,math.pi),0,-1)==pytest.approx(d)
    assert observed_distance(g,(0,0,0),.4,1)>0

def test_speed_cap_braking_and_map_age():
    v=speed_limit(.5,.3,.4,.1)
    assert v*.3+v*v/(2*.4)+.1==pytest.approx(.5)
    assert speed_limit(.5,1.3,.4,.1)<v
    assert speed_limit(.05,.3,.4,.1)==0
    assert speed_limit(.5,.3,0,.1)==0

def test_unknown_cost_changes_choice_without_changing_known_cost():
    g=dict(width=9,height=7,resolution=1.,origin_x=0.,origin_y=0.,data=[0]*63)
    for x in range(2,7):g['data'][3*9+x]=-1
    s=[0.]*63
    g['cell_weights']=compose(g,s,s,obstacle_weight=0,slope_weight=0,terrain_weight=0,unknown_surcharge=0)
    a=create('dstar_lite',seconds=2.).plan(Request(g,(.5,3.5,0.),(8.5,3.5)))
    g['cell_weights']=compose(g,s,s,obstacle_weight=0,slope_weight=0,terrain_weight=0,unknown_surcharge=9)
    b=create('dstar_lite',seconds=2.).plan(Request(g,(.5,3.5,0.),(8.5,3.5)))
    assert a.path and b.path
    assert all(abs(p[1]-3.5)<.01 for p in a.path)
    assert any(abs(p[1]-3.5)>.5 for p in b.path)

@pytest.mark.parametrize('reverse',[False,True])
def test_command_guard_cannot_be_bypassed_by_recovery(reverse):
    from nav_msgs.msg import OccupancyGrid
    from nomad_path_planning.command_selector import PlanningCommandNode
    from test_history_recovery import controller,motion,path
    node,commands=controller();g=grid()
    # Unknown immediately outside the footprint leaves no braking margin.
    g['data'][50*100+56]=-1
    g['data'][50*100+43]=-1
    msg=OccupancyGrid();msg.header.frame_id='odom';msg.header.stamp.sec=10
    msg.info.width=100;msg.info.height=100;msg.info.resolution=.1
    msg.info.origin.position.x=-5.;msg.info.origin.position.y=-5.;msg.data=g['data']
    node.costmap=msg;node.observed_guard=True
    node.guard_half_length=.52;node.guard_half_width=.40;node.guard_deceleration=.4;node.guard_latency=.3
    node.guard_margin=.1;node.guard_wheelbase=.72;node.guard_offset=.36
    node.global_path=path([0.,10.]);node.local_received_at=10.
    node.local_motion=motion([0.,.03]);node.local_path=node.local_motion.path
    if reverse:
        node.recovery_active=True;node.recovery_motion=motion([0.,-.02])
        node.recovery_path=node.recovery_motion.path;node.recovery_received_at=10.
    statuses=[];node.report=statuses.append
    PlanningCommandNode.control(node)
    assert commands[-1].stop_requested
    assert statuses[-1].startswith('STOP: observation boundary')

def test_forward_initial_rear_unknown_is_not_a_veto_but_reverse_is():
    g=grid();g['data'][50*100+47]=-1
    before=list(g['data'])
    assert observed_distance(g,(0,0,0),0,1)>1.9
    assert observed_distance(g,(0,0,0),0,-1)==0
    assert g['data']==before

def test_exemption_does_not_slide_into_unknown_and_never_softens_blocked():
    g=grid();unknown_after(g,.2)
    # Unknown already under the front is tolerated, but new cells are not.
    assert observed_distance(g,(0,0,0),0,1)<.15
    g=grid();g['data'][50*100+50]=100
    assert observed_distance(g,(0,0,0),0,1)==0

def test_newly_swept_side_cell_is_checked_when_turning():
    from nomad_path_planning.observed_guard import footprint_cells
    g=grid();initial=footprint_cells(g,(0,0,0))
    # Find a side cell crossed only by a turn, not by the straight envelope.
    found=False
    for cy in range(55,64):
        for cx in range(54,65):
            if (cx,cy) in initial:continue
            g['data'][cy*100+cx]=-1
            if observed_distance(g,(0,0,0),.4,1)<1.9 and observed_distance(g,(0,0,0),0,1)>1.9:
                found=True;break
            g['data'][cy*100+cx]=0
        if found:break
    assert found

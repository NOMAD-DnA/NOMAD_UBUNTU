"""Rear-steering integration: reference geometry, strategy combinations and tail sweep."""
import math
import pytest
from nomad_vehicle_model import Geometry, advance, profile
from nomad_path_planning.gpp.registry import create, NAMES
from nomad_path_planning.gpp.base import Request, Grid
from nomad_path_planning.gpp.reference import ReferenceAdapter
from nomad_path_planning.lpp.registry import create as local
from nomad_path_planning.observed_guard import observed_distance,footprint_cells

@pytest.mark.parametrize('speed',[.3,-.3])
@pytest.mark.parametrize('angle',[-.4,.4])
def test_exact_front_axle_circle_and_measured_joint_roundtrip(speed,angle):
    g=Geometry();leftv,rightv,left,right=g.mix(speed,angle)
    assert g.equivalent_from_joints(left,right)==pytest.approx(angle)
    assert (rightv-leftv)*g.radius/g.track==pytest.approx(speed*math.tan(angle)/g.wheelbase)
    p=g.advance((0.,0.,0.),speed,angle,2.)
    k=math.tan(angle)/g.wheelbase;theta=speed*2*k
    assert p[:2]==pytest.approx((.36+math.sin(theta)/k-.36*math.cos(theta),
                                (1-math.cos(theta))/k-.36*math.sin(theta)))
    assert g.advance(p,-speed,angle,2.)==pytest.approx((0.,0.,0.),abs=1e-9)

def test_bad_profile_rejected_and_legacy_supported():
    with pytest.raises(ValueError,match='reference_offset'):profile(dict(reference_offset=.36))
    with pytest.raises(ValueError,match='max_steer'):profile(dict(max_steer=.55))
    assert profile(dict(drive_layout='rear_drive_front_steer',reference_offset=.36)).steering_axle=='front'

@pytest.mark.parametrize('gpp',NAMES)
@pytest.mark.parametrize('lpp',['rollout','rpp'])
def test_all_selectable_pairs_support_front_axle(gpp,lpp):
    r=Request(dict(width=60,height=50,resolution=.2,origin_x=-3.,origin_y=-4.,data=[0]*3000),(0.,0.,0.),(5.,1.))
    result=create(gpp,reference_offset=-.36,seconds=4.).plan(r)
    assert result.path,result.detail
    if gpp not in ('hybrid_astar','state_lattice'):
        result=ReferenceAdapter(reference_offset=-.36,seconds=4.).adapt(r,result)
    assert result.path,result.detail
    out=local(lpp,reference_offset=-.36,speed=.4).plan(r.start,result.path,r.grid)
    assert out and out['success'],(gpp,lpp,out)
    best=out['best']
    assert math.isfinite(Grid(r).path_cost(best['trajectory']))
    end=best['trajectory'][-1]
    assert end[0]>0 and abs(best['steer'])<=.4

def test_rear_steering_tail_sweeps_outward_and_unknown_stops():
    rear=advance((0.,0.,0.),math.tan(.4)/.72,.1,-.36)
    assert rear[1]<0  # body initially sweeps right while yaw turns left
    g=dict(width=100,height=100,resolution=.05,origin_x=-2.5,origin_y=-2.5,data=[0]*10000)
    initial=footprint_cells(g,(0.,0.,0.))
    outer=footprint_cells(g,rear)-initial
    assert outer
    for x,y in outer:g['data'][y*100+x]=-1
    assert observed_distance(g,(0.,0.,0.),.4,1,offset=-.36,horizon=.5)<.15


def test_default_envelope_contains_steered_wheel_corners():
    g=Geometry()
    for alpha in (-.4,.4):
        _,_,left,right=g.mix(.3,alpha)
        for y,angle in ((.3,left),(-.3,right)):
            for u in (-.15,.15):
                for v in (-.04,.04):
                    assert abs(-.36+u*math.cos(angle)-v*math.sin(angle))<=.52
                    assert abs(y+u*math.sin(angle)+v*math.cos(angle))<=.42


def test_short_terminal_prefix_can_reach_arrival_without_crossing_unknown():
    from nomad_path_planning.observed_guard import usable_prefix
    reference=[(0.,0.,0.),(.08,0.,0.)]
    assert usable_prefix(reference,reference,(.33,0.))
    assert not usable_prefix(reference[:1],reference,(.33,0.))
    assert not usable_prefix(reference,reference,(2.,0.))

def test_arrival_stops_without_requiring_another_local_motion_path(monkeypatch):
    from test_history_recovery import controller
    from nomad_path_planning.command_selector import PlanningCommandNode
    monkeypatch.setattr('nomad_path_planning.command_selector.time.monotonic',lambda:10.)
    node,commands=controller();node.goal.pose.position.x=.3
    node.local_path=None;node.global_path=None
    statuses=[];node.report=statuses.append
    PlanningCommandNode.control(node)
    assert statuses[-1]=='ARRIVED'
    assert commands[-1].stop_requested and commands[-1].target_speed==0
    node.sync_ok=False
    PlanningCommandNode.control(node)
    assert statuses[-1].startswith('STOP:')  # Freshness/health remain authoritative.


def test_gpp_response_cannot_precede_goal_when_clock_delivery_lags():
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped
    from nomad_path_planning.dstar_lite_node import DStarLiteNode
    goal=PoseStamped();goal.header.stamp=Time(sec=11)
    node=NS(planning_frame='odom',goal_pose=goal,
            get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=10,nanosec=990000000))),
            cell_to_world=lambda cell:tuple(map(float,cell)),path_pub=Mock())
    DStarLiteNode.publish_path(node,[(0,0),(2,0)])
    route=node.path_pub.publish.call_args.args[0]
    assert route.header.stamp==goal.header.stamp
    assert all(p.header.stamp==route.header.stamp for p in route.poses)

def test_causal_stamp_keeps_newer_time_and_does_not_mutate_request():
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped
    from nomad_path_planning.contracts import causal_stamp
    goal=PoseStamped();goal.header.stamp=Time(sec=11)
    assert causal_stamp(Time(sec=12),goal)==Time(sec=12)
    result=causal_stamp(Time(sec=10),goal);result.sec=13
    assert goal.header.stamp.sec==11

"""Regression for base_link tangent being mistaken for vehicle yaw."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from nomad_path_planning.local_planner_node import LocalPlannerNode
from nomad_path_planning.lpp.rpp import RegulatedPurePursuit
from nomad_path_planning.lpp.rollout import AckermannRollout
from nomad_path_planning.gpp.base import Grid,Request


def fixture():
    return json.loads((Path(__file__).parent/'fixtures/rpp_map4_yaw_regression.json').read_text())


def extract(points):
    msg=RosPath()
    for x,y,yaw in points:
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y
        p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
        msg.poses.append(p)
    node=SimpleNamespace(global_path_msg=msg,
        quaternion_to_yaw=lambda q: math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    return node


def test_saved_stall_passes_with_vehicle_yaw_and_unchanged_limits():
    f=fixture();node=extract(f['path']);path=LocalPlannerNode.make_global_path(node)
    assert len(path)==81
    for actual,expected in zip(path,f['path']):assert actual==pytest.approx(expected)
    planner=RegulatedPurePursuit(speed=.4)
    result=planner.plan(f['start'],path,f['grid'])
    assert result['success']
    assert math.degrees(result['best']['steer'])==pytest.approx(f['expected_steer_deg'])
    assert abs(result['best']['steer'])<.4 and planner.lookahead==.8
    assert math.isfinite(Grid(Request(f['grid'],f['start'],path[-1][:2])).path_cost(result['best']['trajectory']))
    # Recreate the old conversion with only the supplied headings replaced by
    # centerline segment tangents. Same pose/map/lookahead then fails as before.
    tangent=[]
    for i,p in enumerate(path):
        a,b=(p,path[i+1]) if i+1<len(path) else (path[i-1],p)
        tangent.append((p[0],p[1],math.atan2(b[1]-a[1],b[0]-a[0])))
    assert not planner.plan(f['start'],tangent,f['grid'])['success']


@pytest.mark.parametrize('speed',[.4,-.2])
def test_rollout_scores_and_commands_identical_for_xy_and_xyyaw(speed):
    f=fixture();planner=AckermannRollout(speed=speed)
    xy=[p[:2] for p in f['path']]
    assert planner.plan(f['start'],f['path'],f['grid'])==planner.plan(f['start'],xy,f['grid'])


@pytest.mark.parametrize('bad',[[(0.,0.),(1.,0.)],[(0.,0.,float('nan'))],[(0.,0.,float('inf'))]])
def test_rpp_does_not_guess_missing_or_invalid_yaw(bad):
    f=fixture()
    assert not RegulatedPurePursuit(speed=.4).plan(f['start'],bad,f['grid'])['success']


def test_invalid_path_quaternion_revokes_reference():
    node=extract([(0.,0.,0.),(1.,0.,.2)])
    node.global_path_msg.poses[1].pose.orientation.w=0.
    node.global_path_msg.poses[1].pose.orientation.z=0.
    assert LocalPlannerNode.make_global_path(node)==[]


@pytest.mark.parametrize('value',[-1,100])
def test_correct_yaw_does_not_bypass_map_safety(value):
    f=fixture();f['grid']['data']=[value]*len(f['grid']['data'])
    assert not RegulatedPurePursuit(speed=.4).plan(f['start'],f['path'],f['grid'])['success']


def test_limit_circle_roundoff_clamped_and_still_collision_checked():
    from nomad_path_planning.gpp.hybrid_astar import advance
    start=(0.,0.,0.)
    path=[advance(start,-math.tan(.4)/.72,i*.075,.36) for i in range(41)]
    grid=dict(width=100,height=100,resolution=.2,origin_x=-10.,origin_y=-10.,data=[0]*10000)
    planner=RegulatedPurePursuit(speed=.4,lookahead=1.2)
    result=planner.plan(start,path,grid)
    assert result['success'] and result['best']['steer']==-.4
    point=result['best']['trajectory'][15]
    cell=planner.world_to_cell(*point[:2],100,100,.2,-10.,-10.)
    grid['data'][cell[1]*100+cell[0]]=100
    assert not planner.plan(start,path,grid)['success']


def test_real_steering_excess_remains_rejected():
    from nomad_path_planning.gpp.hybrid_astar import advance
    path=[advance((0.,0.,0.),-math.tan(.401)/.72,i*.075,.36) for i in range(41)]
    grid=dict(width=100,height=100,resolution=.2,origin_x=-10.,origin_y=-10.,data=[0]*10000)
    assert not RegulatedPurePursuit(speed=.4,lookahead=1.2).plan((0.,0.,0.),path,grid)['success']


@pytest.mark.parametrize('time,base,alternative',[(205,1.2,.8),(90,.8,1.2)])
def test_adaptive_lookahead_resolves_saved_failure_without_relaxing_safety(time,base,alternative):
    f=json.loads((Path(__file__).parent/f'fixtures/rpp_adaptive_{time}.json').read_text())
    assert not RegulatedPurePursuit(speed=.4,lookahead=base).plan(f['start'],f['path'],f['grid'])['success']
    assert RegulatedPurePursuit(speed=.4,lookahead=alternative).plan(f['start'],f['path'],f['grid'])['success']
    p=RegulatedPurePursuit(speed=.4,lookahead=base,adaptive_lookahead=True)
    r=p.plan(f['start'],f['path'],f['grid']);assert r['success']
    assert abs(r['best']['steer'])<=.4 and p.horizon==1.5 and p.lookahead==base
    assert math.isfinite(Grid(Request(f['grid'],f['start'],f['path'][-1][:2])).path_cost(r['best']['trajectory']))
    f['grid']['data']=[100]*len(f['grid']['data'])
    assert not p.plan(f['start'],f['path'],f['grid'])['success']

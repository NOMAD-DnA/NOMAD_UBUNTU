import math
import pytest
from nomad_path_planning.corridor_selector import CorridorSelector, Candidate


def setup():
    grid=dict(width=100,height=100,resolution=.2,origin_x=0.,origin_y=0.,data=[0]*10000)
    selector=CorridorSelector(final_approach_radius=3.)
    path=[(2.+i*.1,5.,0.) for i in range(41)]
    selector.accept(Candidate((6.,5.),0.,4.,0.,0.),path)
    return selector,grid,path


def test_preserves_exact_suffix_despite_soft_cost_changes():
    s,g,p=setup();g['data']=[70]*10000
    target,path=s.retained_path(g,(3.,5.,0.),(18.,5.))
    assert path==p[10:]
    assert target.point==(6.,5.)


@pytest.mark.parametrize('cost',[-1,80,100])
def test_new_unknown_or_obstacle_rejects_retention(cost):
    s,g,_=setup();g['data'][25*100+20]=cost
    assert s.retained_path(g,(3.,5.,0.),(18.,5.)) is None


@pytest.mark.parametrize('start',[(3.,5.3,0.),(3.,5.,math.radians(16)),(5.2,5.,0.)])
def test_deviation_heading_or_waypoint_arrival_replans(start):
    s,g,_=setup()
    assert s.retained_path(g,start,(18.,5.)) is None


def test_final_goal_switch_and_reset():
    s,g,_=setup()
    assert s.retained_path(g,(3.,5.,0.),(5.,5.)) is None
    s.reset()
    assert s.retained_path(g,(3.,5.,0.),(18.,5.)) is None


def test_gate_blocks_retention():
    s,g,_=setup()
    class Gate:
        def blocks(self,a,b):return True
    g['gates']=[Gate()]
    assert s.retained_path(g,(3.,5.,0.),(18.,5.)) is None


def test_node_retains_with_zero_recent_support_but_stops_for_stale_input():
    from types import SimpleNamespace,MethodType
    from unittest.mock import Mock
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from nomad_path_planning.recovery_gpp import RecoveryGPP
    s,g,p=setup()
    g['cell_weights']=[1.]*10000
    m=OccupancyGrid();m.header.stamp.sec=10;m.header.frame_id='odom'
    m.info.width=m.info.height=100;m.info.resolution=.2;m.info.origin.orientation.w=1.
    m.data=g['data']
    support=OccupancyGrid();support.header=m.header;support.info=m.info;support.data=[0]*10000
    pose=PoseStamped();pose.header=m.header;pose.pose.orientation.w=1.
    node=SimpleNamespace(costmap=m,current_pose=pose,observation_weight=0.,observation_guard=True,
        camera_support=support,retain_safe_path=True,retry_pending=False,failed_since=None,corridor=s,gates=[],
        unknown_penalty=1.,lpp_algorithm='rpp',algorithm='hybrid_astar',strategy=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10)),corridor_pub=Mock(),
        subgoal_pub=Mock(),path_pub=Mock(),publish_empty_path=Mock(),report=Mock(),planning_frame='odom')
    node.stop_and_retry=MethodType(RecoveryGPP.stop_and_retry,node)
    RecoveryGPP.plan_corridor(node,g,(3.,5.,0.),(18.,5.))
    node.strategy.plan.assert_not_called()
    node.path_pub.publish.assert_not_called()
    node.subgoal_pub.publish.assert_not_called()
    assert 'validated_safe_path' in node.corridor_pub.publish.call_args.args[0].data
    node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=15))
    RecoveryGPP.plan_corridor(node,g,(3.,5.,0.),(18.,5.))
    node.publish_empty_path.assert_called_once()
    node.path_pub.publish.assert_not_called()


def test_target_blocked_even_when_accepted_path_ends_short():
    s,g,p=setup();s.active_path=p[:-2]
    g['data'][25*100+30]=100
    assert s.retained_path(g,(3.,5.,0.),(18.,5.)) is None
    assert s.invalidate_unreachable(g,(3.,5.,0.),10.)=='target_blocked_or_unknown'
    assert s.active is None and s.active_path is None
    assert s.rejected[-1][0]==(6.,5.)


def test_new_blocked_segment_invalidates_target_too():
    s,g,_=setup();g['data'][25*100+20]=100
    assert s.invalidate_unreachable(g,(3.,5.,0.),10.)=='remaining_path_blocked_or_unknown'
    assert s.active is None


def test_slope_increase_invalidates_but_small_change_keeps_path():
    s,g,_=setup();s.accepted_slope=.1;g['slope_cost']=[.25]*10000
    assert s.invalidate_unreachable(g,(3.,5.,0.),10.) is None
    g['slope_cost'][25*100+20]=.5
    assert s.invalidate_unreachable(g,(3.,5.,0.),11.)=='remaining_path_slope_increased'
    assert s.active_path is None


def test_invalid_final_target_preserves_mission_lock_only():
    s,g,_=setup();s.final_approach_goal=s.active
    g['data'][25*100+30]=-1
    assert s.invalidate_unreachable(g,(3.,5.,0.),10.)=='target_blocked_or_unknown'
    assert s.active is None and s.active_path is None
    assert s.final_approach_goal==(6.,5.)

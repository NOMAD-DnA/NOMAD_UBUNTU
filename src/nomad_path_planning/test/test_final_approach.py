import math
from types import SimpleNamespace, MethodType
from unittest.mock import Mock
import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from nomad_path_planning.corridor_selector import CorridorSelector
from nomad_path_planning.recovery_gpp import RecoveryGPP
from nomad_path_planning.gpp.base import BudgetExceeded


def grid():
    return dict(width=80,height=80,resolution=.2,origin_x=0.,origin_y=0.,data=[0]*6400)


def test_final_approach_stays_locked_after_rejection_drift_and_heading_change():
    selector=CorridorSelector(final_approach_radius=3.,incremental=True)
    g=grid();goal=(6.,5.)
    candidate=selector.candidates(g,(4.,5.,0.),goal,0.)[0]
    assert candidate.final and candidate.point==goal
    selector.accept(candidate,[(4.,5.,0.),(6.,5.,0.)])
    selector.reject(goal,0.)
    assert not selector.rejected
    # Even after overshoot, no new forward waypoint or cooldown may replace goal.
    choices=selector.candidates(g,(10.,5.,0.),goal,1.)
    assert len(choices)==1 and choices[0].point==goal and choices[0].final
    assert selector.detail['state']=='final_goal_locked'


@pytest.mark.parametrize('cost',[-1,80,100])
def test_locked_goal_blocked_or_unknown_returns_no_exploration_candidate(cost):
    selector=CorridorSelector(final_approach_radius=3.);g=grid();goal=(6.,5.)
    selector.candidates(g,(4.,5.,0.),goal,0.)
    g['data'][25*80+30]=cost
    assert selector.candidates(g,(4.,5.,0.),goal,1.)==[]
    assert selector.detail['state']=='final_goal_not_observed_free'
    g['data'][25*80+30]=0
    assert selector.candidates(g,(4.,5.,0.),goal,2.)[0].point==goal


def test_new_mission_and_reset_release_final_approach():
    selector=CorridorSelector(final_approach_radius=3.,seconds=2.)
    selector.candidates(grid(),(4.,5.,0.),(6.,5.),0.)
    selector.candidates(grid(),(4.,5.,0.),(15.,5.),1.)
    assert selector.final_approach_goal is None
    selector.candidates(grid(),(4.,5.,0.),(6.,5.),2.)
    selector.reset()
    assert selector.final_approach_goal is None


def test_final_approach_budget_failure_clears_path_and_retries_same_goal():
    # Exercise production GPP IO branch, including stop_and_retry's empty path.
    clock=Time(seconds=10)
    m=OccupancyGrid();m.header.stamp.sec=10;m.header.frame_id='odom'
    m.info.width=m.info.height=80;m.info.resolution=.2;m.info.origin.orientation.w=1.
    m.data=[0]*6400
    pose=PoseStamped();pose.header=m.header;pose.pose.orientation.w=1.
    node=SimpleNamespace(costmap=m,current_pose=pose,observation_weight=0.,observation_guard=False,
        corridor=CorridorSelector(final_approach_radius=3.),gates=[],unknown_penalty=1.,
        lpp_algorithm='rpp',algorithm='hybrid_astar',strategy=SimpleNamespace(plan=Mock(side_effect=BudgetExceeded('test budget'))),
        get_clock=lambda:SimpleNamespace(now=lambda:clock),corridor_pub=Mock(),subgoal_pub=Mock(),path_pub=Mock(),
        retry_pending=False,publish_empty_path=Mock(),report=Mock(),planning_frame='odom')
    node.stop_and_retry=MethodType(RecoveryGPP.stop_and_retry,node)
    for start in [(4.,5.,0.),(4.,5.,0.),(10.,5.,0.)]:
        RecoveryGPP.plan_corridor(node,grid(),start,(6.,5.))
    assert node.retry_pending
    assert node.publish_empty_path.call_count==3
    assert node.strategy.plan.call_count==3
    assert all(call.args[0].goal==(6.,5.) for call in node.strategy.plan.call_args_list)
    node.path_pub.publish.assert_not_called()
    node.subgoal_pub.publish.assert_not_called()
    # A later valid plan may resume, but only toward the same mission goal.
    node.strategy.plan.side_effect=None
    node.strategy.plan.return_value=SimpleNamespace(path=[(4.,5.,0.),(5.,5.,0.),(6.,5.,0.)],detail='test valid')
    RecoveryGPP.plan_corridor(node,grid(),(4.,5.,0.),(6.,5.))
    assert not node.retry_pending
    target=node.subgoal_pub.publish.call_args.args[0].pose.position
    assert (target.x,target.y)==(6.,5.)
    assert node.path_pub.publish.call_count==1
    # An invalid future map must not release a path across an unknown/blocked cell.
    for cost in (-1,100):
        blocked=grid();blocked['data'][25*80+25]=cost
        RecoveryGPP.plan_corridor(node,blocked,(4.,5.,0.),(6.,5.))
        assert node.retry_pending
        assert node.path_pub.publish.call_count==1
    # Invalidation stops immediately, before the subsequent failed search.
    assert node.publish_empty_path.call_count==6


@pytest.mark.parametrize('radius',[-1.,float('nan'),float('inf')])
def test_invalid_approach_radius_rejected(radius):
    with pytest.raises(ValueError):CorridorSelector(final_approach_radius=radius)

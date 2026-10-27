"""ROS message contracts: valid cached references survive without republication."""
import json
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nomad_path_planning.recovery_gpp import RecoveryGPP
from nomad_path_planning.corridor_selector import Candidate
from nomad_path_planning.gpp.base import Result


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','navigation.goal_mode:=observed_corridor','-p','corridor.retain_safe_path:=true','-p','gpp_algorithm:=hybrid_astar','-p','lpp_algorithm:=rpp'])
    n=RecoveryGPP();n.path_pub=Mock();n.subgoal_pub=Mock();n.corridor_pub=Mock();n.report=Mock()
    now=n.get_clock().now().to_msg()
    grid=OccupancyGrid();grid.header.frame_id='odom';grid.header.stamp=now;grid.info.resolution=.3;grid.info.width=70;grid.info.height=50;grid.info.origin.orientation.w=1.;grid.data=[0]*3500;n.costmap=grid
    pose=PoseStamped();pose.header=grid.header;pose.pose.position.x=2.1;pose.pose.position.y=3.15;pose.pose.orientation.w=1.;n.current_pose=pose
    n.corridor.accept(Candidate((6.0,3.15),0.,3.9,1.,0.),[(2.1,3.15,0.),(6.,3.15,0.)]);n.retry_pending=False
    yield n
    n.destroy_node();rclpy.shutdown()


def view(n):
    i=n.costmap.info
    return dict(width=i.width,height=i.height,resolution=i.resolution,origin_x=0.,origin_y=0.,data=n.costmap.data)


def test_normal_check_no_target_or_path_publication(node):
    original=node.corridor.active_path;node.strategy.plan=Mock(side_effect=AssertionError('No new search expected'))
    for _ in range(3):node.plan_corridor(view(node),(2.1,3.15,0.),(18.,3.15))
    node.path_pub.publish.assert_not_called();node.subgoal_pub.publish.assert_not_called();assert node.corridor.active_path is original
    assert json.loads(node.corridor_pub.publish.call_args.args[0].data)['state']=='validated_safe_path'


def test_empty_path_retry_must_republish_even_if_geometry_safe(node):
    node.retry_pending=True
    candidate=Candidate((6.,3.15),0.,3.9,1.,0.)
    node.corridor.candidates=Mock(return_value=[candidate]);node.strategy.plan=Mock(return_value=Result(path=[(2.1,3.15,0.),(6.,3.15,0.)],detail='test'))
    node.plan_corridor(view(node),(2.1,3.15,0.),(18.,3.15))
    assert node.path_pub.publish.call_count==1;assert node.subgoal_pub.publish.call_count==1
    assert not node.retry_pending


def test_blocked_reference_invalidates_and_replacement_is_countable(node):
    node.costmap.data[10*70+13]=100
    candidate=Candidate((2.1,6.),0.,2.85,1.,0.)
    node.corridor.candidates=Mock(return_value=[candidate]);node.strategy.plan=Mock(return_value=Result(path=[(2.1,3.15,0.),(2.1,6.,1.57)],detail='test'))
    node.plan_corridor(view(node),(2.1,3.15,0.),(18.,3.15))
    calls=node.path_pub.publish.call_args_list;assert len(calls)==2;assert not calls[0].args[0].poses;assert calls[1].args[0].poses
    event=json.loads(node.corridor_pub.publish.call_args.args[0].data)
    assert event['publication']=='new_path' and event['replanned_after_invalidity']


def test_stale_input_cannot_silently_keep_path(node):
    node.costmap.header.stamp.sec-=4
    node.plan_corridor(view(node),(2.1,3.15,0.),(18.,3.15))
    assert not node.path_pub.publish.call_args.args[0].poses;assert node.retry_pending


def test_pose_deviation_counts_replan_but_same_target_is_not_republished(node):
    node.published_navigation_target=(6.,3.15)
    candidate=Candidate((6.,3.15),0.,3.9,1.,0.)
    node.corridor.candidates=Mock(return_value=[candidate])
    node.strategy.plan=Mock(return_value=Result(path=[(2.1,3.5,0.),(6.,3.15,0.)],detail='test'))
    node.plan_corridor(view(node),(2.1,3.5,0.),(18.,3.15))
    node.path_pub.publish.assert_called_once()
    node.subgoal_pub.publish.assert_not_called()
    event=json.loads(node.corridor_pub.publish.call_args.args[0].data)
    assert event['replanned_after_invalidity']
    assert event['validation_reason']=='pose_alignment_exceeded'

from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nomad_path_planning.command_selector import PlanningCommandNode
from nomad_path_planning.recovery_supervisor import RecoverySupervisor


def pose(x,t=11):
    p=PoseStamped();p.header.frame_id='odom';p.header.stamp.sec=t;p.pose.position.x=float(x);p.pose.orientation.w=1.;return p


def test_target_path_ordering_and_mission_goal_preserved():
    node=NS(goal=pose(20,10),goal_epoch=10.,planning_frame='odom',navigation_target=None,
        goal_mode='observed_corridor',global_path=None,local_path=None,recovery_active=False,
        pose=None,pending_global=None,stop=Mock(),report=Mock())
    node.on_global_path=MethodType(PlanningCommandNode.on_global_path,node)
    path=Path();path.header=pose(3).header;path.poses=[pose(0),pose(3)]
    node.on_global_path(path)
    assert node.global_path is None
    PlanningCommandNode.on_navigation_target(node,pose(3))
    assert node.global_path is path and node.goal.pose.position.x==20
    # A different subgoal invalidates old motion; stale previous-mission targets ignored.
    PlanningCommandNode.on_navigation_target(node,pose(6,12))
    assert node.global_path is None
    PlanningCommandNode.on_navigation_target(node,pose(3,9))
    assert node.navigation_target.pose.position.x==6


def test_subgoal_does_not_clear_recovery_history_or_change_mission():
    trail=NS(points=[1,2,3]);node=NS(frame='odom',goal=(20.,0.),mission_stamp=10.,trail=trail,mode='REVERSE')
    RecoverySupervisor.on_navigation_target(node,pose(3))
    assert node.navigation_target==(3.,0.)
    assert node.goal==(20.,0.) and node.trail.points==[1,2,3] and node.mode=='REVERSE'

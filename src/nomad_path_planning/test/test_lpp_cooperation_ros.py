"""GPP adapter/transport tests without Gazebo or command publishers."""
import math
import time
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nomad_interfaces.msg import LppProgress, PlannedMotion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nomad_path_planning.recovery_gpp import RecoveryGPP
from nomad_path_planning.corridor_selector import CorridorSelector, Candidate
from nomad_path_planning.lpp_cooperation import Cooperation


def pose(x,y=5.):
    p=PoseStamped();p.header.frame_id='odom';p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.
    return p


def setup_node():
    path=Path();path.header.frame_id='odom';path.header.stamp.sec=9
    path.poses=[pose(2+i*.1) for i in range(61)]
    selector=CorridorSelector();selector.accept(Candidate((8.,5.),0.,6.,2.,0.),[(2.,5.,0.),(8.,5.,0.)])
    node=NS(cooperation_enabled=True,goal_mode='observed_corridor',planning_frame='odom',
        cooperation=Cooperation(),cooperation_reference=path,failed_since=None,
        corridor=selector,corridor_pub=Mock(),report=Mock(),publish_empty_path=Mock(),retry_pending=False)
    node.on_lpp_progress=MethodType(RecoveryGPP.on_lpp_progress,node)
    msg=LppProgress();msg.header.frame_id='odom';msg.header.stamp.sec=10
    msg.reference_stamp=path.header.stamp;msg.state=LppProgress.AVOIDING;msg.rejoin_index=40
    msg.motion=PlannedMotion();msg.motion.target_speed=.4
    msg.motion.path.header=msg.header;msg.motion.path.poses=[pose(2+i*.1,4.) for i in range(9)]
    grid=dict(width=100,height=100,resolution=.1,origin_x=0.,origin_y=0.,data=[0]*10000)
    grid['data'][50*100+40]=100
    return node,msg,grid


def test_adapter_holds_blocked_reference_without_republishing_or_resetting_target():
    node,msg,grid=setup_node();node.on_lpp_progress(msg)
    assert RecoveryGPP.cooperative_hold(node,grid,(2.,4.,0.),10.)
    assert node.corridor.active==(8.,5.)
    node.publish_empty_path.assert_not_called()
    assert 'lpp_avoidance_hold' in node.corridor_pub.publish.call_args.args[0].data


@pytest.mark.parametrize('kind',['blocked','request','expired','suffix','invalid_motion'])
def test_adapter_revokes_hold_and_releases_retention(kind):
    node,msg,grid=setup_node();node.on_lpp_progress(msg)
    assert RecoveryGPP.cooperative_hold(node,grid,(2.,4.,0.),10.)
    msg.header.stamp.nanosec=100000000
    if kind=='blocked':msg.state=LppProgress.BLOCKED
    if kind=='request':msg.request_replan=True
    if kind=='suffix':grid['data'][50*100+70]=100
    if kind=='invalid_motion':msg.motion.path.poses[0].pose.orientation.w=0.
    node.on_lpp_progress(msg)
    now=11. if kind=='expired' else 10.1
    assert not RecoveryGPP.cooperative_hold(node,grid,(2.,4.,0.),now)
    assert node.corridor.active is None and node.corridor.active_path is None
    node.publish_empty_path.assert_called_once()
    assert node.retry_pending


def test_optional_contract_transport_and_legacy_node_startup(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','188')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    rclpy.init(args=['--ros-args','-p','navigation.goal_mode:=observed_corridor'])
    nodes=[];ex=SingleThreadedExecutor()
    try:
        gpp=RecoveryGPP();nodes.append(gpp)
        harness=Node('cooperation_transport_test');nodes.append(harness)
        for node in nodes:ex.add_node(node)
        publisher=harness.create_publisher(LppProgress,'/nomad/planning/lpp_progress',1)
        _,msg,_=setup_node();msg.header.stamp=harness.get_clock().now().to_msg()
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline and gpp.cooperation.message is None:
            publisher.publish(msg);ex.spin_once(timeout_sec=.05)
        assert gpp.cooperation.message is not None
        assert gpp.cooperation.message.reference==(9,0)
        assert gpp.cooperation.message.state==LppProgress.AVOIDING
        assert not gpp.cooperation.active # delivery alone grants no lease
        # Real node clears the coordination lease when invalidating its path.
        gpp.cooperation.active=True;gpp.cooperation_reference=Path()
        gpp.publish_empty_path()
        assert not gpp.cooperation.active and gpp.cooperation_reference is None
    finally:
        ex.shutdown()
        for node in reversed(nodes):node.destroy_node()
        rclpy.shutdown()

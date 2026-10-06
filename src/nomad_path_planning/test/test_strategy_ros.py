"""Isolated-domain ROS transport tests; no Gazebo or actuator nodes are started."""
import math
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Path
from nomad_interfaces.msg import PlannedMotion
from nomad_path_planning.recovery_gpp import RecoveryGPP
from nomad_path_planning.local_planner_node import LocalPlannerNode


@pytest.mark.parametrize('gpp,lpp',[
    ('hybrid_astar','rpp'),('state_lattice','rpp'),('dstar_lite','rollout'),
    ('astar','rollout'),('weighted_astar','rollout'),('field_dstar','rpp')])
def test_selected_pair_publishes_motion_and_recovers_from_invalid_map(monkeypatch,gpp,lpp):
    monkeypatch.setenv('ROS_DOMAIN_ID','187')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    rclpy.init(args=['--ros-args','-p',f'gpp_algorithm:={gpp}','-p',f'lpp_algorithm:={lpp}',
                     '-p','require_sensor_health:=false','-p','managed_recovery:=true',
                     '-p','gpp.time_budget:=3.0'])
    nodes=[]; executor=SingleThreadedExecutor()
    try:
        global_node=RecoveryGPP(); nodes.append(global_node)
        local_node=LocalPlannerNode(); nodes.append(local_node)
        harness=Node('strategy_test'); nodes.append(harness)
        for node in nodes: executor.add_node(node)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps=harness.create_publisher(OccupancyGrid,'/nomad/costmap',latched)
        poses=harness.create_publisher(PoseStamped,'/nomad/current_pose',10)
        goals=harness.create_publisher(PoseStamped,'/nomad/goal',latched)
        paths=[]; motions=[]
        harness.create_subscription(Path,'/nomad/global_path',paths.append,latched)
        harness.create_subscription(PlannedMotion,'/nomad/planning/local_motion',motions.append,10)
        grid=OccupancyGrid(); grid.header.frame_id='odom'
        grid.info.width=50; grid.info.height=40; grid.info.resolution=.2
        grid.info.origin.position.x=-2.; grid.info.origin.position.y=-4.
        grid.info.origin.orientation.w=1.; grid.data=[0]*2000
        pose=PoseStamped(); pose.header.frame_id='odom'; pose.pose.orientation.w=1.
        goal=PoseStamped(); goal.header.frame_id='odom'; goal.pose.orientation.w=1.
        goal.pose.position.x=5.; goal.pose.position.y=1.
        def spin_until(predicate, timeout=7.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                poses.publish(pose)
                executor.spin_once(timeout_sec=.03)
                if predicate(): return
            pytest.fail(f'{gpp}/{lpp}: ROS condition timed out')
        maps.publish(grid); goals.publish(goal)
        spin_until(lambda: any(m.target_speed>0 and len(m.path.poses)>1 for m in motions))
        assert paths and len(paths[-1].poses)>1
        command=next(m for m in reversed(motions) if m.target_speed>0)
        assert abs(command.target_steering_angle)<=.4 and command.target_speed<=.4
        assert math.hypot(paths[-1].poses[-1].pose.position.x-5.,paths[-1].poses[-1].pose.position.y-1.)<.3
        # Invalid map must revoke motion; republishing the identical valid map
        # must not be swallowed by the new strategy cache.
        motions.clear(); paths.clear()
        grid.header.frame_id='wrong'; maps.publish(grid)
        spin_until(lambda: any(not p.poses for p in paths) and any(m.target_speed==0 for m in motions))
        motions.clear(); paths.clear()
        grid.header.frame_id='odom'; maps.publish(grid)
        spin_until(lambda: any(m.target_speed>0 for m in motions))
    finally:
        executor.shutdown()
        for node in reversed(nodes): node.destroy_node()
        rclpy.shutdown()

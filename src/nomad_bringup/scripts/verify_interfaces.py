#!/usr/bin/env python3
"""Isolated ROS test: renamed topics, external perception/VIO, real planning/control.

No Gazebo and no real actuator topic. Refuses default/user domains and occupied domains.
"""
from pathlib import Path
import math
import os
import signal
import subprocess
import tempfile
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from nomad_interfaces.msg import ModuleStatus, DriveCommand, VehicleState


def main():
    if os.getenv('ROS_DOMAIN_ID','0') in ('0','42'):
        raise RuntimeError('Choose an unused test ROS_DOMAIN_ID, not 0/42')
    rclpy.init()
    node=Node('module_interface_fixture')
    for _ in range(10): rclpy.spin_once(node,timeout_sec=.1)
    if any(n!=node.get_name() for n in node.get_node_names()):
        node.destroy_node(); rclpy.shutdown()
        raise RuntimeError('Test ROS domain is occupied')
    directory=Path(tempfile.mkdtemp(prefix='nomad-module-test-'))
    share=Path(get_package_share_directory('nomad_bringup'))
    topics=yaml.safe_load((share/'config/topics.yaml').read_text())
    names={}
    for section,items in topics['topics'].items():
        for key in items:
            items[key]=f'/interface_test/{section}/{key}'
            names[f'{section}.{key}']=items[key]
    (directory/'topics.yaml').write_text(yaml.safe_dump(topics))
    (directory/'modules.yaml').write_text('perception: external\nvio: external\ncontrol: builtin\ntf_owner: vio\n')
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers={}
    for key,typ,qos in [
        ('sensors.odometry',Odometry,10),('localization.odometry',Odometry,10),
        ('sensors.joint_states',JointState,10),('localization.status',ModuleStatus,10),
        ('perception.status',ModuleStatus,10),('planning.goal_request',PoseStamped,10),
        ('perception.global_costmap',OccupancyGrid,latched),('perception.local_costmap',OccupancyGrid,latched)]:
        publishers[key]=node.create_publisher(typ,names[key],qos)
    clock_pub=node.create_publisher(Clock,'/clock',10)
    seen={}
    for key,typ in [('planning.drive_command',DriveCommand),('control.simulator_command',Twist),
                    ('control.vehicle_state',VehicleState),('planning.global_path',RosPath),
                    ('planning.local_path',RosPath)]:
        qos=latched if key=='planning.global_path' else 10
        node.create_subscription(typ,names[key],lambda m,key=key:seen.__setitem__(key,m),qos)
    started=time.monotonic()
    health=True
    odom=Odometry()
    odom.header.frame_id,odom.child_frame_id='odom','base_link'
    odom.pose.pose.orientation.w=1.
    joints=JointState(name=['front_left_steering_joint','front_right_steering_joint'],position=[0.,0.])
    grid=OccupancyGrid()
    grid.header.frame_id='odom'
    grid.info.width=grid.info.height=80
    grid.info.resolution=.2
    grid.info.origin.position.x=grid.info.origin.position.y=-4.
    grid.info.origin.orientation.w=1.
    grid.data=[0]*6400
    def tick():
        ns=round((10.+time.monotonic()-started)*1e9)
        stamp=Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
        clock_pub.publish(Clock(clock=stamp))
        # Allow /clock to reach other processes before measurements with its stamp.
        rclpy.spin_once(node,timeout_sec=.003)
        odom.header.stamp=joints.header.stamp=grid.header.stamp=stamp
        for key in ('sensors.odometry','localization.odometry'): publishers[key].publish(odom)
        publishers['sensors.joint_states'].publish(joints)
        for key in ('perception.global_costmap','perception.local_costmap'): publishers[key].publish(grid)
        status=ModuleStatus(state=ModuleStatus.READY)
        status.header.frame_id,status.header.stamp='odom',stamp
        status.valid_for.nanosec=600_000_000
        publishers['localization.status'].publish(status)
        status.state=ModuleStatus.READY if health else ModuleStatus.WAITING
        publishers['perception.status'].publish(status)
        end=time.monotonic()+.047
        while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.001)
    def wait(predicate,timeout=15.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            tick()
            if predicate(): return
        raise AssertionError(f'Timeout, received={list(seen)}, logs={directory}')
    process=None
    try:
        with (directory/'launch.log').open('w') as log:
            process=subprocess.Popen(['ros2','launch','nomad_bringup','autonomy.launch.py',
                f'topics_file:={directory}/topics.yaml',f'modules_file:={directory}/modules.yaml'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            wait(lambda:seen.get('control.vehicle_state',VehicleState()).ready
                 and publishers['planning.goal_request'].get_subscription_count()>0
                 and 'control.simulator_command' in seen)
            assert seen['control.simulator_command'].linear.x==0.
            goal=PoseStamped()
            goal.header.frame_id='odom'
            goal.pose.position.x=5.
            goal.pose.orientation.w=1.
            publishers['planning.goal_request'].publish(goal)
            wait(lambda:seen.get('control.simulator_command',Twist()).linear.x>0.)
            assert seen['planning.drive_command'].target_speed>0.
            assert not seen['planning.drive_command'].stop_requested
            assert seen['planning.global_path'].poses and seen['planning.local_path'].poses
            subscriptions=node.get_subscriber_names_and_types_by_node('nomad_control','/')
            subscribed={name for name,_ in subscriptions}
            assert names['planning.drive_command'] in subscribed
            assert names['perception.local_costmap'] not in subscribed
            assert names['planning.local_path'] not in subscribed
            assert '/cmd_vel' not in dict(node.get_topic_names_and_types())
            # Losing one module's readiness must stop the complete command chain.
            health=False
            wait(lambda:seen['planning.drive_command'].stop_requested and
                 seen['control.simulator_command'].linear.x==0.,timeout=3.)
            print('PASS: all topic names changed; external perception/VIO; real GPP/LPP -> '
                  'DriveCommand -> control Twist; measured feedback; readiness-loss stop; '
                  f'control has no path/map subscriptions. Logs: {directory}',flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__': main()

#!/usr/bin/env python3
"""Isolated ROS pipeline regression; synthetic sensors, no Gazebo or real cmd_vel.

Run with a spare ROS_DOMAIN_ID, after sourcing the built workspace.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class Fixture(Node):
    def __init__(self, scenario):
        super().__init__('planning_test_fixture')
        self.scenario, self.t, self.x = scenario, 1.0, 0.0
        self.command = Twist()
        self.status, self.ready = '', False
        self.path_frames = set()
        self.scan_enabled = True
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint = JointState(name=['front_left_steering_joint','front_right_steering_joint'],position=[0.,0.])
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Twist, '/nomad/test/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/nomad/planning_status', self.on_status, 10)
        self.create_subscription(Bool, '/nomad/sensors_ready', self.on_health, 10)
        self.create_subscription(RosPath, '/nomad/global_path', self.on_path, 10)
        self.create_subscription(RosPath, '/nomad/local_path', self.on_path, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'scan'
        tf.transform.translation.x, tf.transform.translation.z = .15, .38
        tf.transform.rotation.w = 1.
        self.static_tf.sendTransform(tf)

    def on_command(self, msg):
        self.command = msg

    def on_status(self, msg):
        self.status = msg.data

    def on_health(self, msg):
        self.ready = msg.data

    def on_path(self, msg):
        if msg.poses:
            self.path_frames.add(msg.header.frame_id)

    def step(self, move=False):
        rclpy.spin_once(self, timeout_sec=0.)
        self.t += .05
        if move:
            self.x += self.command.linear.x*.05
        clock = Clock()
        clock.clock.sec = int(self.t)
        clock.clock.nanosec = int((self.t-int(self.t))*1e9)
        self.clock_pub.publish(clock)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.header.stamp = clock.clock
        odom.pose.pose.position.x = self.x
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = self.command.linear.x
        self.odom_pub.publish(odom)
        self.joint.header.stamp=clock.clock
        self.joint_pub.publish(self.joint)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        if self.scan_enabled:
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'scan', clock.clock
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = 2*math.pi/499
            scan.range_min, scan.range_max = .12, 12.
            values = []
            for i in range(500):
                a = scan.angle_min+i*scan.angle_increment
                value = math.inf
                walls = [] if self.scenario == 'open' else [1.3]
                if self.scenario == 'blocked':
                    walls += [-1.3]
                for wall in walls:
                    c = math.cos(a)
                    d = (wall-self.x-.15)/c if abs(c) > 1e-8 else -1
                    if d > .12 and abs(d*math.sin(a)) < 2.0:
                        value = min(value, d)
                values.append(value)
            scan.ranges = values
            self.scan_pub.publish(scan)
        until = time.monotonic()+.05
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.005)

    def wait(self, condition, timeout, move=False):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.step(move)
            if condition():
                return
        raise AssertionError(f'Timeout: status={self.status!r}, ready={self.ready}, '
                             f'cmd={self.command.linear.x}, x={self.x}')

    def goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = 3.
        msg.pose.orientation.w = 1.
        self.goal_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', choices=['open', 'reverse', 'blocked'])
    args = parser.parse_args()
    if os.getenv('ROS_DOMAIN_ID', '0') in ('0', '42'):
        raise RuntimeError('Set an unused ROS_DOMAIN_ID (not 0/42); this test emits synthetic sensors.')
    directory = Path(tempfile.mkdtemp(prefix='nomad_planning_ros_'))
    rclpy.init()
    fixture = Fixture(args.scenario)
    # Reject an occupied test domain rather than interfere with a live graph.
    for _ in range(10):
        rclpy.spin_once(fixture, timeout_sec=.1)
    if any(name != fixture.get_name() for name in fixture.get_node_names()):
        raise RuntimeError('Test domain already contains other ROS nodes')
    process = None
    try:
        with (directory/'launch.log').open('w') as log:
            import yaml
            overrides = {'nomad_perception':{'ros__parameters':{'navigation.lidar_enabled':True,'terrain.enabled':False}}}
            (directory/'overrides.yaml').write_text(yaml.safe_dump(overrides))
            process = subprocess.Popen(
                ['ros2', 'launch', 'nomad_path_planning', 'planning.launch.py',
                 'cmd_vel_topic:=/nomad/test/cmd_vel',f'params_file:={directory}/overrides.yaml'], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            fixture.wait(lambda: fixture.ready and fixture.goal_pub.get_subscription_count() > 0, 20)
            assert fixture.command.linear.x == 0., 'Moved without goal'
            fixture.goal()
            if args.scenario == 'open':
                fixture.wait(lambda: fixture.command.linear.x > 0, 15)
                fixture.wait(lambda: fixture.status == 'ARRIVED', 25, move=True)
                assert fixture.x > 2.6
                assert fixture.path_frames == {'odom'}
            elif args.scenario == 'reverse':
                # No driven history at startup: managed recovery must not invent a retreat.
                for _ in range(140):
                    fixture.step()
                    assert fixture.command.linear.x == 0., 'Reversed without driven history'
            else:
                for _ in range(80):
                    fixture.step()
                    assert fixture.command.linear.x == 0., 'Moved with front/rear blocked'
            fixture.scan_enabled = False
            fixture.wait(lambda: not fixture.ready and fixture.command.linear.x == 0., 4)
            print(f'PASS {args.scenario}: x={fixture.x:.3f}, sensor-loss stop; log={directory}', flush=True)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

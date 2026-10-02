#!/usr/bin/env python3
"""Launch the unchanged NOMAD forest in an isolated domain and test a short goal."""
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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def main():
    domain = os.getenv('ROS_DOMAIN_ID', '0')
    if domain in ('0', '42', '43') or not os.getenv('GZ_PARTITION', '').startswith('nomad_planning_test_'):
        raise RuntimeError('Use a spare ROS domain and GZ_PARTITION=nomad_planning_test_<unique>')
    directory = Path(tempfile.mkdtemp(prefix='nomad_planning_gazebo_'))
    print(f'Logs: {directory}', flush=True)
    rclpy.init()
    node = Node('nomad_gazebo_validation')
    state = {'odom': None, 'ready': False, 'status': '', 'mapping': '', 'cmd': 0.}
    def update(key, value):
        state[key] = value
    node.create_subscription(Odometry, '/odom', lambda m: update('odom', m), qos_profile_sensor_data)
    node.create_subscription(Bool, '/nomad/sensors_ready', lambda m: update('ready', m.data), 10)
    node.create_subscription(String, '/nomad/planning_status', lambda m: update('status', m.data), 10)
    node.create_subscription(String, '/nomad/mapping_status', lambda m: update('mapping', m.data), 10)
    node.create_subscription(Twist, '/cmd_vel', lambda m: update('cmd', m.linear.x), 10)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=.1)
    if any(name != node.get_name() for name in node.get_node_names()):
        raise RuntimeError('Test domain contains other nodes')
    processes, files = [], []
    try:
        for name, command in [
            ('gazebo', ['ros2', 'launch', 'nomad_gazebo', 'forest.launch.py', 'headless:=true']),
            ('planning', ['ros2', 'launch', 'nomad_path_planning', 'planning.launch.py']),
        ]:
            log = (directory/f'{name}.log').open('w')
            files.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        deadline, goal, start, last_report = time.monotonic()+150, None, None, 0.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if any(p.poll() is not None for p in processes):
                raise AssertionError(f'A launch exited; see {directory}')
            if state['ready'] and state['odom'] is not None and goal is None:
                odom = state['odom']
                p, q = odom.pose.pose.position, odom.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                start = (p.x, p.y)
                goal = PoseStamped()
                goal.header.frame_id = 'odom'
                goal.pose.position.x = p.x+1.5*math.cos(yaw)
                goal.pose.position.y = p.y+1.5*math.sin(yaw)
                goal.pose.orientation.w = 1.
                goal_pub.publish(goal)
                print(f'Goal from {start} to {(goal.pose.position.x, goal.pose.position.y)}', flush=True)
            if state['status'] == 'ARRIVED' and abs(state['cmd']) < 1e-6:
                p = state['odom'].pose.pose.position
                displacement = math.hypot(p.x-start[0], p.y-start[1])
                error = math.hypot(p.x-goal.pose.position.x, p.y-goal.pose.position.y)
                assert displacement > 1.0 and error < .4, (displacement, error)
                print(f'PASS forest Gazebo: displacement={displacement:.3f}m, goal_error={error:.3f}m', flush=True)
                break
            if time.monotonic()-last_report > 10:
                print({k: v for k, v in state.items() if k != 'odom'}, flush=True)
                last_report = time.monotonic()
        else:
            raise AssertionError(f'Forest goal timeout; status={state["status"]}, mapping={state["mapping"]}')
    finally:
        # Stop only the exact launch processes started by this test.
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            # gz's Ruby wrapper may exit while its native server survives.
            # This session/process group belongs exclusively to this test.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for log in files:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

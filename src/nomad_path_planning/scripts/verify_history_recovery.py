#!/usr/bin/env python3
"""Isolated ROS dead-end regression with real GPP/LPP/recovery/command-selector nodes.

Synthetic already-inflated maps and Ackermann vehicle feedback, no Gazebo.
The control boundary is modeled here; verify_interfaces.py checks the real controller.
A recorded forward approach is replayed, then actual commands move the test car.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from nomad_path_planning.recovery_supervisor import RecoverySupervisor
from nomad_path_planning.command_selector import PlanningCommandNode
from nomad_interfaces.msg import DriveCommand, VehicleState
from nomad_control.controller import actuator_twist
from nomad_path_planning.recovery_gpp import RecoveryGPP
from nomad_path_planning.local_planner_node import LocalPlannerNode
from nomad_path_planning.route_guard import path_is_behind


def main():
    if os.getenv('ROS_DOMAIN_ID', '0') in ('0', '42'):
        raise RuntimeError('Use a spare test ROS_DOMAIN_ID, never 0 or 42')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'managed_recovery:=true',
                    '-r', '/cmd_vel:=/nomad/test/recovery_cmd',
                    '-r', '/nomad/costmap:=/nomad/local_costmap'])
    fixture = Node('continuous_recovery_fixture')
    for _ in range(10):
        rclpy.spin_once(fixture, timeout_sec=.1)
    if any(n != fixture.get_name() for n in fixture.get_node_names()):
        fixture.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Test domain occupied')
    supervisor, controller = RecoverySupervisor(), PlanningCommandNode()
    gpp, lpp = RecoveryGPP(), LocalPlannerNode()
    executor = SingleThreadedExecutor()
    nodes = [fixture, supervisor, controller, gpp, lpp]
    for node in nodes:
        executor.add_node(node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock_pub = fixture.create_publisher(Clock, '/clock', 10)
    pose_pub = fixture.create_publisher(PoseStamped, '/nomad/current_pose', 10)
    map_pub = fixture.create_publisher(OccupancyGrid, '/nomad/local_costmap', latched)
    health_pub = fixture.create_publisher(Bool, '/nomad/sensors_ready', 10)
    goal_pub = fixture.create_publisher(PoseStamped, '/nomad/goal', latched)
    state_pub = fixture.create_publisher(VehicleState, '/nomad/control/vehicle_state', 10)
    command = Twist()
    def on_command(msg):
        nonlocal command
        command = Twist() if msg.stop_requested else actuator_twist(msg.target_speed,msg.target_steering_angle,.72)
    fixture.create_subscription(DriveCommand, '/nomad/planning/drive_command', on_command, 10)
    t, x, y, heading = 1., 0., 0., 0.
    costmap = OccupancyGrid()
    costmap.header.frame_id = 'odom'
    costmap.info.resolution = .2
    costmap.info.width, costmap.info.height = 110,80
    costmap.info.origin.position.x, costmap.info.origin.position.y = -3.,-6.
    costmap.info.origin.orientation.w = 1.
    data = []
    for cy in range(80):
        for cx in range(110):
            px,py = -3.+(cx+.5)*.2, -6.+(cy+.5)*.2
            blocked = (5.<=px<=16. and .9<=abs(py)<=2.) or (14.5<=px<=16. and abs(py)<=2.)
            data.append(100 if blocked else 0)
    costmap.data = [0]*len(data)  # The blocked passage is observed after entry.
    def spin(seconds=.025):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.001)
    def step(move=False):
        nonlocal t,x,y,heading
        if move:
            # Integrate at rear axle, publish the base_link center.
            rx,ry = x-.36*math.cos(heading),y-.36*math.sin(heading)
            rx += command.linear.x*.1*math.cos(heading)
            ry += command.linear.x*.1*math.sin(heading)
            heading += command.angular.z*.1
            x,y = rx+.36*math.cos(heading),ry+.36*math.sin(heading)
        t += .1
        clock = Clock()
        clock.clock.sec = int(t)
        clock.clock.nanosec = round((t-int(t))*1e9)
        clock_pub.publish(clock)
        spin(.003)
        pose = PoseStamped()
        pose.header.frame_id,pose.header.stamp = 'odom',clock.clock
        pose.pose.position.x,pose.pose.position.y = x,y
        pose.pose.orientation.z,pose.pose.orientation.w = math.sin(heading/2),math.cos(heading/2)
        pose_pub.publish(pose)
        state = VehicleState(speed=command.linear.x, speed_valid=True,steering_valid=True,ready=True)
        state.header.frame_id,state.header.stamp = "base_link",clock.clock
        state_pub.publish(state)
        costmap.header.stamp = clock.clock
        map_pub.publish(costmap)
        health_pub.publish(Bool(data=True))
        spin()
    try:
        for _ in range(20):
            step()
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x,goal.pose.position.y,goal.pose.orientation.w = 18.,5.,1.
        goal_pub.publish(goal)
        # Startup wait must never latch recovery without connected driven history.
        for _ in range(120):
            step()
            assert supervisor.mode == 'FOLLOW' and not controller.recovery_active
        # Replay entry history; afterward move only through controller commands.
        for i in range(1,141):
            x = i*.1
            step()
        costmap.data = data
        first_rearward = None
        for _ in range(65):
            step(move=True)
            if path_is_behind(controller.pose, controller.global_path):
                if first_rearward is None:
                    first_rearward = t
                assert command.linear.x <= 0., 'Forward command after rearward GPP route'
            if command.linear.x < 0:
                break
        assert command.linear.x < 0 and first_rearward is not None, supervisor.mode
        reversal_delay = t-first_rearward
        assert reversal_delay < 1., ('Unexpected 4s stall wait',reversal_delay)
        start_x = x
        phases, switch_x = [], None
        obstacle_tested = False
        for _ in range(1100):
            if not phases or phases[-1] != supervisor.mode:
                phases.append(supervisor.mode)
            if supervisor.mode == 'SWITCH' and switch_x is None:
                switch_x = x
                assert x < 5., ('Ended backing inside corridor',x)
            if supervisor.mode == 'FOLLOW':
                break
            if supervisor.mode == 'REVERSE' and 9. < x < 10. and not obstacle_tested:
                # Live rear obstacle halts continuous reverse without giving LPP control.
                original = list(costmap.data)
                barrier = math.floor((x-.3+3.)/.2)
                for row in range(80):
                    costmap.data[row*110+barrier] = 100
                for _ in range(10):
                    step()
                assert command.linear.x == 0. and controller.recovery_active
                costmap.data = original
                obstacle_tested = True
                for _ in range(5):
                    step()
            step(move=True)
        assert phases == ['REVERSE','SWITCH','EXIT','FOLLOW'], (phases,x,y)
        assert start_x-switch_x > 9. and y > 1. and obstacle_tested
        assert supervisor.goal == (18.,5.) and supervisor.attempts == 1
        spin(.15)
        assert not controller.recovery_active
        exit_position = (x,y)
        # After leaving the stem, normal GPP/LPP must continue to the original goal.
        for _ in range(700):
            step(move=True)
            if math.hypot(x-18.,y-5.) < .35 and command.linear.x == 0.:
                break
        assert math.hypot(x-18.,y-5.) < .35 and supervisor.mode == 'FOLLOW', (x,y,supervisor.mode)
        print(f'PASS: rearward route stopped forward immediately, reverse after {reversal_delay:.2f}s sim; '
              f'continuous reverse {start_x-switch_x:.3f}m to branch; '
              f'forward exit at ({exit_position[0]:.3f},{exit_position[1]:.3f}); rear-obstacle stop/resume; '
              f'one recovery attempt; phases={phases}; original goal reached at ({x:.3f},{y:.3f})',flush=True)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

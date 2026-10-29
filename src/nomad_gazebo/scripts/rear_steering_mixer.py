#!/usr/bin/env python3
"""Gazebo joint backend; never publishes VESC or physical servo commands."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nomad_interfaces.msg import DriveCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from rear_steering_model import Geometry, CommandGate


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class RearSteeringMixer(Node):
    def __init__(self):
        super().__init__('rear_steering_mixer')
        p=lambda name,value:self.declare_parameter(name,value).value
        self.geometry=Geometry(p('wheelbase',.72),p('track',.6),p('wheel_radius',.15),p('joint_limit',.55))
        self.gate=CommandGate(self.geometry,p('max_speed',.5),p('command_wall_timeout',.35))
        self.acceleration=p('acceleration_limit',.6)
        if not math.isfinite(self.acceleration) or self.acceleration<=0:
            raise ValueError('Invalid acceleration')
        mode=p('input_mode','drive_command')
        if mode not in ('drive_command','twist'):raise ValueError('Unknown input_mode')
        topic=p('command_topic','/nomad/vehicle/cmd_vel' if mode=='twist' else '/nomad/rear_steer/drive_command')
        self.create_subscription(Twist if mode=='twist' else DriveCommand,topic,
                                 self.receive_twist if mode=='twist' else self.receive,1)
        names=['front_left_velocity','front_right_velocity','rear_left_position','rear_right_position']
        self.outputs=[self.create_publisher(Float64,'/nomad/rear_steer/actuators/'+name,1) for name in names]
        self.status=self.create_publisher(String,'/nomad/rear_steer/status',5)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)
        self.measured_angle=None
        self.joint_stamp=self.odom_stamp=-math.inf
        self.joint_wall=self.odom_wall=-math.inf
        self.speed=0.
        self.previous_sim=None
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02,self.tick,clock=self.wall_clock)
        self.get_logger().info('Rear steering SIM only: front-axle m/s, equivalent angle rad; single configured command source')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def receive(self,msg):
        self.gate.receive(msg.target_speed,msg.target_steering_angle,seconds(msg.header.stamp),
                          seconds(msg.valid_for),msg.stop_requested,msg.header.frame_id,self.now(),time.monotonic())

    def receive_twist(self,msg):
        v,w=msg.linear.x,msg.angular.z
        if not all(map(math.isfinite,(v,w))):
            self.gate.clear('non-finite Twist');return
        if abs(v)<1e-9:
            # Twist cannot encode stationary wheel angle. A zero speed always stops.
            self.gate.clear('zero-speed Twist');return
        angle=math.atan(self.geometry.wheelbase*w/v)
        now=self.now()
        self.gate.receive(v,angle,now,.35,False,'base_link',now,time.monotonic())

    def joints(self,msg):
        try:
            left=msg.position[msg.name.index('rear_left_steering_joint')]
            right=msg.position[msg.name.index('rear_right_steering_joint')]
            angle=self.geometry.equivalent_from_joints(left,right)
            stamp=seconds(msg.header.stamp)
            if not -.05<=self.now()-stamp<=.6:raise ValueError('stale joints')
            if stamp<=self.joint_stamp:return
            self.measured_angle=angle
            self.joint_stamp,self.joint_wall=stamp,time.monotonic()
        except (ValueError,IndexError,ZeroDivisionError):
            self.joint_stamp=-math.inf

    def odometry(self,msg):
        stamp=seconds(msg.header.stamp)
        if (msg.child_frame_id!='base_link' or not math.isfinite(msg.twist.twist.linear.x)
                or not -.05<=self.now()-stamp<=.6):
            self.odom_stamp=-math.inf
        elif stamp>self.odom_stamp:self.odom_stamp,self.odom_wall=stamp,time.monotonic()

    def write(self,values):
        for publisher,value in zip(self.outputs,values):publisher.publish(Float64(data=float(value)))

    def tick(self):
        now,wall=self.now(),time.monotonic()
        target,angle=self.gate.sample(now,wall)
        ready=(-.05<=now-self.joint_stamp<=.6 and -.05<=now-self.odom_stamp<=.6
               and wall-self.joint_wall<=3 and wall-self.odom_wall<=3)
        reason=self.gate.reason
        if not ready:
            self.gate.clear('waiting for measured joints/odometry')
            target,angle=0.,0.
            reason=self.gate.reason
        # Acceleration is per simulated second, while the lease uses wall time.
        dt=0. if self.previous_sim is None else min(.1,max(0.,now-self.previous_sim))
        self.previous_sim=now
        if self.gate.command is None:
            self.speed=0.
        elif self.speed*target<0 and abs(self.speed)>.02:
            self.speed+=max(-self.acceleration*dt,min(self.acceleration*dt,-self.speed))
        else:self.speed+=max(-self.acceleration*dt,min(self.acceleration*dt,target-self.speed))
        values=self.geometry.mix(self.speed,angle)
        self.write(values)
        msg=String(data=json.dumps(dict(reason=reason,ready=ready,command_speed=self.speed,
              equivalent_angle=angle,measured_equivalent_angle=self.measured_angle,
              wheel_commands_rad_s=values[:2],rear_commands_rad=values[2:])))
        self.status.publish(msg)


def main():
    rclpy.init()
    node=RearSteeringMixer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.write((0.,0.,0.,0.))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

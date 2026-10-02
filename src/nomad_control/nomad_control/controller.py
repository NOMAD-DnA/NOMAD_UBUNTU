"""Replaceable simulator control: DriveCommand in, actuator Twist out.

No paths, goals, costmaps or recovery state enter this module.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from nomad_interfaces.msg import DriveCommand, VehicleState, ModuleStatus


def seconds(t):
    return t.sec+t.nanosec*1e-9


def command_error(msg, now, max_speed, max_steer, base_frame='base_link', max_lease=.5):
    if msg.header.frame_id != base_frame:
        return 'command frame mismatch'
    if not all(math.isfinite(v) for v in (msg.target_speed,msg.target_steering_angle)):
        return 'non-finite command'
    lease, stamp = seconds(msg.valid_for), seconds(msg.header.stamp)
    if stamp <= 0 or not 0 < lease <= max_lease or not -.05 <= now-stamp <= lease:
        return 'expired/future command or invalid lease'
    if not msg.stop_requested and (abs(msg.target_speed) > max_speed+1e-6
                                  or abs(msg.target_steering_angle) > max_steer+1e-6):
        return 'command exceeds vehicle limits'
    return None


def actuator_twist(speed, steer, wheelbase):
    msg = Twist()
    msg.linear.x = float(speed)
    msg.angular.z = float(speed/wheelbase*math.tan(steer))
    return msg


def centre_steering(left, right, wheelbase, track):
    """Recover centre-wheel curvature from both measured Ackermann joints."""
    tl, tr = math.tan(left), math.tan(right)
    curvature = .5*(tl/(wheelbase+.5*track*tl) + tr/(wheelbase-.5*track*tr))
    return math.atan(wheelbase*curvature)


class Controller(Node):
    def __init__(self):
        super().__init__('nomad_control')
        p = lambda name, default: self.declare_parameter(name,default).value
        self.base_frame = p('base_frame','base_link')
        self.max_speed, self.max_steer = p('max_speed',.4),p('max_steer',.4)
        self.wheelbase, self.track = p('wheelbase',.72),p('track',.60)
        self.command_timeout = p('command_wall_timeout',.35)
        self.rate = p('control_rate',20.)
        self.left_joint = p('left_steering_joint','front_left_steering_joint')
        self.right_joint = p('right_steering_joint','front_right_steering_joint')
        if not all(math.isfinite(v) and v>0 for v in
                   (self.max_speed,self.max_steer,self.wheelbase,self.track,self.command_timeout,self.rate)):
            raise ValueError('Control limits/timing must be finite and positive')
        self.command = None
        self.command_wall = -math.inf
        self.latest_command_stamp = -math.inf
        self.speed = self.steer = 0.
        self.speed_stamp = self.steer_stamp = -math.inf
        self.speed_wall = self.steer_wall = -math.inf
        self.last_sim_time = None
        self.last_clock_change = time.monotonic()
        self.clock_fault = False
        self.reason = 'WAIT: command and measurements'
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.state_pub = self.create_publisher(VehicleState,'/nomad/control/vehicle_state',10)
        self.status_pub = self.create_publisher(ModuleStatus,'/nomad/control/status',10)
        self.create_subscription(DriveCommand,'/nomad/planning/drive_command',self.on_command,1)
        sensor_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry,'/odom',self.on_odometry,sensor_qos)
        self.create_subscription(JointState,'/joint_states',self.on_joints,sensor_qos)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1./self.rate,self.tick,clock=self.wall_clock)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_command(self,msg):
        stamp = seconds(msg.header.stamp)
        # Replayed packets cannot renew a lease or a wall-clock timeout.
        if stamp < self.latest_command_stamp:
            return
        if stamp == self.latest_command_stamp:
            # A same-tick stop must override motion, but duplicates cannot renew it.
            if msg.stop_requested:
                self.command = None
                self.pub.publish(Twist())
            return
        error = command_error(msg,self.now(),self.max_speed,self.max_steer,self.base_frame)
        if error:
            self.command = None
            self.reason = error
            self.pub.publish(Twist())
            return
        self.latest_command_stamp = stamp
        self.command,self.command_wall = msg,time.monotonic()
        if msg.stop_requested:
            self.pub.publish(Twist())

    def on_odometry(self,msg):
        stamp = seconds(msg.header.stamp)
        speed = msg.twist.twist.linear.x
        if (msg.child_frame_id != self.base_frame or not math.isfinite(speed)
                or not -.05 <= self.now()-stamp <= .6):
            self.speed_stamp = -math.inf
            return
        if stamp > self.speed_stamp:
            self.speed,self.speed_stamp,self.speed_wall = float(speed),stamp,time.monotonic()

    def on_joints(self,msg):
        stamp = seconds(msg.header.stamp)
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
            if not all(math.isfinite(v) and abs(v)<1.2 for v in (left,right)):
                raise ValueError('invalid joint angle')
            angle = centre_steering(left,right,self.wheelbase,self.track)
            if not math.isfinite(angle) or not -.05 <= self.now()-stamp <= .6:
                raise ValueError('stale joint angle')
        except (ValueError,IndexError,ZeroDivisionError):
            self.steer_stamp = -math.inf
            return
        if stamp > self.steer_stamp:
            self.steer,self.steer_stamp,self.steer_wall = angle,stamp,time.monotonic()

    def tick(self):
        now,wall = self.now(),time.monotonic()
        if self.last_sim_time is not None and now < self.last_sim_time-.05:
            self.clock_fault = True
        if self.last_sim_time != now:
            self.last_clock_change = wall
            self.last_sim_time = now
        speed_valid = -.05 <= now-self.speed_stamp <= .6 and wall-self.speed_wall <= 3.
        steer_valid = -.05 <= now-self.steer_stamp <= .6 and wall-self.steer_wall <= 3.
        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.base_frame
        state.speed,state.steering_angle = self.speed,self.steer
        state.speed_valid,state.steering_valid = speed_valid,steer_valid
        state.fault = self.clock_fault
        state.ready = bool(speed_valid and steer_valid and not state.fault and wall-self.last_clock_change <= .5)
        state.detail = 'SIM: measured odometry and steering joints' if state.ready else 'WAIT: fresh measurements/clock; restart after clock reset'
        self.state_pub.publish(state)
        reason = None
        if not state.ready:
            reason = state.detail
        elif self.command is None:
            reason = 'WAIT: DriveCommand'
        elif wall-self.command_wall > self.command_timeout:
            reason = 'STOP: command wall timeout'
        else:
            reason = command_error(self.command,now,self.max_speed,self.max_steer,self.base_frame)
        output = Twist()
        if reason is None:
            if self.command.stop_requested:
                reason = 'STOP: planning request'
            elif self.speed*self.command.target_speed < 0 and abs(self.speed) > .03:
                reason = 'STOP: brake before direction change'
            else:
                output = actuator_twist(self.command.target_speed,self.command.target_steering_angle,self.wheelbase)
                reason = 'RUNNING'
        self.pub.publish(output)
        self.reason = reason
        status = ModuleStatus()
        status.header = state.header
        status.state = ModuleStatus.FAULT if state.fault else (ModuleStatus.READY if state.ready else ModuleStatus.WAITING)
        status.valid_for.nanosec = 600_000_000
        status.detail = reason
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

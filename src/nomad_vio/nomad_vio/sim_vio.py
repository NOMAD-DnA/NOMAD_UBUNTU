"""Ground-truth substitute for VIO. It performs no visual-inertial estimation."""
from copy import deepcopy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from nomad_interfaces.msg import ModuleStatus


def valid_odometry(msg, frame, child):
    p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
    return (msg.header.frame_id == frame and msg.child_frame_id == child
            and all(math.isfinite(v) for v in values)
            and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .02)


class SimVIO(Node):
    def __init__(self):
        super().__init__('nomad_vio')
        self.frame = self.declare_parameter('planning_frame', 'odom').value
        self.child = self.declare_parameter('base_frame', 'base_link').value
        self.last_stamp = self.last_wall = -math.inf
        self.last_now = None
        self.fault = False
        self.pub = self.create_publisher(Odometry, '/nomad/localization/odometry', 10)
        self.status_pub = self.create_publisher(ModuleStatus, '/nomad/localization/status', 10)
        self.create_subscription(Odometry, '/odom', self.receive, qos_profile_sensor_data)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.wall_clock)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def receive(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (self.fault or not valid_odometry(msg, self.frame, self.child)
                or not -.05 <= self.now()-stamp <= .6):
            self.last_stamp = -math.inf
            return
        if stamp <= self.last_stamp:
            return
        self.last_stamp, self.last_wall = stamp, time.monotonic()
        self.pub.publish(deepcopy(msg))  # Preserve measurement time and covariance.

    def tick(self):
        now = self.now()
        if self.last_now is not None and now < self.last_now-.05:
            self.fault = True
        self.last_now = now
        ready = not self.fault and -.05 <= now-self.last_stamp <= .6 and time.monotonic()-self.last_wall <= 3.
        msg = ModuleStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.valid_for.nanosec = 600_000_000
        msg.state = ModuleStatus.FAULT if self.fault else (ModuleStatus.READY if ready else ModuleStatus.WAITING)
        msg.detail = 'SIM_GROUND_TRUTH: Gazebo owns odom -> base_link TF' if ready else 'WAIT: fresh valid simulator odometry; restart after clock reset'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimVIO()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

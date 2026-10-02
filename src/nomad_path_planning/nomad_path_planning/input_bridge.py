"""Validate public module inputs and adapt them to planning's private poses/health."""
from copy import deepcopy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nomad_interfaces.msg import ModuleStatus


def seconds(t):
    return t.sec+t.nanosec*1e-9


def status_ready(msg, now):
    return (msg is not None and msg.state == ModuleStatus.READY
            and 0 < seconds(msg.valid_for) <= 1.
            and -.05 <= now-seconds(msg.header.stamp) <= seconds(msg.valid_for))


class PlanningInputs(Node):
    def __init__(self):
        super().__init__('nomad_planning_inputs')
        self.frame = self.declare_parameter('planning_frame', 'odom').value
        self.child = self.declare_parameter('base_frame', 'base_link').value
        self.pose = None
        self.pose_stamp = self.pose_wall = -math.inf
        self.perception = self.localization = None
        self.perception_wall = self.localization_wall = -math.inf
        self.last_now = None
        self.fault = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, '/nomad/current_pose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/nomad/goal', latched)
        self.health_pub = self.create_publisher(Bool, '/nomad/sensors_ready', 10)
        self.create_subscription(Odometry, '/nomad/localization/odometry', self.on_odom, 10)
        self.create_subscription(ModuleStatus, '/nomad/perception/status', self.on_perception, 10)
        self.create_subscription(ModuleStatus, '/nomad/localization/status', self.on_localization, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.wall_clock)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_perception(self, msg):
        self.perception, self.perception_wall = msg, time.monotonic()

    def on_localization(self, msg):
        self.localization, self.localization_wall = msg, time.monotonic()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = seconds(msg.header.stamp)
        if (self.fault or msg.header.frame_id != self.frame or msg.child_frame_id != self.child
                or not -.05 <= self.now()-stamp <= .6
                or not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .02):
            self.pose = None
            return
        if stamp <= self.pose_stamp:
            return
        if self.pose is not None:
            prev = self.pose.pose.position
            if math.hypot(p.x-prev.x,p.y-prev.y) > 1.+2.*(stamp-self.pose_stamp):
                self.fault = True
                self.pose = None
                return
        self.pose = PoseStamped(header=deepcopy(msg.header), pose=deepcopy(msg.pose.pose))
        self.pose_stamp, self.pose_wall = stamp, time.monotonic()
        self.pose_pub.publish(self.pose)

    def on_goal(self, msg):
        p = msg.pose.position
        if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in (p.x,p.y)):
            self.get_logger().warn('Goal rejected: finite XY coordinates in planning_frame required')
            return
        goal = deepcopy(msg)
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal)

    def tick(self):
        now, wall = self.now(), time.monotonic()
        if self.last_now is not None and now < self.last_now-.05:
            self.fault = True
        self.last_now = now
        ready = (not self.fault and self.pose is not None and -.05 <= now-self.pose_stamp <= .6
                 and wall-self.pose_wall <= 3. and wall-self.perception_wall <= 3.
                 and wall-self.localization_wall <= 3.
                 and status_ready(self.perception,now) and status_ready(self.localization,now)
                 and self.perception.header.frame_id == self.frame
                 and self.localization.header.frame_id == self.frame)
        self.health_pub.publish(Bool(data=bool(ready)))


def main(args=None):
    rclpy.init(args=args)
    node = PlanningInputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.health_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

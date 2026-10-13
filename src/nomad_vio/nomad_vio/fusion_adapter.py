"""Convert fused IMU poses to the NOMAD base-link contract.

The input frame must already be the configured planning ENU frame. No frame
renaming or GT alignment is performed. GPS-only output has no valid attitude
and is withheld from autonomy. Velocity is a finite difference, with explicitly
unknown covariance; the backend covariance is not a calibrated posterior.
"""
from copy import deepcopy
from nomad_logging import CsvLog

import math
import time
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from nomad_interfaces.msg import ModuleStatus
from tf2_ros import TransformBroadcaster


def rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > .02:
        raise ValueError('Invalid attitude')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def base_position(position, quaternion, imu_in_base):
    # This profile has parallel base/IMU axes. Rotated installations must supply
    # a separate calibrated input transform rather than copying this profile.
    p = np.asarray(position, dtype=float)
    if not np.isfinite(p).all():
        raise ValueError('Invalid position')
    return p-rotation(quaternion)@np.asarray(imu_in_base)


class FusionAdapter(Node):
    def __init__(self):
        super().__init__('nomad_fusion_adapter')
        p = lambda n, v: self.declare_parameter(n, v).value
        self.frame = p('planning_frame', 'odom')
        self.base = p('base_frame', 'base_link')
        self.imu = p('imu_frame', 'oak_imu_frame')
        self.offset = np.array(p('imu_in_base', [.46, 0., .29]))
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError('imu_in_base must be 3 finite meters')
        self.publish_tf = p('publish_tf', True)
        self.numeric_log = CsvLog('localization', ['x','y','z','qx','qy','qz','qw','vx','vy','vz','angular_z','frame_id','child_frame_id'])
        self.valid_orientation = False
        self.mode = 'WAITING'
        self.diagnostic_stamp = -math.inf
        self.pending_pose = None
        self.last_status_wall = self.last_wall = -math.inf
        self.last_stamp = -math.inf
        self.previous = None
        self.last_now = None
        self.clock_fault = False
        self.pub = self.create_publisher(Odometry, '/nomad/localization/odometry', 10)
        self.status_pub = self.create_publisher(ModuleStatus, '/nomad/localization/status', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/fusion/odometry', self.receive, 100)
        self.create_subscription(DiagnosticArray, '/fusion/status', self.diagnostics, 100)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def diagnostics(self, msg):
        for s in msg.status:
            if s.name != 'fusion':
                continue
            values = {kv.key: kv.value for kv in s.values}
            self.valid_orientation = values.get('orientation_valid') == 'true'
            self.mode = s.message
            self.diagnostic_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.last_status_wall = time.monotonic()
            if not self.valid_orientation:
                self.previous = None
                self.last_wall = -math.inf
            pending = self.pending_pose
            if pending is not None:
                stamp = pending.header.stamp.sec+pending.header.stamp.nanosec*1e-9
                if stamp <= self.diagnostic_stamp+1e-6:
                    self.pending_pose = None
                    self.receive(pending)

    def receive(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        # Cross-topic arrival order is not guaranteed. Wait for diagnostics for
        # this measurement, so GPS-only held attitude cannot leak into autonomy.
        if stamp > self.diagnostic_stamp+1e-6:
            self.pending_pose = deepcopy(msg)
            return
        if (self.clock_fault or not self.valid_orientation
                or time.monotonic()-self.last_status_wall > 1.
                or msg.header.frame_id != self.frame or msg.child_frame_id != self.imu
                or not -.05 <= now-stamp <= .6 or stamp <= self.last_stamp):
            return
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        try:
            point = base_position([pos.x, pos.y, pos.z], quat, self.offset)
        except ValueError:
            return
        out = deepcopy(msg)
        out.child_frame_id = self.base
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, point)
        # Backend diagonal values are conservative placeholders. Increase the
        # position placeholder for lever-arm attitude uncertainty.
        for i in (0, 7, 14):
            out.pose.covariance[i] += float(np.dot(self.offset, self.offset))*max(msg.pose.covariance[21], msg.pose.covariance[28], msg.pose.covariance[35])
        if self.previous is not None:
            prev_t, prev_point, prev_q = self.previous
            dt = stamp-prev_t
            if .001 < dt <= .6:
                velocity = rotation(quat).T@(point-prev_point)/dt
                # Relative rotation expressed in the current child frame.
                rel = rotation(quat).T@rotation(prev_q)
                cosine = float(np.clip((np.trace(rel)-1.)/2., -1., 1.))
                angle = math.acos(cosine)
                axis_sin = np.array([rel[2, 1]-rel[1, 2], rel[0, 2]-rel[2, 0], rel[1, 0]-rel[0, 1]])/2.
                omega = -axis_sin/dt if angle < 1e-6 else -axis_sin*angle/(math.sin(angle)*dt)
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, velocity)
                out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, omega)
        self.previous = (stamp, point, quat)
        self.last_stamp, self.last_wall = stamp, time.monotonic()
        self.numeric_log.emit(stamp,*point,q.x,q.y,q.z,q.w,
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z,
            out.twist.twist.angular.z,out.header.frame_id,out.child_frame_id)
        self.pub.publish(out)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header = deepcopy(out.header)
            tf.child_frame_id = self.base
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, point)
            tf.transform.rotation = deepcopy(q)
            self.tf.sendTransform(tf)

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_now is not None and now < self.last_now-.05:
            self.clock_fault = True
        self.last_now = now
        ready = (not self.clock_fault and self.valid_orientation
                 and -.05 <= now-self.last_stamp <= .6
                 and time.monotonic()-self.last_wall <= 3.
                 and time.monotonic()-self.last_status_wall <= 1.)
        status = ModuleStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.frame
        status.valid_for.nanosec = 600_000_000
        status.state = ModuleStatus.READY if ready else (ModuleStatus.FAULT if self.clock_fault else ModuleStatus.WAITING)
        status.detail = 'GNSS_VIO: '+self.mode if ready else 'WAIT: fresh fused pose and valid VIO attitude; '+self.mode
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FusionAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Pi 5 odom poses converted to the raw IMU used by the measured camera chain.

T_odom_rawIMU(t) = T_odom_child(t) @ T_child_rawIMU.
The fixed transform comes from /tf_static, never from simulation dimensions.
No raw Basalt fallback, extrapolation, world relabeling or TF publication.
"""
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .map_core import rigid


@dataclass(frozen=True)
class PoseConfig:
    pose_topic: str = '/nomad/localization/odometry'
    status_topic: str = '/nomad/localization/status'
    frame: str = 'odom'
    child_frame: str = 'base_link'
    raw_imu_frame: str = 'oak_imu_frame'
    max_age_s: float = .6

    def __post_init__(self):
        if (self.frame != 'odom' or any(not isinstance(v,str) or not v for v in
                (self.child_frame,self.raw_imu_frame)) or
                any(not isinstance(v,str) or not v.startswith('/') for v in
                    (self.pose_topic,self.status_topic)) or
                not math.isfinite(self.max_age_s) or not 0 < self.max_age_s <= .6):
            raise ValueError('Expected Pi 5 odom contract, absolute topics and pose age <=0.6s')


def load_config(path):
    return PoseConfig(**json.loads(Path(path).read_text()))


def test_ros_arguments(prefix):
    """Keep synthetic Planning output away from canonical consumers."""
    if not prefix.startswith('/') or prefix=='/' or not prefix.replace('/','').replace('_','').isalnum():
        raise ValueError('Invalid synthetic topic prefix')
    result=['--ros-args']
    for topic in ('/nomad/navigation_costmap','/nomad/navigation_local_costmap',
                  '/nomad/planning_cost_layers','/nomad/camera_observation_support',
                  '/nomad/perception/status','/nomad/localization/odometry',
                  '/nomad/localization/status','/tf','/tf_static'):
        result.extend(['-r',topic+':='+prefix.rstrip('/')+topic])
    return result


def ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def transform_matrix(transform):
    p,q=transform.translation,transform.rotation
    quat=np.array([q.x,q.y,q.z,q.w],float)
    if not np.isfinite(quat).all() or not .99 < np.linalg.norm(quat) < 1.01:
        raise ValueError('Invalid fixed IMU rotation')
    matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat(quat).as_matrix()
    matrix[:3,3]=[p.x,p.y,p.z]
    return rigid(matrix)


def raw_imu_row(message, fixed, received_wall):
    p,q=message.pose.pose.position,message.pose.pose.orientation
    quat=np.array([q.x,q.y,q.z,q.w],float)
    if not np.isfinite(quat).all() or not .99 < np.linalg.norm(quat) < 1.01:
        raise ValueError('Invalid Pi 5 pose rotation')
    pose=np.eye(4);pose[:3,:3]=Rotation.from_quat(quat).as_matrix();pose[:3,3]=[p.x,p.y,p.z]
    out=rigid(pose)@rigid(fixed)
    return [ns(message.header),*out[:3,3].tolist(),
            *Rotation.from_matrix(out[:3,:3]).as_quat().tolist(),received_wall]


class Pi5PoseInput:
    def __init__(self,node,config,*,lookup_fixed=None):
        from nomad_interfaces.msg import ModuleStatus
        from .planning_transport import status_qos
        self.node,self.config=node,config
        self.status=None;self.status_wall=-math.inf
        self.pose_stamp=0;self.pose_wall=-math.inf
        self.last_now=None;self.clock_fault=False;self.fixed=None
        self.reason='Waiting for Pi 5 localization status and calibrated IMU TF'
        self.buffer=self.listener=None
        if lookup_fixed is None:
            from tf2_ros import Buffer,TransformListener
            from rclpy.time import Time
            self.buffer=Buffer(node=node);self.listener=TransformListener(self.buffer,node)
            lookup_fixed=lambda: self.buffer.lookup_transform(config.child_frame,config.raw_imu_frame,Time())
        self.lookup_fixed=lookup_fixed
        self.subscription=node.create_subscription(ModuleStatus,config.status_topic,self.on_status,status_qos())
        # Receiver owns the pose subscription and forwards messages to convert().

    def _now(self):
        now=self.node.get_clock().now().nanoseconds
        if self.last_now is not None and now < self.last_now-50_000_000:self.clock_fault=True
        self.last_now=now
        return now

    def on_status(self,message):
        self.status=message;self.status_wall=time.monotonic()

    def _status_valid(self,now,wall):
        m=self.status
        if m is None:return False
        valid=m.valid_for.sec*1_000_000_000+m.valid_for.nanosec
        return (m.header.frame_id==self.config.frame and m.state==m.READY
                and 0 < valid <= 1_000_000_000 and
                -50_000_000 <= now-ns(m.header) <= valid and wall-self.status_wall <= 1.)

    def health(self):
        now=self._now();wall=time.monotonic()
        if self.clock_fault:return False,'Pi 5 input clock moved backwards; restart the session'
        if not self._status_valid(now,wall):return False,'Pi 5 localization is not READY or status expired'
        if self.fixed is None:return False,self.reason
        if not -50_000_000 <= now-self.pose_stamp <= self.config.max_age_s*1e9 or wall-self.pose_wall>1.:
            return False,'Pi 5 fused pose expired'
        return True,'Pi 5 capture-time odom poses and fixed IMU transform available'

    def convert(self,message):
        now=self._now();wall=time.monotonic();c=self.config
        if self.clock_fault or not self._status_valid(now,wall):
            raise ValueError('Waiting for fresh READY Pi 5 localization status')
        stamp=ns(message.header)
        if (message.header.frame_id!=c.frame or message.child_frame_id!=c.child_frame
                or stamp<=0 or stamp<=self.pose_stamp
                or not -50_000_000 <= now-stamp <= c.max_age_s*1e9):
            raise ValueError('Pi 5 pose has wrong frame, stale/future or reordered timestamp')
        if c.child_frame==c.raw_imu_frame:fixed=np.eye(4)
        else:
            try:
                tf=self.lookup_fixed()
                if tf.header.frame_id!=c.child_frame or tf.child_frame_id!=c.raw_imu_frame or ns(tf.header)!=0:
                    raise ValueError('Expected a static child-to-raw-IMU TF chain')
                fixed=transform_matrix(tf.transform)
            except Exception as exc:
                self.reason='Missing calibrated static TF '+c.child_frame+' <- '+c.raw_imu_frame+': '+str(exc)
                raise ValueError(self.reason) from exc
        if self.fixed is not None and not np.allclose(fixed,self.fixed,rtol=0,atol=1e-9):
            self.clock_fault=True
            raise ValueError('Fixed IMU transform changed; restart with an empty map')
        row=raw_imu_row(message,fixed,wall)
        self.fixed=fixed;self.pose_stamp=stamp;self.pose_wall=wall
        self.reason='Pi 5 odom pose accepted'
        return row

    def report(self):
        ready,reason=self.health()
        return dict(ready=ready,reason=reason,pose_topic=self.config.pose_topic,
                    frame=self.config.frame,pose_child_frame=self.config.child_frame,
                    mapping_body_frame=self.config.raw_imu_frame,
                    T_child_raw_imu=None if self.fixed is None else self.fixed.tolist())

"""ROS-only real sensor entry point. No web UI, experiment recorder or VIO owner."""
from collections import deque
import json
import os
import signal
from pathlib import Path
import tempfile
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,PointCloud,PointCloud2
from .map_core import load_extrinsics,rigid,scan_with_times,deskew
from .depth_core import DEPTH_FRAME,depth_to_world
from .depth_filter import SORSettings
from .ros_cloud import xyz_points
from .pi5_pose import Pi5PoseInput,load_config as load_pose_config
from .semantic_validation import SemanticBridge,load_config,validate_oak_camera_metadata
from .ground_slope_live import GroundSlopeWorker,sensor_mounts
from .planning_transport import status_qos,TimingDiagnostics
from .capture_timing import trace
from .cost_output import CostOutput
from .vio_health import stationary_metrics,PoseHealth
from .publisher_lock import PublisherLock


def stamp(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec


class RealPerception(Node):
    def __init__(self):
        super().__init__('nomad_real_perception')
        self.closing=False;self.fault=None;self.worker=None;self.output=None;self.semantic=None
        self.temporary=None;self.rows=[];self.first=None;self.pairs={}
        self.scans=deque(maxlen=80);self.depths=deque(maxlen=8)
        self.Tic=self.Tia=None;self.mapping=False;self.still_since=None
        try:self.configure()
        except BaseException:
            try:self.close()
            finally:self.destroy_node()
            raise

    def configure(self):
        root=Path(get_package_share_directory('nomad_perception'))/'config/real'
        param=lambda n,v:self.declare_parameter(n,v).value
        semantic_file=param('semantic_config',str(root/'semantic.real.json'))
        pose_file=param('pose_config',str(root/'pi5_pose.json'))
        lidar_file=param('lidar_calibration',str(root/'oak_g2b_selected_20261030.json'))
        self.metadata_file=Path(param('camera_metadata_file',''))
        if str(self.metadata_file)=='.':raise ValueError('camera_metadata_file must name live OAK metadata.json')
        self.profile=param('slope_profile','12cm_3s_optimized')
        if self.profile not in ('12cm_3s_optimized','reference_7cm_10s'):raise ValueError('Unknown slope_profile')
        os.environ['MAPPING_SLOPE_PROFILE']=self.profile
        self.cfg=load_config(semantic_file);self.cfg['save_evidence']=False
        self.pose_config=load_pose_config(pose_file)
        self.Tcl,_=load_extrinsics(lidar_file)
        self.publisher_lock=PublisherLock(self.resolve_topic_name('/nomad/planning_cost_layers'))
        self.pose_health=PoseHealth()
        self.time_offset_s=float(param('lidar_time_offset_ms',0.))/1000.
        self.settings=SORSettings(max_z_m=4.,sor_neighbors=80,sor_std_ratio=1.,voxel_m=.03)
        self.temporary=tempfile.TemporaryDirectory(prefix='nomad_real_perception_')
        self.state_directory=Path(self.temporary.name)
        self.pose=Pi5PoseInput(self,self.pose_config)
        self.create_subscription(Odometry,self.pose_config.pose_topic,self.on_pose,status_qos())
        self.create_subscription(LaserScan,param('scan_topic','/scan'),self.on_scan,qos_profile_sensor_data)
        self.create_subscription(PointCloud,param('lidar_points_topic','/point_cloud'),self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,param('oak_depth_topic','/oak/depth/points'),self.on_depth,qos_profile_sensor_data)
        self.semantic=SemanticBridge(self,self.cfg,self.state_directory,self.settings)
        self.semantic.grid.frame='odom'
        self.worker=GroundSlopeWorker(self.state_directory,camera_settings=self.settings,semantic_prefiltered=True,frame='odom')
        self.output=CostOutput(self,self.state_directory,self.semantic.grid.snapshot,
                               clock_verified=self.cfg['common_clock_confirmed'],readiness=self.health)
        self.timing=TimingDiagnostics(self,self.state_directory,clock_verified=self.cfg['common_clock_confirmed'],frame='odom')
        self.timer=self.create_timer(.015,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Real perception consumes external OAK/LiDAR/ZED and Pi 5 poses; no simulator, estimator or Control is started')

    def health(self):
        if self.closing:return False,'Real perception shutting down'
        if self.fault:return False,self.fault
        if self.Tic is None:return False,'Waiting for OAK camera metadata'
        if not self.mapping:return False,'Waiting for Pi 5 READY and stationary initialization'
        if self.worker is None or not self.worker.process.is_alive():return False,'Slope worker stopped'
        for topic in ('/nomad/planning_cost_layers','/nomad/navigation_costmap','/nomad/navigation_local_costmap','/nomad/camera_observation_support'):
            if len(self.get_publishers_info_by_topic(self.resolve_topic_name(topic)))>1:
                self.fault='Duplicate perception publisher on '+topic
                return False,self.fault
        return self.pose.health()

    def fail(self,error):
        self.fault=str(error)
        if self.output is not None:self.output.planning.invalidate(self.fault)
        self.get_logger().error(self.fault)

    def on_pose(self,message):
        try:row=self.pose.convert(message)
        except ValueError:return
        self.pose_health.observe(row)
        if self.pose_health.error:self.fail(self.pose_health.error);return
        if self.first is None:self.first=row
        self.rows.append(row)
        if len(self.rows)>10_000:del self.rows[:-10_000]

    def on_scan(self,message):
        if message.header.frame_id!='g2_laser':return
        value=dict(stamp_ns=stamp(message),ranges=[float(v) if np.isfinite(v) else None for v in message.ranges],
            angle_min=float(message.angle_min),angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),range_max=float(message.range_max))
        self.join(stamp(message),'scan',value)

    def on_cloud(self,message):
        if message.header.frame_id!='g2_laser':return
        channels={v.name:list(v.values) for v in message.channels}
        try:timing=trace(stamp(message),received_ros_ns=self.get_clock().now().nanoseconds)
        except ValueError:return
        value=dict(stamp_ns=stamp(message),points=[[p.x,p.y,p.z] for p in message.points],
            offsets=channels.get('stamps',[]),intensities=channels.get('intensities',[]),
            received_monotonic=time.monotonic(),timing=timing)
        self.join(stamp(message),'cloud',value)

    def join(self,key,kind,value):
        pair=self.pairs.setdefault(key,{});pair[kind]=value
        if len(pair)==2:
            self.pairs.pop(key)
            try:
                selected=scan_with_times(pair['scan'],pair['cloud'],self.Tcl)
                selected['timing']=pair['cloud']['timing'];self.scans.append(selected)
            except ValueError:pass
        while len(self.pairs)>30:self.pairs.pop(next(iter(self.pairs)))

    def on_depth(self,message):
        if message.header.frame_id!=DEPTH_FRAME:return
        try:
            points=xyz_points(message)
            if not np.isfinite(points).all() or np.any(points[:,2]<=0):return
            timing=trace(stamp(message),received_ros_ns=self.get_clock().now().nanoseconds)
        except ValueError:return
        if len(points):self.depths.append(dict(points=points,stamp_ns=stamp(message),received_monotonic=time.monotonic(),timing=timing))

    def observation_context(self):
        return (self.rows[-1][:8],sensor_mounts(self.Tic,self.Tcl,self.Tia),self.first[3],self.first[0])

    def emit(self,points,stamp_ns,source,context,timing,point_stamps):
        self.worker.submit(points,stamp_ns,source,pose=context[0],mounts=context[1],start_z=context[2],
            start_stamp_ns=context[3],timing=timing,point_stamps=point_stamps)

    def tick(self):
        if self.closing or self.fault:return
        try:
            if self.Tic is None:
                if not self.metadata_file.is_file():return
                metadata=json.loads(self.metadata_file.read_text())
                if self.cfg.get('expected_oak_camera_info') is not None:validate_oak_camera_metadata(metadata,self.cfg['expected_oak_camera_info'])
                self.Tic=rigid(metadata['calibration_provenance']['T_raw_imu_cameras'][0])
                self.Tia=rigid(metadata['calibration_provenance']['T_raw_imu_cam_a'])
            if not self.worker.process.is_alive() or self.worker.status().get('error'):
                raise RuntimeError(self.worker.status().get('error') or 'Slope worker stopped')
            if not self.pose.health()[0]:
                self.scans.clear();self.depths.clear();self.still_since=None;return
            if not self.mapping:
                stable=stationary_metrics(self.rows)
                if not stable['ready'] or stable['translation_m']>.03 or stable['rotation_deg']>2:
                    self.still_since=None;return
                if self.still_since is None:self.still_since=time.monotonic()
                if time.monotonic()-self.still_since<5:return
                self.mapping=True;self.scans.clear();self.depths.clear()
                self.get_logger().info('Pi 5 stationary initialization complete')
            for queue,source in ((self.scans,0),(self.depths,1)):
                while queue:
                    item=queue[0];ns=item['stamp_ns']
                    end=ns+round((max(item['point_time_offsets_s'])+self.time_offset_s)*1e9) if source==0 else ns
                    if end>self.rows[-1][0]:
                        if time.monotonic()-item['received_monotonic']<1.5:break
                        queue.popleft();continue
                    queue.popleft()
                    try:
                        if source==0:
                            points=deskew(item,self.rows,self.Tic,self.Tcl,self.time_offset_s)
                            point_stamps=np.int64(ns)+np.rint((item['point_time_offsets_s']+self.time_offset_s)*1e9).astype(np.int64)
                            camera=None
                        else:
                            camera=item['points'];points=depth_to_world(camera,ns,self.rows,self.Tia);point_stamps=None
                        self.semantic.submit(points,ns,source,camera,self.observation_context(),point_stamps=point_stamps,timing=item['timing'])
                    except ValueError:continue
            T_imu_zed=self.Tic@self.Tcl@np.linalg.inv(self.cfg['T_zed_optical_from_lidar'])
            self.semantic.process(self.rows,T_imu_zed,lambda *a:None,output_timed=self.emit)
        except Exception as exc:self.fail(exc)

    def close(self):
        self.closing=True
        try:
            try:
                if self.output is not None:self.output.close()
                if hasattr(self,'timing'):self.timing.close()
            finally:
                if self.worker is not None and not self.worker.closed:self.worker.close()
        finally:
            if self.temporary is not None:self.temporary.cleanup()
            if hasattr(self,'publisher_lock'):self.publisher_lock.close()


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    # Keep DDS alive until STOPPED is sent and the owned worker is reaped.
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=None
    def request_stop(signum,frame):raise KeyboardInterrupt
    previous_term=signal.signal(signal.SIGTERM,request_stop)
    try:
        node=RealPerception();rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        try:
            if node is not None:
                try:node.close()
                finally:node.destroy_node()
        finally:
            if rclpy.ok():rclpy.shutdown()
            signal.signal(signal.SIGTERM,previous_term)

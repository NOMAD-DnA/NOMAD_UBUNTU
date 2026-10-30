"""Installed executable, live DDS, synthetic sensors; no hardware or motors."""
from array import array
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nomad_interfaces.msg import ModuleStatus,PlanningCostLayers
from nomad_perception.real.pi5_pose import test_ros_arguments as ros_test_arguments
from nomad_perception.real.planning_transport import map_qos,status_qos
from nomad_perception.real.ros_cloud import cloud_message
from nomad_perception.real.publisher_lock import PublisherLock


def test_publisher_lock_rejects_duplicate_owner():
    topic='/test_lock_'+uuid.uuid4().hex
    first=PublisherLock(topic)
    try:
        import pytest
        with pytest.raises(RuntimeError):PublisherLock(topic)
    finally:first.close()
    second=PublisherLock(topic);second.close()


def test_installed_node_projects_capture_time_and_revokes_ready():
    share=Path(get_package_share_directory('nomad_perception'))/'config/real'
    entry=Path(get_package_prefix('nomad_perception'))/'lib/nomad_perception/real_perception'
    assert entry.is_file()
    prefix='/real_port_test_'+uuid.uuid4().hex
    args=ros_test_arguments(prefix)
    args.extend(['-r','/oak/depth/points:='+prefix+'/oak_points'])
    context=Context();rclpy.init(context=context,args=[])
    node=Node('real_port_test_sensor_source',context=context,cli_args=args)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    pose_pub=node.create_publisher(Odometry,'/nomad/localization/odometry',status_qos())
    status_pub=node.create_publisher(ModuleStatus,'/nomad/localization/status',status_qos())
    cloud_pub=node.create_publisher(PointCloud2,'/oak/depth/points',qos_profile_sensor_data)
    labels_pub=node.create_publisher(Image,prefix+'/labels',qos_profile_sensor_data)
    depth_pub=node.create_publisher(Image,prefix+'/depth',qos_profile_sensor_data)
    info_pub=node.create_publisher(CameraInfo,prefix+'/info',qos_profile_sensor_data)
    tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='oak_imu_frame'
    tf.transform.translation.x=.23;tf.transform.rotation.w=1.
    broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(tf)
    received={}
    for topic in ('/nomad/navigation_costmap','/nomad/navigation_local_costmap',
                  '/nomad/planning_cost_layers','/nomad/camera_observation_support','/nomad/perception/status'):
        cls=ModuleStatus if topic.endswith('/status') else PlanningCostLayers if topic.endswith('planning_cost_layers') else OccupancyGrid
        node.create_subscription(cls,topic,lambda m,t=topic:received.setdefault(t,[]).append(m),status_qos() if cls==ModuleStatus else map_qos())
    process=None
    try:
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            Tic=np.eye(4);Tia=np.array([[0,0,1,.1],[-1,0,0,-.2],[0,-1,0,.03],[0,0,0,1.]])
            metadata=folder/'metadata.json';metadata.write_text(json.dumps({'calibration_provenance':{'T_raw_imu_cameras':[Tic.tolist()],'T_raw_imu_cam_a':Tia.tolist()}}))
            cfg=json.loads((share/'semantic.example.json').read_text())
            calib=json.loads((share/'oak_g2b_selected_20261030.json').read_text())
            Tcl=np.eye(4);Tcl[:3,:3]=calib['R_camera_lidar'];Tcl[:3,3]=calib['t_camera_lidar_m']
            cfg.update(confirmed_for_recording=True,image_alignment_confirmed=True,common_clock_confirmed=True,
                calibration_provenance='synthetic test only',T_zed_optical_from_lidar=Tcl.tolist(),
                zed_optical_frame='synthetic_zed_optical',topics=dict(labels=prefix+'/labels',depth=prefix+'/depth',camera_info=prefix+'/info'),
                ground_class_ids=[7],classes={'7':dict(name='ground',kind='ground',cost=0)},
                max_point_label_dt_ms=300,max_label_depth_dt_ms=20,visibility_absolute_m=.5,
                visibility_relative=0.,dynamic_retention_s=1.,boundary_margin_px=0,save_evidence=False,
                expected_oak_camera_info=None,expected_zed_camera_info=None)
            config=folder/'semantic.json';config.write_text(json.dumps(cfg))
            log=(folder/'node.log').open('w')
            command=[sys.executable,str(entry),*args,'-p','semantic_config:='+str(config),'-p','camera_metadata_file:='+str(metadata)]
            process=subprocess.Popen(command,cwd=folder,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            x,y=np.meshgrid(np.arange(-.25,.25,.01)+.701,np.arange(-.25,.25,.01)+.001)
            landmarks=np.c_[x.ravel(),y.ravel(),2.+np.tan(np.deg2rad(10))*x.ravel()]
            started=time.monotonic();last_cloud=0.;mapping_started=None;capture_stamps=set()

            def publish(ready=True):
                nonlocal last_cloud,mapping_started
                now=time.monotonic();ns=time.time_ns();elapsed=now-started
                if any(m.state==m.READY for m in received.get('/nomad/perception/status',[])) and mapping_started is None:mapping_started=elapsed
                motion=.1*np.sin(elapsed-mapping_started) if mapping_started is not None else 0.
                m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link'
                m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9)
                m.pose.pose.position.x=1.1+motion-.23;m.pose.pose.position.y=-.7;m.pose.pose.position.z=.2;m.pose.pose.orientation.w=1.
                status=ModuleStatus();status.header=m.header;status.valid_for.nanosec=600_000_000
                status.state=status.READY if ready else status.WAITING;status_pub.publish(status);pose_pub.publish(m)
                if now-last_cloud<.25:return
                last_cloud=now;capture=ns-100_000_000;capture_stamps.add(capture)
                old_motion=.1*np.sin(elapsed-.1-mapping_started) if mapping_started is not None and elapsed-.1>=mapping_started else 0.
                points=(landmarks-np.array([old_motion,0,0])-Tia[:3,3])@Tia[:3,:3]
                cloud_pub.publish(cloud_message(points,capture,'oak_cam_a_optical_frame'))
                def image(data,encoding):
                    out=Image();out.header.frame_id='synthetic_zed_optical';out.header.stamp.sec,out.header.stamp.nanosec=divmod(capture,10**9)
                    out.height=out.width=80;out.encoding=encoding;out.step=data.strides[0];out.data=array('B',data.tobytes());return out
                labels_pub.publish(image(np.full((80,80),7,np.uint8),'mono8'))
                depth_pub.publish(image(np.full((80,80),2.,np.float32),'32FC1'))
                info=CameraInfo();info.header.frame_id='synthetic_zed_optical';info.width=info.height=80
                info.p=[50.,0.,40.,0.,0.,50.,40.,0.,0.,0.,1.,0.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info_pub.publish(info)

            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                publish();executor.spin_once(timeout_sec=.01)
                if process.poll() is not None:log.flush();raise AssertionError((folder/'node.log').read_text())
                support=received.get('/nomad/camera_observation_support',[])
                if len(received)==5 and any(max(m.data,default=0)==100 for m in support):break
            log.flush()
            assert len(received)==5,(received.keys(),(folder/'node.log').read_text())
            assert any(max(m.data,default=0)==100 for m in received['/nomad/camera_observation_support'])
            layers=received['/nomad/planning_cost_layers'][-1]
            assert layers.grid.header.frame_id=='odom'
            ns=layers.grid.header.stamp.sec*10**9+layers.grid.header.stamp.nanosec
            assert ns in capture_stamps
            costs=np.asarray(layers.slope_cost);known=costs[np.isfinite(costs)]
            assert len(known)>5 and abs(float(np.median(known))-.4)<.01
            # Nonzero Pi 5 origin must move the output grid with the sensor pose.
            assert layers.grid.info.origin.position.x>1.
            end=time.monotonic()+1.
            while time.monotonic()<end:publish(False);executor.spin_once(timeout_sec=.01)
            assert received['/nomad/perception/status'][-1].state!=ModuleStatus.READY
            process.send_signal(signal.SIGINT);process.wait(timeout=12);log.close()
            assert process.returncode==0
            assert not list(folder.rglob('*.npz')) and not list(folder.rglob('*.png'))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        executor.shutdown();node.destroy_node();context.shutdown()

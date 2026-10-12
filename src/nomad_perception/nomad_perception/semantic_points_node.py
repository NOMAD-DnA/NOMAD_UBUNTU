"""Simulation segmentation/depth adapter for the mutable semantic XYZ contract."""
from __future__ import annotations

import json
import threading
import os
from pathlib import Path
import time
import uuid
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .stage1_collection_audit import PublisherLedger, payload_sha256

from .semantic_input import ExactCaptureSynchronizer, encode_semantic_cloud, project_semantic_points, stamp_ns


class SemanticPointsNode(Node):
    def __init__(self):
        super().__init__('nomad_semantic_points')
        defaults = {'depth_topic': '/oak/depth/image_raw',
                    'depth_info_topic': '/oak/depth/camera_info',
                    'labels_topic': '/nomad/semantic_gt/oak_rgb/labels',
                    'labels_info_topic': '/nomad/semantic_gt/oak_rgb/camera_info',
                    'output_topic': '/nomad/semantic_points',
                    'metadata_topic': '/nomad/semantic_points/metadata',
                    'depth_frame': 'oak_depth_optical_frame',
                    'labels_frame': 'oak_rgb_optical_frame',
                    'stride': 2, 'min_depth_m': .4, 'max_depth_m': 8.,
                    'output_hz': 10., 'max_pending': 100, 'max_pair_age_wall_sec': 2.,
                    'pixel_center_offset_px': float(os.environ.get('NOMAD_STAGE1_PIXEL_OFFSET','0'))}
        self.declare_parameters('', list(defaults.items()))
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        p = self.settings
        if not np.isfinite(p['output_hz']) or p['output_hz'] <= 0:
            raise ValueError('output_hz must be finite and positive')
        if p['stride'] < 1 or not np.isfinite([p['min_depth_m'], p['max_depth_m']]).all() or not 0 < p['min_depth_m'] < p['max_depth_m']:
            raise ValueError('Invalid sampling stride or optical-z depth range')
        self.ledger = None
        ledger_directory=os.environ.get('NOMAD_STAGE1_LEDGER_DIRECTORY')
        if ledger_directory:
            p['max_pending']=100;p['max_pair_age_wall_sec']=2.
            self.ledger=PublisherLedger(ledger_directory,os.environ['NOMAD_STAGE1_EXPERIMENT_ID'],
                os.environ['NOMAD_STAGE1_RUN_ID'],uuid.uuid4().hex,p)
        self.finish_started_wall=None;self.last_depth_slot=None;self.selected_stamps=set()
        self.synchronizer = ExactCaptureSynchronizer(max_pending=p['max_pending'],
            max_age_wall_sec=p['max_pair_age_wall_sec'],on_event=self.audit_event)
        self.interval_ns = round(1e9 / p['output_hz'])
        self.last_published = None
        self.reset_requested = threading.Event()
        self.reset_guard = self.create_guard_condition(self.apply_clock_reset)
        self.counts = {'paired': 0, 'published': 0, 'invalid': 0, 'rate_skipped': 0,
                       'points_published': 0}
        self.last_error = ''
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        output_qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(PointCloud2, p['output_topic'], output_qos)
        self.metadata = self.create_publisher(String, p['metadata_topic'], 10)
        self.subscriptions_keepalive = [
            self.create_subscription(kind, p[stream + '_topic'],
                                     lambda message, stream=stream: self.receive(stream, message), qos)
            for stream, kind in (('depth', Image), ('labels', Image),
                                 ('depth_info', CameraInfo), ('labels_info', CameraInfo))]
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1., self.publish_metadata, clock=self.steady_clock)
        self.control_arm=self.create_service(Trigger,'/stage1/producer/arm',self.arm)
        self.control_finish=self.create_service(Trigger,'/stage1/producer/finish',self.finish)
        self.drain_timer=self.create_timer(.02,self.check_drain,clock=self.steady_clock)
        self.jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=False), post_callback=self.clock_jump)
        self.get_logger().info(f'Exact capture depth + GOOSE64 labels -> {p["output_topic"]}; '
                               'FLOAT32 metre XYZ, UINT32 class_id; no pose applied')

    def clock_jump(self, _jump):
        # A /clock callback may run outside the executor; mutate queues only
        # inside the node's normal callback thread.
        self.reset_requested.set()
        self.reset_guard.trigger()

    def apply_clock_reset(self):
        if self.reset_requested.is_set():
            self.reset_requested.clear()
            self.synchronizer.reset()
            self.last_published = None

    def audit_event(self,kind,**values):
        if kind=='synchronization_excluded' and values.get('stream')=='depth':self.selected_stamps.discard(values['stamp_ns'])
        if self.ledger is not None and not self.ledger.finished:self.ledger.event(kind,**values)

    def arm(self,request,response):
        try:
            if self.ledger is None:raise ValueError('No producer local ledger configured')
            self.ledger.arm(self.get_clock().now().nanoseconds)
            self.last_depth_slot=None;self.selected_stamps.clear()
            response.success=True;response.message=json.dumps({'start_stamp_ns':self.ledger.start_stamp})
        except Exception as error:response.success=False;response.message=str(error)
        return response

    def finish(self,request,response):
        if self.ledger is None or not self.ledger.armed:
            response.success=False;response.message='Producer not armed';return response
        if self.ledger.end_stamp is None:
            self.ledger.end_stamp=self.get_clock().now().nanoseconds
            self.finish_started_wall=time.monotonic()
            self.audit_event('end_boundary_declared',capture_stamp_ns=self.ledger.end_stamp)
        response.success=True;response.message=json.dumps({'end_stamp_ns':self.ledger.end_stamp})
        return response

    def check_drain(self):
        self.synchronizer.expire()
        if self.ledger is None or self.ledger.finished or self.finish_started_wall is None:return
        # A declared source-sync horizon expires unresolved exposures explicitly.
        # Completion requires the queues to be empty and a final fsynced ledger.
        if time.monotonic()-self.finish_started_wall>=self.settings['max_pair_age_wall_sec']:
            self.synchronizer.close()
            if self.synchronizer.pending_exposures==0:
                self.ledger.finalize(self.ledger.end_stamp,0,dict(self.counts))

    def receive(self, stream, message):
        self.apply_clock_reset()
        stamp=stamp_ns(message)
        if self.ledger is not None and self.ledger.finished:return
        if self.ledger is not None and self.ledger.end_stamp is not None and stamp>self.ledger.end_stamp:
            self.audit_event('after_end_boundary',stream=stream,stamp_ns=stamp);return
        self.audit_event('bridge_received',stream=stream,stamp_ns=stamp,payload_sha256=payload_sha256(message) if hasattr(message,'data') else None)
        # Select exposures from the depth stream BEFORE attempting synchronization.
        # Missing labels can never be relabelled as intentional rate skips.
        if stream=='depth':
            slot=stamp//self.interval_ns
            if slot!=self.last_depth_slot:
                self.selected_stamps.add(stamp);self.last_depth_slot=slot
                self.audit_event('depth_selected',stamp_ns=stamp,slot=slot)
            else:
                self.audit_event('intentional_rate_skipped',stamp_ns=stamp,slot=slot)
        capture=self.synchronizer.add(stream,message)
        if capture is None:return
        self.counts['paired']+=1
        stamp=stamp_ns(capture['depth'])
        self.audit_event('synchronized',stamp_ns=stamp)
        if stamp not in self.selected_stamps:
            self.counts['rate_skipped']+=1;return
        self.selected_stamps.remove(stamp)
        if self.ledger is not None and self.ledger.armed and stamp<self.ledger.start_stamp:
            self.audit_event('before_start_boundary',stamp_ns=stamp);return
        p=self.settings
        try:
            xyz,classes=project_semantic_points(capture['depth'],capture['labels'],capture['depth_info'],capture['labels_info'],
                stride=p['stride'],min_depth_m=p['min_depth_m'],max_depth_m=p['max_depth_m'],
                depth_frame=p['depth_frame'],labels_frame=p['labels_frame'],pixel_center_offset_px=p['pixel_center_offset_px'])
            if not len(xyz):
                self.audit_event('selected_no_valid_depth',stamp_ns=stamp);return
            cloud=encode_semantic_cloud(xyz,classes,capture['depth'].header)
            source=self.ledger.source_capture(capture,cloud,p['pixel_center_offset_px']) if self.ledger is not None else {}
        except (ValueError,TypeError,OverflowError) as error:
            self.counts['invalid']+=1;self.last_error=str(error)
            self.audit_event('selected_projection_invalid',stamp_ns=stamp,error=str(error));return
        self.publisher.publish(cloud)
        if self.ledger is not None:self.ledger.published(cloud,source)
        self.last_published=stamp;self.counts['published']+=1
        self.counts['points_published']+=len(xyz);self.last_error=''

    def destroy_node(self):
        if self.ledger is not None and not self.ledger.finished:
            self.audit_event('abnormal_shutdown',pending_exposure_items=self.synchronizer.pending_exposures)
            self.ledger.stream.flush();self.ledger.stream.close()
        return super().destroy_node()

    def publish_metadata(self):
        message = String()
        message.data = json.dumps({**self.counts, 'unpaired_dropped': self.synchronizer.dropped,
                                   'clock_resets': self.synchronizer.resets,
                                   'last_capture_stamp_ns': self.last_published,
                                   'last_error': self.last_error,
                                   'contract': 'camera-optical XYZ metres + original GOOSE64 UINT32',
                                   'input_contract_mutable': True}, sort_keys=True)
        self.metadata.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Real DDS late join plus failed/stale/partial snapshot publication contracts."""
from array import array
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy,DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
from nomad_perception.real.planning_transport import map_qos,status_qos,set_stamp,TimingDiagnostics,PlanningSnapshotPublisher


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(deepcopy(msg))


class FakeNode:
    def __init__(self,now=10_000_000_000):self.now_ns=now;self.publishers={}
    def create_publisher(self,cls,topic,qos):
        self.publishers[topic]=Publisher();return self.publishers[topic]
    def get_clock(self):
        def now():
            sec,ns=divmod(self.now_ns,10**9)
            from builtin_interfaces.msg import Time
            return NS(nanoseconds=self.now_ns,to_msg=lambda:Time(sec=sec,nanosec=ns))
        return NS(now=now)
    def create_timer(self,*a,**kw):return object()
    def destroy_timer(self,timer):pass


def grid(capture=10_000_000_000):
    msg=OccupancyGrid();msg.header.frame_id='odom';set_stamp(msg.header,capture)
    msg.info.width=msg.info.height=2;msg.info.resolution=.12
    msg.info.origin.orientation.w=1.;msg.data=array('b',[0,0,-1,100])
    return msg


def bundle(capture=10_000_000_000):
    from nomad_interfaces.msg import PlanningCostLayers
    value=PlanningCostLayers();value.grid=grid(capture)
    value.slope_cost=array('f',[.1,.2,float('nan'),.9])
    value.terrain_cost=array('f',[.3,.4,float('nan'),.9])
    return value


class DiagnosticTests(unittest.TestCase):
    def test_qos_is_explicit_and_independent(self):
        maps,states=map_qos(),status_qos()
        self.assertEqual((maps.reliability,maps.durability,maps.depth),
                         (ReliabilityPolicy.RELIABLE,DurabilityPolicy.TRANSIENT_LOCAL,1))
        self.assertEqual((states.reliability,states.durability,states.depth),
                         (ReliabilityPolicy.RELIABLE,DurabilityPolicy.VOLATILE,10))
        maps.depth=9;self.assertEqual(map_qos().depth,1)

    def test_stale_worker_failure_pause_and_clock_reset_are_reported(self):
        with tempfile.TemporaryDirectory() as folder:
            node=FakeNode();diagnostics=TimingDiagnostics(node,folder,clock_verified=True)
            target=Path(folder)/'terrain';target.mkdir()
            value=dict(sequence=1,snapshot_capture_ns=node.now_ns,
                       worker_completed_monotonic_ns=time.monotonic_ns(),source_capture_ns=[node.now_ns,0])
            (target/'timing.json').write_text(json.dumps(value))
            diagnostics.tick()
            self.assertTrue(diagnostics.report()['timing_fresh'])
            snapshot=node.publishers['/nomad/semantic_validation/capture_timing'].messages[-1]
            self.assertEqual(snapshot.header.stamp.sec,10)
            node.now_ns+=2_100_000_000;diagnostics.tick()
            self.assertFalse(diagnostics.report()['timing_fresh'])
            self.assertEqual(len(node.publishers['/nomad/semantic_validation/capture_timing'].messages),1)
            diagnostics.fault='worker died';self.assertFalse(diagnostics.report()['timing_fresh'])
            diagnostics.fault=None;diagnostics.enabled=False;self.assertFalse(diagnostics.report()['timing_fresh'])
            node.now_ns=5_000_000_000;diagnostics.tick();self.assertTrue(diagnostics.report()['clock_fault'])
            self.assertFalse(diagnostics.report()['navigation_ready'])
            diagnostics.close()

    def test_sensor_free_dds_late_join_keeps_capture_stamp(self):
        context=Context();rclpy.init(context=context,args=[])
        executor=SingleThreadedExecutor(context=context)
        publisher=Node('timing_transport_test_publisher',context=context)
        executor.add_node(publisher)
        subscriber=None
        try:
            with tempfile.TemporaryDirectory() as folder:
                target=Path(folder)/'terrain';target.mkdir()
                capture=publisher.get_clock().now().nanoseconds-400_000_000
                (target/'timing.json').write_text(json.dumps(dict(sequence=1,snapshot_capture_ns=capture,
                    worker_completed_monotonic_ns=time.monotonic_ns(),source_capture_ns=[capture,capture])))
                diagnostics=TimingDiagnostics(publisher,folder,clock_verified=True)
                diagnostics.tick()  # Publish before the subscriber exists.
                subscriber=Node('timing_transport_test_late_subscriber',context=context)
                executor.add_node(subscriber)
                received=[]
                subscriber.create_subscription(DiagnosticArray,'/nomad/semantic_validation/capture_timing',
                                               received.append,map_qos())
                deadline=time.monotonic()+5
                while not received and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.02)
                self.assertTrue(received,'Transient-local late subscriber received no snapshot')
                stamp=received[-1].header.stamp
                self.assertEqual(stamp.sec*10**9+stamp.nanosec,capture)
                values={v.key:json.loads(v.value) for v in received[-1].status[0].values}
                self.assertEqual(values['snapshot_capture_ns'],capture)
                diagnostics.close()
        finally:
            executor.shutdown()
            if subscriber is not None:subscriber.destroy_node()
            publisher.destroy_node();context.shutdown()


try:
    from nomad_interfaces.msg import PlanningCostLayers
    HAS_INTERFACE=True
except ImportError:
    HAS_INTERFACE=False


@unittest.skipUnless(HAS_INTERFACE,'Source a NOMAD interface overlay for PlanningCostLayers tests')
class PlanningPublicationTests(unittest.TestCase):
    def test_all_four_planning_topics_reach_late_dds_subscribers(self):
        from nomad_interfaces.msg import PlanningCostLayers
        context=Context();rclpy.init(context=context,args=[])
        executor=SingleThreadedExecutor(context=context)
        prefix='/timing_transport_test/run_'+uuid.uuid4().hex
        topics=('/nomad/navigation_costmap','/nomad/navigation_local_costmap',
                '/nomad/planning_cost_layers','/nomad/camera_observation_support','/nomad/perception/status')
        args=['--ros-args']
        for topic in topics:args.extend(['-r',topic+':='+prefix+topic])
        producer=Node('planning_transport_test_producer',context=context,cli_args=args)
        consumer=None
        executor.add_node(producer)
        try:
            output=PlanningSnapshotPublisher(producer)
            capture=producer.get_clock().now().nanoseconds-100_000_000
            self.assertTrue(output.publish(grid(capture),grid(capture),bundle(capture),grid(capture),
                                           capture_ns=capture,navigation_validated=True))
            consumer=Node('planning_transport_test_late_consumer',context=context,cli_args=args)
            executor.add_node(consumer)
            received={}
            for topic in ('/nomad/navigation_costmap','/nomad/navigation_local_costmap',
                          '/nomad/planning_cost_layers','/nomad/camera_observation_support'):
                cls=PlanningCostLayers if topic.endswith('planning_cost_layers') else OccupancyGrid
                consumer.create_subscription(cls,topic,lambda msg,t=topic:received.__setitem__(t,msg),map_qos())
            deadline=time.monotonic()+5
            while len(received)<4 and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
            self.assertEqual(len(received),4)
            for topic,msg in received.items():
                header=msg.grid.header if topic.endswith('planning_cost_layers') else msg.header
                self.assertEqual(header.stamp.sec*10**9+header.stamp.nanosec,capture)
            output.close()
        finally:
            executor.shutdown()
            if consumer is not None:consumer.destroy_node()
            producer.destroy_node();context.shutdown()

    def test_validated_bundle_preserves_exact_source_headers_and_nan(self):
        node=FakeNode();output=PlanningSnapshotPublisher(node)
        maps=grid();layers=bundle();support=grid()
        self.assertTrue(output.publish(maps,maps,layers,support,capture_ns=node.now_ns,navigation_validated=True))
        self.assertEqual(node.publishers['/nomad/perception/status'].messages[-1].state,1)
        saved=node.publishers['/nomad/planning_cost_layers'].messages[-1]
        self.assertNotEqual(saved.slope_cost[2],saved.slope_cost[2])
        self.assertEqual(saved.grid.header.stamp.sec,10)
        node.now_ns+=2_100_000_000;output.tick()
        self.assertEqual(node.publishers['/nomad/perception/status'].messages[-1].state,2)
        self.assertEqual(len(node.publishers['/nomad/planning_cost_layers'].messages),1)
        output.close()

    def test_mixed_frame_stamp_geometry_and_invalid_features_publish_nothing(self):
        for mutation in ('stamp','frame','origin','cost','feature'):
            node=FakeNode();output=PlanningSnapshotPublisher(node)
            maps=grid();layers=bundle();support=grid()
            if mutation=='stamp':support.header.stamp.nanosec=1
            if mutation=='frame':maps.header.frame_id='world'
            if mutation=='origin':support.info.origin.position.x=.12
            if mutation=='cost':layers.grid.data[0]=1
            if mutation=='feature':layers.slope_cost[0]=10.
            self.assertFalse(output.publish(maps,maps,layers,support,capture_ns=node.now_ns,navigation_validated=True),mutation)
            self.assertFalse(any(p.messages for topic,p in node.publishers.items() if topic!='/nomad/perception/status'))
            output.close()

    def test_duplicates_wall_timeout_unverified_and_shutdown_cannot_renew(self):
        node=FakeNode();output=PlanningSnapshotPublisher(node)
        with patch('nomad_perception.real.planning_transport.time.monotonic',return_value=20.):
            self.assertFalse(output.publish(grid(),grid(),bundle(),grid(),capture_ns=node.now_ns))
            self.assertTrue(output.publish(grid(),grid(),bundle(),grid(),capture_ns=node.now_ns,navigation_validated=True))
        with patch('nomad_perception.real.planning_transport.time.monotonic',return_value=23.1):
            output.tick()
            self.assertEqual(node.publishers['/nomad/perception/status'].messages[-1].state,2)
            self.assertFalse(output.publish(grid(),grid(),bundle(),grid(),capture_ns=node.now_ns,navigation_validated=True))
        output.close()
        self.assertFalse(output.publish(grid(),grid(),bundle(),grid(),capture_ns=node.now_ns+1,navigation_validated=True))


if __name__=='__main__':unittest.main(verbosity=2)

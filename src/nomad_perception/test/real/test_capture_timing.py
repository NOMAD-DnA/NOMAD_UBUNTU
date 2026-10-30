"""Capture provenance survives queues, plane reuse and semantic matching."""
import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np

from nomad_perception.real.capture_timing import trace, capture_range, stage, summary
from nomad_perception.real.ground_slope_live import GroundSlope, GroundSlopeWorker
from .test_ground_slope_live import mounts, plane


class CaptureTimingTests(unittest.TestCase):
    def test_integer_nanoseconds_and_stage_clocks_remain_separate(self):
        stamp = 1_793_333_333_123_456_789
        original = trace(stamp, received_ros_ns=stamp+20_000_000,
                         received_monotonic_ns=1_000_000_000,
                         point_stamps_ns=np.array([stamp,stamp+17],np.int64))
        result = stage(original,'semantic_ready',monotonic_ns=1_100_000_000)
        result = stage(result,'submitted',monotonic_ns=1_110_000_000)
        result = stage(result,'worker_started',monotonic_ns=1_310_000_000)
        result = stage(result,'worker_completed',monotonic_ns=1_610_000_000)
        report = summary(result,now_ros_ns=stamp+650_000_000,now_monotonic_ns=1_630_000_000)
        self.assertEqual(result['source_stamp_ns'],stamp)
        self.assertEqual(report['capture_last_ns'],stamp+17)
        self.assertEqual(report['semantic_wait_ms'],100.)
        self.assertEqual(report['worker_queue_ms'],200.)
        self.assertEqual(report['worker_processing_ms'],300.)
        self.assertEqual(report['oldest_point_age_ms'],650.)
        self.assertNotIn('worker_completed_monotonic_ns',original)
        with self.assertRaises(ValueError):trace(float(stamp))
        with self.assertRaises(ValueError):trace(0)
        with self.assertRaises(ValueError):capture_range(original,[0])

    def test_reused_plane_and_empty_capture_do_not_refresh_cell_observation(self):
        g=GroundSlope(mounts(),0.,100)
        pose=[1_000_000_000,0,0,0,0,0,0,1]
        points=plane(10.)
        stamps=np.arange(len(points),dtype=np.int64)+1_000_000_000
        g.ingest(points,1_000_000_000,0,None,pose,point_stamps=stamps)
        before=g.snapshot()
        self.assertEqual(int(before['snapshot_capture_ns']),int(stamps[-1]))
        self.assertGreater(before['slope_valid'].sum(),20)
        g.ingest(np.empty((0,3)),12_000_000_000,0,None,[12_000_000_000,4,0,0,0,0,0,1])
        after=g.snapshot()
        np.testing.assert_array_equal(after['last_observed_stamp_ns'],before['last_observed_stamp_ns'])
        self.assertEqual(int(after['snapshot_capture_ns']),int(stamps[-1]))

    def test_worker_transports_source_time_and_queue_delay(self):
        with tempfile.TemporaryDirectory() as folder:
            worker=GroundSlopeWorker(folder,True)
            capture=time.time_ns()-300_000_000
            points=plane();stamps=np.full(len(points),capture,dtype=np.int64)
            provenance=trace(capture,received_ros_ns=capture+10_000_000,
                             received_monotonic_ns=time.monotonic_ns()-200_000_000,
                             point_stamps_ns=stamps)
            try:
                worker.submit(points,capture,0,pose=[capture,0,0,0,0,0,0,1],mounts=mounts(),
                              start_z=0.,start_stamp_ns=100,timing=provenance,point_stamps=stamps)
                deadline=time.monotonic()+10
                path=Path(folder)/'terrain/timing.json'
                while time.monotonic()<deadline and not path.exists():
                    if worker.status().get('error'):self.fail(worker.status()['error'])
                    time.sleep(.02)
                data=json.loads(path.read_text())
                self.assertEqual(data['snapshot_capture_ns'],capture)
                self.assertEqual(data['source_stamp_ns'],capture)
                self.assertGreaterEqual(data['worker_queue_ms'],0.)
                self.assertGreater(data['local_pipeline_ms'],190.)
                self.assertFalse(data['navigation_ready'])
            finally:
                worker.close()
            live=json.loads((Path(folder)/'terrain/live.json').read_text())
            self.assertEqual(live['capture_timing']['snapshot_capture_ns'],capture)
            self.assertFalse(list(Path(folder).rglob('*.npz')))

    def test_points_outside_output_roi_do_not_refresh_snapshot_reference(self):
        g=GroundSlope(mounts(),0.,100)
        g.ingest(plane(),1_000_000_000,0,None,[1_000_000_000,0,0,0,0,0,0,1])
        before=g.snapshot()
        g.ingest(plane(offset=20.),2_000_000_000,0,None,[2_000_000_000,0,0,0,0,0,0,1])
        after=g.snapshot()
        self.assertEqual(int(after['snapshot_capture_ns']),int(before['snapshot_capture_ns']))

    def test_semantic_match_passes_original_point_and_image_times(self):
        from nomad_perception.real.semantic_validation import SemanticBridge
        from .test_semantic_validation import settings,DummyNode
        from nomad_perception.real.depth_filter import SORSettings
        with tempfile.TemporaryDirectory() as folder:
            config=settings(folder);config['save_evidence']=True
            bridge=SemanticBridge(DummyNode(),config,folder,SORSettings())
            bridge.info=(8,8,np.array([[4.,0.,4.],[0.,4.,4.],[0.,0.,1.]]))
            capture=1_050_000_000
            bridge.labels.append((capture+10_000_000,np.full((8,8),7,np.uint8)))
            bridge.depths.append((capture+12_000_000,np.full((8,8),2.,np.float32)))
            rows=[[1_000_000_000,0,0,0,0,0,0,1],[1_100_000_000,0,0,0,0,0,0,1]]
            context=([capture,0,0,0,0,0,0,1],mounts(),0.,1)
            provenance=trace(capture,received_monotonic_ns=time.monotonic_ns()-100_000_000)
            bridge.submit(np.array([[0.,0.,2.]]),capture,0,None,context,
                          point_stamps=np.array([capture+5],np.int64),timing=provenance)
            received=[]
            bridge.process(rows,np.eye(4),lambda *a:self.fail('Untimed callback used'),
                           output_timed=lambda *args:received.append(args))
            self.assertEqual(len(received),1)
            timing=received[0][4]
            self.assertEqual(timing['source_stamp_ns'],capture)
            self.assertEqual(timing['capture_last_ns'],capture+5)
            self.assertEqual(timing['label_capture_ns'],capture+10_000_000)
            self.assertEqual(timing['depth_capture_ns'],capture+12_000_000)
            self.assertGreaterEqual(summary(timing)['semantic_wait_ms'],100.)
            bridge.close(capture)
            self.assertFalse(list(Path(folder).rglob('*.npz')))


if __name__=='__main__':unittest.main(verbosity=2)

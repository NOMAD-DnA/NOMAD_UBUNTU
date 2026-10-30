import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from nomad_perception.real.ground_slope_live import GroundSlope, GroundSlopeWorker, REFERENCE, sensor_mounts, configuration


def mounts():
    m = np.array([np.eye(4), np.eye(4)])
    m[1,:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    return m


def plane(angle=10., z=-1., offset=0.):
    x,y=np.meshgrid(np.arange(.23,.79,.02)+offset,np.arange(-.24,.25,.02))
    return np.repeat(np.c_[x.ravel(),y.ravel(),z+np.tan(np.deg2rad(angle))*x.ravel()],3,axis=0)


class GroundTests(unittest.TestCase):
    def test_pinned_source_and_settings(self):
        for name,digest in json.loads((REFERENCE/'provenance.json').read_text())['files'].items():
            self.assertEqual(hashlib.sha256((REFERENCE/name).read_bytes()).hexdigest(),digest)
        c,q,p=configuration()
        self.assertEqual((c.resolution_m,c.radius_m,q.bin_size_m),(.07,.105,.03))
        self.assertEqual(q.min_raw_points,12)
        self.assertEqual(q.min_xy_bins,6)

    def test_known_planes_and_forward_roi(self):
        for angle in [0.,5.,10.,25.]:
            g=GroundSlope(mounts(),0.,100)
            p=plane(angle)
            g.ingest(np.concatenate([p,plane(angle,offset=-1.5),plane(angle,offset=3.)]),1_000_000_000,0,None,[1_000_000_000,0,0,0,0,0,0,1])
            s=g.model.snapshot()
            self.assertGreater(s['slope_valid'].sum(),20)
            np.testing.assert_allclose(s['slope_deg'][s['slope_valid']],angle,atol=1e-4)
            self.assertNotIn('cost',s)
            rows,cols=np.nonzero(s['observed'])
            centers=s['origin_xy']+(np.c_[cols,rows]+.5)*.07
            self.assertTrue(g.model.roi.contains(centers).all())

    def test_all_heights_reach_both_sources_after_preprocessing(self):
        from nomad_perception.real.depth_filter import frame_voxels
        g=GroundSlope(mounts(),0.,100)
        p=plane(z=-1.)
        above=plane(z=1.,offset=.9)
        inputs=np.r_[p,above]
        pose=[1_000_000_000,0,0,1,0,0,0,1]
        actual=g.ingest(inputs,1_000_000_000,0,None,pose)
        np.testing.assert_array_equal(actual,inputs)
        self.assertTrue((actual[:,2]>g.start_z).any())
        camera=np.c_[inputs[:,:2],np.ones(len(inputs))]
        expected=frame_voxels(camera,inputs,g.settings)[1]
        actual=g.ingest(inputs,1_000_000_000,1,camera,pose)
        np.testing.assert_allclose(actual,expected)
        self.assertTrue((actual[:,2]>g.start_z).any())
        self.assertTrue((actual[:,2]<g.start_z).any())
        self.assertEqual(g.height_excluded,[0,0])
        self.assertFalse(g.metadata()['height_selection']['enabled'])
        self.assertIsNone(g.metadata()['height_selection']['operator'])
        self.assertFalse(g.metadata()['camera_preprocessing']['repeated_frame_filter'])
        self.assertFalse(g.metadata()['camera_preprocessing']['radius_support_filter'])

    def test_known_plane_above_initial_z_and_reference_independence(self):
        outputs=[]
        for start_z in [-10.,0.,10.]:
            g=GroundSlope(mounts(),start_z,100)
            g.ingest(plane(10.,z=1.),1_000_000_000,0,None,[1_000_000_000,0,0,0,0,0,0,1])
            snap=g.model.snapshot()
            self.assertGreater(snap['slope_valid'].sum(),20)
            np.testing.assert_allclose(snap['slope_deg'][snap['slope_valid']],10.,atol=1e-4)
            outputs.append(snap)
        for snap in outputs[1:]:
            np.testing.assert_array_equal(snap['slope_valid'],outputs[0]['slope_valid'])
            np.testing.assert_allclose(snap['slope_deg'],outputs[0]['slope_deg'],equal_nan=True)

    def test_same_source_height_span_invalidates_previous(self):
        g=GroundSlope(mounts(),1.,100)
        p=plane(0.)
        pose=[1_000_000_000,0,0,0,0,0,0,1]
        g.ingest(p,1_000_000_000,0,None,pose)
        self.assertGreater(g.model.snapshot()['slope_valid'].sum(),20)
        raised=p.copy();raised[:,2]+=.2
        g.ingest(raised,1_500_000_000,0,None,pose)
        s=g.model.snapshot()
        self.assertEqual(s['slope_valid'].sum(),0)
        self.assertGreater(g.model.counts['stage3_within_sensor_conflicts'],0)
        self.assertFalse(s['slope_reused'].any())

    def test_cross_source_height_span_does_not_set_conflict(self):
        g=GroundSlope(mounts(),1.,100)
        pose=[1_000_000_000,0,0,0,0,0,0,1]
        g.ingest(plane(0.),1_000_000_000,0,None,pose)
        p=plane(0.,z=-.8)
        g.ingest(p,1_000_000_000,1,np.c_[p[:,:2],np.ones(len(p))],pose)
        g.model.snapshot()
        self.assertEqual(g.model.counts['stage3_within_sensor_conflicts'],0)
        self.assertFalse(g.model.metadata()['cross_sensor_representative_conflict_check'])

    def test_sampling_expiry_and_persistence(self):
        g=GroundSlope(mounts(),0.,100)
        pose=[1_000_000_000,0,0,0,0,0,0,1]
        g.ingest(plane(),1_000_000_000,0,None,pose)
        s=g.model.snapshot();valid=s['slope_valid'].sum()
        g.ingest(plane(20.),1_010_000_000,0,None,pose)
        self.assertEqual(g.sampled[0],1)
        g.ingest(np.empty((0,3)),12_000_000_000,0,None,[12_000_000_000,4,0,0,0,0,0,1])
        s=g.model.snapshot()
        self.assertFalse(g.model.samples)
        self.assertEqual(s['slope_valid'].sum(),valid)

    def test_worker_live_update_and_final_files(self):
        import time
        with tempfile.TemporaryDirectory() as path:
            w=GroundSlopeWorker(path,True)
            try:
                w.submit(plane(z=1.),1_000_000_000,0,pose=[1_000_000_000,0,0,0,0,0,0,1],mounts=mounts(),start_z=0.,start_stamp_ns=100)
                deadline=time.monotonic()+12
                while time.monotonic()<deadline:
                    status=w.status()
                    if status.get('modes',{}).get('fused',{}).get('valid_cells',0)>20:break
                    time.sleep(.1)
                self.assertGreater(status['modes']['fused']['valid_cells'],20)
                self.assertFalse(status['saved'])
            finally:
                status=w.close()
            self.assertTrue(status['saved'])
            live=json.loads((Path(path)/'terrain/live.json').read_text())
            self.assertTrue(status['saved'])
            self.assertGreater(live['modes']['fused']['valid_cells'],20)
            self.assertFalse(list(Path(path).rglob('*.npz')))


if __name__=='__main__':unittest.main(verbosity=2)

"""Repeated camera evidence, source deduplication and cost independence."""
from copy import deepcopy
import unittest
import numpy as np
from nomad_perception.real.observation_support import support_metrics
from nomad_perception.real.semantic_validation import SemanticGrid
from nomad_perception.real.planning_costs import build_cost_snapshot
from .test_planning_costs import inputs

STAMP=1_793_330_000_123_456_789


class SupportTests(unittest.TestCase):
    def test_single_partial_and_full_support(self):
        for offsets,expected in (([0],0),([0,300_000_000],40),
                                 ([0,250_000_000,500_000_000],100),
                                 ([0,600_000_000],67),([0,100_000_000,200_000_000],40)):
            times=[STAMP+x for x in offsets]
            score,count,span=support_metrics(times,times[-1])
            self.assertEqual(score,expected);self.assertEqual(count,len(times))
            self.assertEqual(span,offsets[-1])

    def test_hold_and_decay_keep_strength_anchored_to_last_capture(self):
        times=[STAMP,STAMP+300_000_000,STAMP+600_000_000]
        last=times[-1]
        for age,expected in ((0,100),(2.4,100),(10,100),(20,50),(30,0),(50,0)):
            self.assertEqual(support_metrics(times,last+round(age*1e9))[0],expected)
        partial=[STAMP,STAMP+600_000_000]
        self.assertEqual(support_metrics(partial,partial[-1]+10_000_000_000)[0],67)
        self.assertEqual(support_metrics(partial,partial[-1]+20_000_000_000)[0],33)

    def test_duplicates_future_and_window_do_not_invent_evidence(self):
        self.assertEqual(support_metrics([STAMP]*20,STAMP),(0,1,0))
        self.assertEqual(support_metrics([STAMP+5_000_000_000],STAMP),(0,0,0))
        self.assertEqual(support_metrics([],STAMP),(0,0,0))
        times=[STAMP,STAMP+1_000_000_000,STAMP+4_000_000_000]
        self.assertEqual(support_metrics(times,times[-1]),(67,2,3_000_000_000))

    def test_integer_ns_preserves_exact_half_second_span(self):
        times=[STAMP,STAMP+250_000_000,STAMP+500_000_000]
        self.assertEqual(support_metrics(times,times[-1]),(100,3,500_000_000))
        with self.assertRaises(ValueError):support_metrics([float(STAMP)],STAMP)

    def grid(self):
        return SemanticGrid(dict(grid_resolution_m=.12,dynamic_retention_s=1.,
            class_table={1:dict(name='ground',kind='ground',cost=20),
                         2:dict(name='other',kind='other',cost=None)}))

    def test_one_label_image_counts_once_across_points_classes_and_sensors(self):
        grid=self.grid();points=np.array([[.01,.01,0.],[.02,.02,0.]])
        for source in (0,1,0,1):
            grid.add(points,np.array([1,2]),STAMP,source,camera_stamp_ns=STAMP)
        cell=grid.snapshot(STAMP)['cells'][0]
        self.assertEqual(cell['camera_capture_ns'],[STAMP])
        self.assertEqual(support_metrics(cell['camera_capture_ns'],STAMP)[0],0)
        cell['camera_capture_ns'].append(STAMP+100)
        self.assertEqual(grid.snapshot(STAMP)['cells'][0]['camera_capture_ns'],[STAMP])

    def test_history_is_pruned_on_new_capture_and_out_of_order_is_ignored(self):
        grid=self.grid();point=np.array([[.01,.01,0.]])
        for offset in (0,300_000_000,600_000_000):
            grid.add(point,np.array([1]),STAMP+offset,0,camera_stamp_ns=STAMP+offset)
        history=grid.snapshot(STAMP+600_000_000)['cells'][0]['camera_capture_ns']
        self.assertEqual(support_metrics(history,STAMP+10_600_000_000)[0],100)
        grid.add(point,np.array([1]),STAMP+4_000_000_000,1,camera_stamp_ns=STAMP+4_000_000_000)
        grid.add(point,np.array([1]),STAMP+4_100_000_000,0,camera_stamp_ns=STAMP+1_000_000_000)
        cell=grid.snapshot(STAMP+4_100_000_000)['cells'][0]
        self.assertEqual(cell['camera_capture_ns'],[STAMP+4_000_000_000])
        self.assertEqual(cell['camera_observed_ns'],STAMP+4_000_000_000)

    def test_unmatched_or_unknown_labels_cannot_raise_support(self):
        grid=self.grid();point=np.array([[.01,.01,0.]])
        grid.add(point,np.array([-1]),STAMP,0,camera_stamp_ns=STAMP)
        cell=grid.snapshot(STAMP)['cells'][0]
        self.assertEqual(cell['camera_capture_ns'],[]);self.assertEqual(cell['camera_observed_ns'],0)

    def test_support_changes_leave_costs_blocked_and_unknown_values_unchanged(self):
        live,sem=inputs((5.,None,25.),frame='odom')
        old=build_cost_snapshot(live,sem)
        self.assertTrue((old['camera_support']==0).all())
        stamp=sem['cells'][0]['camera_observed_ns']
        for count,span,score in ((1,0,0),(2,300_000_000,40),(3,600_000_000,100)):
            updated=deepcopy(sem)
            updated['cells'][0]['camera_capture_ns']=[stamp+round(i*span/(count-1)) for i in range(count)] if count>1 else [stamp]
            updated['cells'][0]['camera_observed_ns']=stamp+span
            result=build_cost_snapshot(live,updated)
            self.assertEqual(int(result['camera_support'][0,0]),score)
            self.assertEqual(int(result['camera_capture_count'][0,0]),count)
            self.assertEqual(int(result['camera_capture_span_ns'][0,0]),span)
            for field in ('slope_score','semantic_score','slope_cost','terrain_cost','global_costmap','local_costmap'):
                np.testing.assert_array_equal(result[field],old[field],err_msg=field)


if __name__=='__main__':unittest.main(verbosity=2)

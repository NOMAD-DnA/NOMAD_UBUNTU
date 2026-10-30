"""User slope boundary, independent features, evidence ages and DDS handoff."""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from nomad_perception.real.planning_costs import CostPolicy,slope_scores,build_cost_snapshot,ros_messages
from nomad_perception.real.semantic_validation import SemanticGrid


STAMP=10_000_000_000


def inputs(angles=(0.,5.,10.,15.,19.99,20.,25.),cost=20,frame='world'):
    rows=[[i,0,angle,0.,1.,False,10.,STAMP] for i,angle in enumerate(angles)]
    columns=['ix','iy','slope_deg','height_m','confidence','slope_reused','last_calculated_stamp_s','last_observed_stamp_ns']
    live=dict(metadata=dict(frame_id=frame),timing=dict(updates=1),
              capture_timing=dict(sequence=1,snapshot_capture_ns=STAMP),
              modes=dict(fused=dict(resolution_m=.12,columns=columns,cells=rows)))
    semantic=dict(frame=frame,resolution_m=.12,cells=[dict(ix=i,iy=0,cost=cost,
        last_semantic_ns=STAMP,camera_observed_ns=STAMP) for i in range(len(rows))])
    return live,semantic


def snapshot(angles=(5.,),cost=20,frame='world'):
    return build_cost_snapshot(*inputs(angles,cost,frame),CostPolicy(obstacle_cost_radius_m=.01))


class CostTests(unittest.TestCase):
    def test_exact_linear_scores_and_boundary_jump(self):
        np.testing.assert_allclose(slope_scores([0,5,10,15,19.99,20,25]),
                                   [0,20,40,60,79.96,100,100],atol=1e-5)
        self.assertTrue(np.isnan(slope_scores([math.nan,math.inf,-1])).all())

    def test_wire_features_stay_separate_and_soft_rounding_never_blocks(self):
        result=snapshot()
        self.assertAlmostEqual(float(result['slope_cost'][0,0]),.2,places=6)
        self.assertAlmostEqual(float(result['terrain_cost'][0,0]),.2,places=6)
        changed=snapshot(cost=70)
        np.testing.assert_array_equal(result['slope_cost'],changed['slope_cost'])
        self.assertAlmostEqual(float(changed['terrain_cost'][0,0]),.7,places=6)
        boundary=snapshot((19.999,20.),cost=99)
        self.assertLess(boundary['global_costmap'][0,0],80)
        self.assertEqual(boundary['global_costmap'][0,1],100)
        self.assertAlmostEqual(float(boundary['slope_cost'][0,0]),.79996,places=5)
        self.assertEqual(boundary['slope_cost'][0,1],1.)

    def test_unknown_features_are_nan_but_either_confirmed_block_is_hard(self):
        live,sem=inputs((5.,25.,None),cost=None)
        result=build_cost_snapshot(live,sem,CostPolicy(obstacle_cost_radius_m=.01))
        self.assertTrue(np.isnan(result['terrain_cost']).all())
        self.assertEqual(result['global_costmap'].tolist(),[[-1,100,-1]])
        self.assertTrue(np.isnan(result['slope_cost'][0,2]))
        self.assertEqual(snapshot((None,),100)['global_costmap'][0,0],100)

    def test_different_geometry_or_frames_is_rejected_without_relabeling(self):
        for change in ('frame','resolution'):
            live,sem=inputs()
            if change=='frame':sem['frame']='odom'
            else:sem['resolution_m']=.2
            with self.assertRaises(ValueError):build_cost_snapshot(live,sem)
        self.assertEqual(snapshot()['frame'],'world')

    def test_local_expiry_and_camera_hold_decay_preserve_capture_time(self):
        live,sem=inputs((5.,5.,5.),frame='odom')
        for i,age in enumerate((0,20,40)):
            stamp=STAMP+40_000_000_000-age*1_000_000_000
            live['modes']['fused']['cells'][i][-1]=stamp
            sem['cells'][i].update(last_semantic_ns=stamp,camera_observed_ns=stamp,
                camera_capture_ns=[stamp-600_000_000,stamp-300_000_000,stamp])
        result=build_cost_snapshot(live,sem)
        self.assertEqual(result['capture_ns'],STAMP+40_000_000_000)
        self.assertEqual(result['camera_support'].tolist(),[[100,50,0]])
        self.assertEqual(result['local_costmap'][0,2],-1)
        self.assertEqual(result['slope_observed_ns'][0,2],STAMP)

    def test_inflation_does_not_fill_unknown_cells(self):
        live,sem=inputs((25.,None,5.))
        result=build_cost_snapshot(live,sem)
        self.assertEqual(result['global_costmap'][0,0],100)
        self.assertEqual(result['global_costmap'][0,1],-1)
        self.assertGreaterEqual(result['global_costmap'][0,2],80)
        self.assertNotEqual(result['global_costmap'][0,2],100)

    def test_semantic_points_keep_cell_timestamps_and_expired_dynamic_unknown(self):
        config=dict(grid_resolution_m=.12,dynamic_retention_s=1.,class_table={
            1:dict(name='ground',kind='ground',cost=20),2:dict(name='moving',kind='dynamic',cost=100)})
        grid=SemanticGrid(config)
        grid.add(np.array([[.01,0.,0.],[.13,0.,0.]]),np.array([1,1]),STAMP,0,
                 point_stamps=np.array([STAMP,STAMP+10]),camera_stamp_ns=STAMP)
        data=grid.snapshot(STAMP)
        self.assertEqual([r['last_semantic_ns'] for r in data['cells']],[STAMP,STAMP+10])
        grid.add(np.array([[.01,0.,0.]]),np.array([2]),STAMP+100,1,camera_stamp_ns=STAMP+100)
        expired=grid.snapshot(STAMP+2_000_000_000)
        self.assertIsNone(expired['cells'][0]['cost'])
        self.assertEqual(expired['cells'][0]['last_semantic_ns'],0)


try:
    from nomad_interfaces.msg import PlanningCostLayers
    HAS_INTERFACE=True
except ImportError:
    HAS_INTERFACE=False


@unittest.skipUnless(HAS_INTERFACE,'Source NOMAD_INTERFACES_SETUP for wire tests')
class WireTests(unittest.TestCase):
    def test_converted_layers_reach_real_dds_late_subscriber(self):
        import rclpy,time,uuid
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from nomad_perception.real.planning_transport import PlanningSnapshotPublisher,map_qos
        context=Context();rclpy.init(context=context,args=[])
        executor=SingleThreadedExecutor(context=context)
        prefix='/cost_output_test/run_'+uuid.uuid4().hex
        topics=('/nomad/navigation_costmap','/nomad/navigation_local_costmap',
                '/nomad/planning_cost_layers','/nomad/camera_observation_support','/nomad/perception/status')
        args=['--ros-args']
        for topic in topics:args.extend(['-r',topic+':='+prefix+topic])
        producer=Node('cost_output_test_producer',context=context,cli_args=args)
        executor.add_node(producer);consumer=None;output=None
        try:
            capture=producer.get_clock().now().nanoseconds-100_000_000
            live,sem=inputs((0.,10.,19.99,20.),frame='odom')
            for row,entry in zip(live['modes']['fused']['cells'],sem['cells']):
                row[-1]=capture;entry.update(last_semantic_ns=capture,camera_observed_ns=capture)
            converted=build_cost_snapshot(live,sem,CostPolicy(obstacle_cost_radius_m=.01))
            output=PlanningSnapshotPublisher(producer)
            self.assertTrue(output.publish(*ros_messages(converted),capture_ns=capture,navigation_validated=True))
            consumer=Node('cost_output_test_late_consumer',context=context,cli_args=args)
            executor.add_node(consumer);received=[]
            consumer.create_subscription(PlanningCostLayers,'/nomad/planning_cost_layers',received.append,map_qos())
            deadline=time.monotonic()+5
            while not received and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
            self.assertTrue(received)
            message=received[-1]
            np.testing.assert_allclose(message.slope_cost,[0.,.4,.7996,1.],atol=1e-6)
            np.testing.assert_allclose(message.terrain_cost,[.2]*4)
            self.assertEqual(message.grid.header.stamp.sec*10**9+message.grid.header.stamp.nanosec,capture)
            self.assertLess(message.grid.data[2],80)
            self.assertEqual(message.grid.data[3],100)
        finally:
            if output is not None:output.close()
            executor.shutdown()
            if consumer is not None:consumer.destroy_node()
            producer.destroy_node();context.shutdown()

    def test_converted_costs_fit_dev_publisher_and_unknown_contract(self):
        from .test_planning_transport import FakeNode
        from nomad_perception.real.planning_transport import PlanningSnapshotPublisher
        result=snapshot((5.,20.,None),frame='odom')
        messages=ros_messages(result);node=FakeNode()
        output=PlanningSnapshotPublisher(node)
        try:
            self.assertTrue(output.publish(*messages,capture_ns=STAMP,navigation_validated=True))
            layers=node.publishers['/nomad/planning_cost_layers'].messages[-1]
            np.testing.assert_allclose(layers.slope_cost[:2],[.2,1.])
            self.assertTrue(math.isnan(layers.slope_cost[2]))
            self.assertEqual(layers.grid.data[1],100)
        finally:output.close()

    def test_active_export_keeps_world_and_prepared_odom_uses_existing_transport(self):
        from .test_planning_transport import FakeNode
        from nomad_perception.real.cost_output import CostOutput
        node=FakeNode()
        with tempfile.TemporaryDirectory() as folder:
            output=CostOutput(node,folder,lambda now:inputs()[1],clock_verified=True)
            try:
                result=snapshot();output.export(result)
                msg=node.publishers['/nomad/semantic_validation/cost_layers'].messages[-1]
                self.assertEqual(msg.grid.header.frame_id,'world')
                self.assertAlmostEqual(float(output.latest['slope_score'][0,0]),20.)
                self.assertFalse(list(Path(folder).glob('**/*.npz')))
                with self.assertRaises(ValueError):output.publish_prepared(result,capture_pose_validated=True)
                aligned=snapshot(frame='odom')
                self.assertTrue(output.publish_prepared(aligned,capture_pose_validated=True))
                state=node.publishers['/nomad/perception/status'].messages[-1]
                self.assertEqual(state.state,state.READY)
                node.now_ns+=2_100_000_000;output.planning.tick()
                self.assertEqual(node.publishers['/nomad/perception/status'].messages[-1].state,state.STOPPED)
            finally:output.close()


if __name__=='__main__':unittest.main(verbosity=2)

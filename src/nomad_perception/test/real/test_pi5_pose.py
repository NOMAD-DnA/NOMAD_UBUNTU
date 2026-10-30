"""Pi 5 frame composition and freshness gates, without sensors or motor output."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from nomad_perception.real.pi5_pose import PoseConfig,raw_imu_row,Pi5PoseInput

try:
    from nomad_interfaces.msg import ModuleStatus
    HAS_INTERFACE=True
except ImportError:
    HAS_INTERFACE=False


def pose(stamp=10_000_000_000,frame='odom',child='base_link'):
    m=Odometry();m.header.frame_id=frame;m.child_frame_id=child
    m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9)
    m.pose.pose.position.x=4.;m.pose.pose.position.y=-2.;m.pose.pose.position.z=.3
    q=Rotation.from_euler('xyz',[.1,-.2,.7]).as_quat()
    for key,value in zip(('x','y','z','w'),q):setattr(m.pose.pose.orientation,key,float(value))
    return m


def fixed_tf():
    m=TransformStamped();m.header.frame_id='base_link';m.child_frame_id='oak_imu_frame'
    m.transform.translation.x=.23;m.transform.translation.y=-.12;m.transform.translation.z=.17
    q=Rotation.from_euler('xyz',[-.2,.1,-1.2]).as_quat()
    for key,value in zip(('x','y','z','w'),q):setattr(m.transform.rotation,key,float(value))
    return m


class GeometryTests(unittest.TestCase):
    def test_full_rotation_translation_and_exact_capture_stamp(self):
        from nomad_perception.real.pi5_pose import transform_matrix
        m=pose(10_123_456_789);tf=fixed_tf();fixed=transform_matrix(tf.transform)
        row=raw_imu_row(m,fixed,42.)
        base=Rotation.from_quat([getattr(m.pose.pose.orientation,k) for k in ('x','y','z','w')])
        np.testing.assert_allclose(row[1:4],np.array([4,-2,.3])+base.apply([.23,-.12,.17]))
        np.testing.assert_allclose(Rotation.from_quat(row[4:8]).as_matrix(),base.as_matrix()@fixed[:3,:3])
        self.assertEqual(row[0],10_123_456_789);self.assertEqual(row[-1],42.)
        self.assertGreater(np.linalg.norm(np.array(row[1:4])-[4,-2,.3]),.2)

    def test_bad_rotation_and_nonrigid_mount_are_rejected(self):
        m=pose();m.pose.pose.orientation.w=5.
        with self.assertRaises(ValueError):raw_imu_row(m,np.eye(4),0.)
        m=pose();bad=np.eye(4);bad[0,0]=2.
        with self.assertRaises(ValueError):raw_imu_row(m,bad,0.)


@unittest.skipUnless(HAS_INTERFACE,'Source NOMAD interfaces for Pi 5 status tests')
class InputTests(unittest.TestCase):
    def setup_input(self,lookup=None):
        from .test_planning_transport import FakeNode
        node=FakeNode();node.create_subscription=lambda *a,**kw:object()
        source=Pi5PoseInput(node,PoseConfig(),lookup_fixed=lookup or fixed_tf)
        status=ModuleStatus();status.header.frame_id='odom';status.header.stamp.sec=10
        status.state=status.READY;status.valid_for.nanosec=600_000_000
        source.on_status(status)
        return node,source,status

    def test_valid_pose_and_expired_or_waiting_status(self):
        node,source,status=self.setup_input()
        row=source.convert(pose());self.assertEqual(row[0],10_000_000_000)
        self.assertTrue(source.health()[0])
        status.state=status.WAITING;source.on_status(status)
        self.assertFalse(source.health()[0])
        with self.assertRaises(ValueError):source.convert(pose(10_000_000_001))
        status.state=status.READY;source.on_status(status)
        node.now_ns+=700_000_000
        self.assertFalse(source.health()[0])

    def test_frame_and_stale_or_reordered_poses_never_fallback(self):
        for m in (pose(frame='world'),pose(child='odom'),pose(9_000_000_000),pose(11_000_000_000)):
            _,source,_=self.setup_input()
            with self.assertRaises(ValueError):source.convert(m)
            self.assertFalse(source.health()[0])
        _,source,_=self.setup_input();source.convert(pose())
        with self.assertRaises(ValueError):source.convert(pose())

    def test_missing_dynamic_and_changed_mounts_are_rejected(self):
        def missing():raise LookupError('No static TF')
        _,source,_=self.setup_input(missing)
        with self.assertRaisesRegex(ValueError,'Missing calibrated'):source.convert(pose())
        self.assertIsNone(source.fixed)
        tf=fixed_tf();tf.header.stamp.sec=10
        _,source,_=self.setup_input(lambda:tf)
        with self.assertRaises(ValueError):source.convert(pose())
        tf.header.stamp.sec=0;source.convert(pose());tf.transform.translation.x+=.1
        with self.assertRaisesRegex(ValueError,'changed'):source.convert(pose(10_000_000_001))
        self.assertFalse(source.health()[0])

    def test_clock_rollback_latches_until_new_session(self):
        node,source,_=self.setup_input();source.convert(pose())
        node.now_ns-=1_000_000_000
        self.assertFalse(source.health()[0]);node.now_ns+=1_000_000_000
        self.assertFalse(source.health()[0])

    def test_runtime_cost_output_and_status_drop_without_a_new_map(self):
        from nomad_perception.real.cost_output import CostOutput
        from .test_planning_costs import inputs
        node,source,status=self.setup_input();source.convert(pose())
        with tempfile.TemporaryDirectory() as directory:
            terrain=Path(directory)/'terrain';terrain.mkdir()
            live,sem=inputs((10.,20.,None),frame='odom')
            (terrain/'live.json').write_text(json.dumps(live))
            output=CostOutput(node,directory,lambda now:sem,clock_verified=True,readiness=source.health)
            output.tick()
            self.assertTrue(output.canonical_published)
            layers=node.publishers['/nomad/planning_cost_layers'].messages[-1]
            self.assertEqual(layers.grid.header.frame_id,'odom')
            self.assertEqual(layers.grid.header.stamp.sec,10)
            self.assertAlmostEqual(layers.slope_cost[0],.4,places=6)
            self.assertTrue(np.isnan(layers.slope_cost[-1]))
            count=len(node.publishers['/nomad/planning_cost_layers'].messages)
            status.state=status.WAITING;source.on_status(status)
            output.planning.tick()
            self.assertEqual(node.publishers['/nomad/perception/status'].messages[-1].state,status.STOPPED)
            output.tick();self.assertFalse(output.canonical_published)
            self.assertEqual(len(node.publishers['/nomad/planning_cost_layers'].messages),count)
            output.close()


if __name__=='__main__':unittest.main(verbosity=2)

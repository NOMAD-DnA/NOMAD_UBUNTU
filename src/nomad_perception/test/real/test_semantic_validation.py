import json, tempfile, unittest, time
from pathlib import Path
import numpy as np
from nomad_perception.real.semantic_validation import load_config,transfer,warp_depth,pose_at,SemanticBridge
from nomad_perception.real.depth_filter import SORSettings

from ament_index_python.packages import get_package_share_directory
ROOT=Path(get_package_share_directory('nomad_perception'))

def settings(tmp):
    c=json.loads((ROOT/'config/real/semantic.example.json').read_text())
    c.update(confirmed_for_recording=True,image_alignment_confirmed=True,common_clock_confirmed=True,
      calibration_provenance='synthetic only',T_zed_optical_from_lidar=np.eye(4).tolist(),
      ground_class_ids=[7],classes={'7':{'name':'ground','kind':'ground','cost':0},'2':{'name':'person','kind':'dynamic','cost':100}},
      max_point_label_dt_ms=30,max_label_depth_dt_ms=30,visibility_absolute_m=.05,visibility_relative=.01,
      dynamic_retention_s=1,boundary_margin_px=0,frame_wait_wall_s=.01)
    p=Path(tmp)/'config.json';p.write_text(json.dumps(c));return load_config(p)

class DummyNode:
    def create_subscription(self,*args):return args

def image(values, stamp, encoding, frame='zed_left_camera_optical_frame'):
    from sensor_msgs.msg import Image
    from array import array
    m=Image();m.header.frame_id=frame;m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9
    values=np.asarray(values);m.height,m.width=values.shape;m.encoding=encoding;m.step=values.strides[0];m.is_bigendian=False;m.data=array('B',values.tobytes());return m

def info():
    from sensor_msgs.msg import CameraInfo
    m=CameraInfo();m.header.frame_id='zed_left_camera_optical_frame';m.height=m.width=8
    m.p=[4.,0.,4.,0.,0.,4.,4.,0.,0.,0.,1.,0.];m.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];return m

class FusionTest(unittest.TestCase):
    def test_placeholder_refuses_live(self):
        with self.assertRaises(ValueError):load_config(ROOT/'config/real/semantic.example.json')
    def test_projection_rejects_occlusion_and_boundary(self):
        with tempfile.TemporaryDirectory() as d:
            c=settings(d);labels=np.full((8,8),7,np.uint8);depth=np.full((8,8),2.,np.float32)
            K=np.array([[4.,0,4],[0,4,4],[0,0,1]])
            points=np.array([[0,0,2],[0,0,4],[99,0,2],[0,0,-1]])
            ids,stats=transfer(points,labels,depth,K,(np.eye(3),np.zeros(3)),c)
            self.assertEqual(ids.tolist(),[7,-1,-1,-1]);self.assertEqual(stats['depth_unmatched'],1)
            labels[:,4:]=2;c['boundary_margin_px']=1
            ids,_=transfer(points[:1],labels,depth,K,(np.eye(3),np.zeros(3)),c)
            self.assertEqual(ids.tolist(),[-1])
    def test_pose_interpolation_and_depth_warp(self):
        rows=[[1_000_000_000,0,0,0,0,0,0,1],[1_100_000_000,1,0,0,0,0,0,1]]
        _,pos=pose_at(rows,1_050_000_000,200_000_000);np.testing.assert_allclose(pos,[.5,0,0])
        with self.assertRaises(ValueError):pose_at(rows,1_200_000_000,200_000_000)
        depth=np.full((8,8),np.nan,np.float32);depth[4,4]=2
        K=np.array([[4.,0,4],[0,4,4],[0,0,1]])
        w=warp_depth(depth,K,(np.eye(3),np.array([.5,0,0])),(np.eye(3),np.zeros(3)))
        self.assertEqual(w[4,5],2)
    def test_bridge_both_sources_time_gate_without_storage(self):
        from nomad_perception.real.ground_slope_live import GroundSlope
        from .test_ground_slope_live import mounts
        with tempfile.TemporaryDirectory() as d:
            c=settings(d);b=SemanticBridge(DummyNode(),c,d,SORSettings(sor_neighbors=80));b.camera_info(info())
            t=1_050_000_000
            b.label(image(np.full((8,8),7,np.uint8),t,'mono8'));b.depth(image(np.full((8,8),2,np.float32),t,'32FC1'))
            rows=[[1_000_000_000,0,0,0,0,0,0,1],[1_100_000_000,0,0,0,0,0,0,1]]
            emitted=[]
            ctx=([t,0,0,0,0,0,0,1],mounts(),0.,1)
            b.submit(np.array([[0.,0.,2.],[0.,0.,4.],[.1,0,2.]]),t,0,None,ctx,np.array([t,t,t+100_000_000]))
            b.submit(np.array([[0.,0.,2.]]),t,1,np.array([[0.,0.,2.]]),ctx)
            b.process(rows,np.eye(4),lambda *a:emitted.append(a))
            self.assertEqual([len(a[0]) for a in emitted],[1,1])
            g=GroundSlope(mounts(),0.,1,camera_settings=SORSettings(),semantic_prefiltered=True)
            for pts,st,src,ctx in emitted:g.ingest(pts,st,src,None,[st,0,0,0,0,0,0,1])
            self.assertEqual(g.selected_points,[1,1]);self.assertTrue(g.metadata()['ground_segmentation'])
            self.assertEqual(b.close(t)['counts']['processed_frames'],2)
            data=b.grid.snapshot(t);self.assertTrue(data['diagnostic_only'])
            self.assertTrue(data['cells'])
            self.assertTrue(all(cell['class_id']==7 for cell in data['cells']))
            self.assertFalse(list(Path(d).rglob('*.npz')))
    def test_expiry_and_unknown_do_not_enter_slope(self):
        with tempfile.TemporaryDirectory() as d:
            c=settings(d);b=SemanticBridge(DummyNode(),c,d,SORSettings());b.camera_info(info())
            t=1_000_000_000;b.submit(np.array([[0,0,2.]]),t,0,None,None)
            out=[];b.process([],np.eye(4),lambda *a:out.append(a),flush=True)
            self.assertFalse(out);b.close(t)
            self.assertEqual(b.close(t)['counts']['no_label_in_window'],1)
    def test_ground_pipeline_populates_live_slope_worker(self):
        from nomad_perception.real.ground_slope_live import GroundSlopeWorker
        from .test_ground_slope_live import mounts
        with tempfile.TemporaryDirectory() as d:
            c=settings(d);c['save_evidence']=False
            b=SemanticBridge(DummyNode(),c,d,SORSettings())
            b.info=(80,80,np.array([[50.,0.,40.],[0.,50.,40.],[0.,0.,1.]]))
            t=1_050_000_000;label=np.full((80,80),7,np.uint8);depth=np.full((80,80),2.,np.float32)
            b.labels.append((t,label));b.depths.append((t,depth))
            x,y=np.meshgrid(np.arange(.24,.8,.02),np.arange(-.2,.22,.02));points=np.c_[x.ravel(),y.ravel(),np.full(x.size,2.)]
            context=([t,0,0,0,0,0,0,1],mounts(),0.,1)
            rows=[[1_000_000_000,0,0,0,0,0,0,1],[1_100_000_000,0,0,0,0,0,0,1]]
            w=GroundSlopeWorker(d,True,semantic_prefiltered=True)
            try:
                b.submit(points,t,0,None,context)
                b.process(rows,np.eye(4),lambda pts,ts,src,ctx:w.submit(pts,ts,src,pose=ctx[0],mounts=ctx[1],start_z=ctx[2],start_stamp_ns=ctx[3]))
                deadline=time.monotonic()+12;status={}
                while time.monotonic()<deadline:
                    status=w.status()
                    if status.get('modes',{}).get('fused',{}).get('valid_cells',0)>3:break
                    if status.get('error'):self.fail(status['error'])
                    time.sleep(.1)
                self.assertGreater(status.get('modes',{}).get('fused',{}).get('valid_cells',0),3)
                w.close()
                live=json.loads((Path(d)/'terrain/live.json').read_text())
                self.assertTrue(live['metadata']['ground_segmentation'])
                angles=[c[2] for c in live['modes']['fused']['cells'] if c[2] is not None]
                self.assertAlmostEqual(float(np.median(angles)),0.,places=3)
            finally:
                if not w.closed:w.close()
                b.close(t)

    def test_ros_delivery(self):
        import rclpy
        from sensor_msgs.msg import Image,CameraInfo
        from rclpy.qos import qos_profile_sensor_data
        with tempfile.TemporaryDirectory() as d:
            c=settings(d);rclpy.init(args=None)
            consumer=rclpy.create_node('semantic_test_receiver');producer=rclpy.create_node('semantic_test_producer')
            try:
                b=SemanticBridge(consumer,c,d,SORSettings())
                pubs=[producer.create_publisher(Image,c['topics']['labels'],qos_profile_sensor_data),producer.create_publisher(Image,c['topics']['depth'],qos_profile_sensor_data),producer.create_publisher(CameraInfo,c['topics']['camera_info'],qos_profile_sensor_data)]
                t=1_050_000_000; deadline=time.monotonic()+5
                while time.monotonic()<deadline and (not b.labels or not b.depths or b.info is None):
                    pubs[0].publish(image(np.full((8,8),7,np.uint8),t,'mono8'))
                    pubs[1].publish(image(np.full((8,8),2,np.float32),t,'32FC1'));pubs[2].publish(info())

                    for _ in range(12):rclpy.spin_once(consumer,timeout_sec=.02)
                    rclpy.spin_once(producer,timeout_sec=.01)
                self.assertTrue(b.labels and b.depths and b.info is not None, dict(b.counts))
                b.close(t)
            finally:
                consumer.destroy_node();producer.destroy_node();rclpy.shutdown()

    def test_dynamic_expiry_is_unknown_not_free(self):
        from nomad_perception.real.semantic_validation import SemanticGrid
        with tempfile.TemporaryDirectory() as d:
            c=settings(d);grid=SemanticGrid(c);p=np.array([[.1,.1,0.]])
            grid.add(p,np.array([7]),1_000_000_000,0);grid.add(p,np.array([2]),1_100_000_000,0)
            self.assertEqual(grid.snapshot(1_500_000_000)['cells'][0]['cost'],100)
            self.assertIsNone(grid.snapshot(3_000_000_000)['cells'][0]['cost'])

if __name__=='__main__':unittest.main()

import unittest
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
from nomad_perception.real.map_core import load_extrinsics,uncalibrated_extrinsics,rigid,front_mask,scan_with_times,deskew,Voxels

def driver_pair(T):
    count=720;inc=2*np.pi/count;theta=-np.pi+(np.arange(count)+.35)*inc
    p=np.c_[2*np.cos(theta),2*np.sin(theta),np.zeros(count)].astype(np.float32)
    ranges=np.zeros(count+1)
    for i in range(count):ranges[int(np.ceil((theta[i]+np.pi)/inc))]=2
    scan={'stamp_ns':10**9,'ranges':ranges.tolist(),'angle_min':-np.pi,'angle_increment':inc,'range_min':.28,'range_max':16}
    cloud={'stamp_ns':10**9,'points':p,'offsets':np.arange(count)*.00025,'received_monotonic':0,'intensities':np.arange(count)}
    return scan,cloud,scan_with_times(scan,cloud,T)

class Geometry(unittest.TestCase):
    def setUp(self):self.T,self.cal=load_extrinsics(Path(get_package_share_directory('nomad_perception'))/'config/real/oak_g2b_selected_20261030.json')
    def test_uncalibrated_baseline_only_converts_body_to_optical_axes(self):
        baseline=rigid(uncalibrated_extrinsics())
        np.testing.assert_array_equal(baseline[:3,3],np.zeros(3))
        # Forward -> optical forward, left -> optical left, up -> optical up.
        np.testing.assert_array_equal(np.eye(3)@baseline[:3,:3].T,
                                      [[0,0,1],[-1,0,0],[0,-1,0]])
        self.assertFalse(np.allclose(baseline,self.T))
    def test_exact_half_sector_uses_measured_camera_axis(self):
        theta=np.arange(3600)*2*np.pi/3600;p=np.c_[np.cos(theta),np.sin(theta),np.zeros(len(theta))]
        mask=front_mask(p,self.T);self.assertEqual(int(mask.sum()),1800)
        center=np.arctan2(self.T[2,1],self.T[2,0]);self.assertTrue(front_mask(np.array([[np.cos(center),np.sin(center),0]]),self.T)[0])
        self.assertFalse(front_mask(np.array([[-np.cos(center),-np.sin(center),0]]),self.T)[0])
        shifted=self.T.copy();shifted[:3,3]=[9,8,7];np.testing.assert_array_equal(mask,front_mask(p,shifted))
    def test_scan_bin_geometry_and_original_ray_times_survive_crop(self):
        scan,cloud,_=driver_pair(self.T)
        cloud['points']=np.roll(cloud['points'],360,axis=0)
        selected=scan_with_times(scan,cloud,self.T);self.assertEqual(len(selected['points_lidar_m']),360)
        self.assertGreater(selected['point_time_offsets_s'][0],0)
        self.assertTrue(np.all(np.diff(selected['point_time_offsets_s'])>0))
        bins=selected['bin_indices'];a=scan['angle_min']+bins*scan['angle_increment']
        np.testing.assert_allclose(selected['points_lidar_m'][:,:2],np.c_[2*np.cos(a),2*np.sin(a)])
        expected=((bins-1-360)%720)*.00025;np.testing.assert_allclose(selected['point_time_offsets_s'],expected)
    def test_stale_or_mismatched_time_stream_rejected(self):
        scan,cloud,_=driver_pair(self.T);cloud['stamp_ns']+=1
        with self.assertRaises(ValueError):scan_with_times(scan,cloud,self.T)
        cloud['stamp_ns']-=1;cloud['offsets'][3]=cloud['offsets'][2]
        with self.assertRaises(ValueError):scan_with_times(scan,cloud,self.T)
    def test_camera_b_chain_rotation_translation_and_interpolation(self):
        scan={'stamp_ns':10**9,'points_lidar_m':np.array([[1.,2,0],[2.,1,0]]),'point_time_offsets_s':np.array([0.,.1])}
        Tic=np.eye(4);Tic[:3,:3]=Rotation.from_euler('y',.3).as_matrix();Tic[:3,3]=[.03,-.02,.01]
        rows=[]
        for i in range(4):rows.append([10**9+i*50_000_000,i*.05,0,0,*Rotation.from_euler('z',i*.05).as_quat()])
        actual=deskew(scan,rows,Tic,self.T);imu=scan['points_lidar_m']@(Tic@self.T)[:3,:3].T+(Tic@self.T)[:3,3]
        expected=np.vstack([imu[0],Rotation.from_euler('z',.1).apply(imu[1])+[.1,0,0]])
        np.testing.assert_allclose(actual,expected,atol=1e-12)
        rows[0][0]+=1
        with self.assertRaises(ValueError):deskew(scan,rows,Tic,self.T)
    def test_no_pose_extrapolation_or_large_gap(self):
        scan={'stamp_ns':10**9,'points_lidar_m':np.array([[1.,0,0]]),'point_time_offsets_s':np.array([.15])}
        rows=[[10**9,0,0,0,0,0,0,1],[1_300_000_000,0,0,0,0,0,0,1]]
        with self.assertRaises(ValueError):deskew(scan,rows,np.eye(4),self.T)
        scan['point_time_offsets_s'][0]=.4
        with self.assertRaises(ValueError):deskew(scan,rows,np.eye(4),self.T)
    def test_voxel_means_contain_only_actual_samples(self):
        v=Voxels(.1);v.add(np.array([[.01,.02,.03],[.02,.04,.06],[.2,0,0]]));p,n=v.arrays()
        self.assertEqual(n.sum(),3);self.assertEqual(len(p),2);np.testing.assert_allclose(p[0],[.015,.03,.045])

if __name__=='__main__':unittest.main(verbosity=2)

import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from nomad_perception.real.map_core import Voxels
from nomad_perception.real.depth_core import sample_depth,depth_to_world,combined_arrays


class DepthGeometry(unittest.TestCase):
    def test_invalid_depth_and_sampling(self):
        p=np.zeros((4,4,3),np.float32);p[:,:,2]=2
        p[0,0]=[np.nan,0,2];p[0,2]=[0,0,0];p[2,0]=[0,0,9]
        np.testing.assert_array_equal(sample_depth(p,2),[[0,0,2]])
        p[2,2,2]=-1
        self.assertEqual(len(sample_depth(p,2)),0)

    def test_cam_a_pose_rotation_and_translation_at_acquisition_time(self):
        T=np.eye(4);T[:3,3]=[.1,-.2,.03];T[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
        rows=[[1_000_000_000,0,0,0,0,0,0,1],
              [1_100_000_000,2,0,0,*Rotation.from_euler('z',90,degrees=True).as_quat()]]
        point=np.array([[1.,0,2]])
        # At the middle exposure, local [1,0,2] -> IMU [.1,.8,2.03],
        # then world yaw 45 degrees and world translation [1,0,0].
        expected=[[1+(.1-.8)/np.sqrt(2),(.1+.8)/np.sqrt(2),2.03]]
        np.testing.assert_allclose(depth_to_world(point,1_050_000_000,rows,T),expected,atol=1e-12)
        for ns in [999_999_999,1_100_000_001]:
            with self.assertRaises(ValueError):depth_to_world(point,ns,rows,T)
        rows[-1][0]=1_300_000_000
        with self.assertRaises(ValueError):depth_to_world(point,1_050_000_000,rows,T)

    def test_overlapping_sensor_voxels_keep_source(self):
        lidar=Voxels(.03);camera=Voxels(.03)
        lidar.add(np.array([[1.,2.,3.]]));camera.add(np.array([[1.,2.,3.]]))
        p,n,s=combined_arrays(lidar,camera)
        self.assertEqual(len(p),2);np.testing.assert_array_equal(s,[0,1])

if __name__=='__main__':unittest.main(verbosity=2)

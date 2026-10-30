import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from nomad_perception.real.stereo_rectification import prepare


class Rectification(unittest.TestCase):
    def test_virtual_cameras_keep_the_same_imu_and_raw_mapping_geometry(self):
        a=np.eye(4);a[:3,:3]=Rotation.from_euler('xyz',[.02,-.01,.03]).as_matrix();a[:3,3]=[.01,.02,-.03]
        relative=np.eye(4);relative[:3,:3]=Rotation.from_euler('xyz',[.004,.012,-.006]).as_matrix();relative[:3,3]=[.075,0,0]
        b=a@relative
        raw=[a.tolist(),b.tolist()];provenance={'T_raw_imu_cameras':raw}
        K0=np.array([[450.,0,340],[0,449,249],[0,0,1.]])
        K1=np.array([[453.,0,302],[0,452,228],[0,0,1.]])
        calibration={'value0':{'T_imu_cam':[{},{}],'intrinsics':[{},{}]}}
        result,maps,details=prepare(calibration,provenance,K0,K1)
        np.testing.assert_array_equal(provenance['T_raw_imu_cameras'],raw)
        points=np.array([[.2,.1,2],[0.,0.,3.],[-.3,.2,4.]])
        in_imu=points@a[:3,:3].T+a[:3,3]
        projected=[]
        for i,T in enumerate([a,b]):
            raw_points=(in_imu-T[:3,3])@T[:3,:3]
            R=np.array(details['R_rectified_raw'][i]);rect=raw_points@R.T
            Trect=np.array(details['T_raw_imu_rectified_cameras'][i])
            np.testing.assert_allclose(rect@Trect[:3,:3].T+Trect[:3,3],in_imu,atol=1e-12)
            k=result['value0']['intrinsics'][i]['intrinsics']
            projected.append(np.c_[rect[:,0]/rect[:,2]*k['fx']+k['cx'],rect[:,1]/rect[:,2]*k['fy']+k['cy']])
            q=result['value0']['T_imu_cam'][i]
            np.testing.assert_allclose(Rotation.from_quat([q[n]for n in ['qx','qy','qz','qw']]).as_matrix(),Trect[:3,:3],atol=1e-12)
        np.testing.assert_allclose(projected[0][:,1],projected[1][:,1],atol=1e-8)
        self.assertTrue(details['raw_mapping_extrinsics_unchanged'])
        self.assertEqual(maps[0][0].shape,(480,640))

if __name__=='__main__':unittest.main(verbosity=2)

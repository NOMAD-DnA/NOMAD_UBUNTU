"""Rectify SDK-undistorted VIO images, including virtual camera extrinsics.

No second lens-undistortion step: input distortion coefficients are zero.
Raw camera/IMU transforms used by LiDAR and depth mapping stay unchanged.
"""
import copy
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from .map_core import rigid


def prepare(calibration,provenance,left_K,right_K,size=(640,480)):
    raw=[rigid(v)for v in provenance['T_raw_imu_cameras']]
    T_right_left=np.linalg.inv(raw[1])@raw[0]
    R0,R1,P0,P1,_,_,_=cv2.stereoRectify(np.array(left_K),np.zeros(5),
        np.array(right_K),np.zeros(5),size,T_right_left[:3,:3],T_right_left[:3,3],
        flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    rotations=[R0,R1];intrinsics=[P0[:3,:3],P1[:3,:3]]
    result=copy.deepcopy(calibration);transforms=[];maps=[]
    for i,(K,R,Knew)in enumerate(zip([left_K,right_K],rotations,intrinsics)):
        # R maps raw points to rectified points. Basalt needs the inverse chain:
        # p_IMU = T_IMU_raw @ T_raw_rect @ p_rect, T_raw_rect.rotation=R.T.
        inverse_rect=np.eye(4);inverse_rect[:3,:3]=R.T
        T=rigid(raw[i]@inverse_rect);transforms.append(T)
        q=Rotation.from_matrix(T[:3,:3]).as_quat()
        result['value0']['T_imu_cam'][i]=dict(zip(['px','py','pz','qx','qy','qz','qw'],map(float,[*T[:3,3],*q])))
        result['value0']['intrinsics'][i]={'camera_type':'pinhole','intrinsics':{
            'fx':float(Knew[0,0]),'fy':float(Knew[1,1]),'cx':float(Knew[0,2]),'cy':float(Knew[1,2])}}
        maps.append(cv2.initUndistortRectifyMap(np.array(K),np.zeros(5),R,Knew,size,cv2.CV_32FC1))
    details={'method':'OpenCV stereoRectify on already SDK-undistorted inputs',
        'R_rectified_raw':[r.tolist()for r in rotations],
        'intrinsics':[k.tolist()for k in intrinsics],
        'T_raw_imu_rectified_cameras':[v.tolist()for v in transforms],
        'raw_mapping_extrinsics_unchanged':True,'second_lens_undistortion':False}
    return result,maps,details


def apply(left,right,maps):
    return tuple(cv2.remap(a,*m,interpolation=cv2.INTER_LINEAR)for a,m in zip([left,right],maps))

"""Explicit CAM_B raw intrinsics for one mapping run; never flash EEPROM."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def load_model(path):
    model = json.loads(Path(path).read_text())
    if (model.get('frame_id') != 'oak_cam_b_optical_frame'
            or (model.get('width'), model.get('height')) != (640, 480)
            or model.get('distortion_model') != 'plumb_bob'):
        raise ValueError('Expected raw CAM_B 640x480 plumb_bob intrinsics')
    K = np.asarray(model['K'], float).reshape(3, 3)
    D = np.asarray(model['D'], float)
    if (D.shape not in ((4,), (5,)) or not np.isfinite(K).all()
            or not np.isfinite(D).all() or not np.allclose(K[2], [0, 0, 1])
            or not np.allclose([K[0, 1], K[1, 0]], 0)
            or K[0, 0] <= 0 or K[1, 1] <= 0):
        raise ValueError('Invalid CAM_B K/D')
    return model


def apply_model(calibration, path, dai):
    model = load_model(path)
    left, right = dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C
    if (calibration.getStereoLeftCameraId() != left
            or calibration.getStereoRightCameraId() != right):
        raise ValueError('Expected CAM_B/C stereo pair')
    K = np.asarray(model['K'], float).reshape(3, 3)
    # DepthAI stores 14 OpenCV coefficients. radtan4/5 has zero higher terms.
    D = list(model['D']) + [0.] * (14 - len(model['D']))
    calibration.setCameraIntrinsics(left, K.tolist(), 640, 480)
    calibration.setDistortionCoefficients(left, D)
    right_K = np.asarray(calibration.getCameraIntrinsics(right, 640, 480), float)
    right_D = np.asarray(calibration.getDistortionCoefficients(right), float)
    T = np.asarray(calibration.getCameraExtrinsics(left, right, False, dai.LengthUnit.METER), float)
    R0, R1, *_ = cv2.stereoRectify(K, np.array(D), right_K, right_D,
        (640, 480), T[:3, :3], T[:3, 3], flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    calibration.setStereoLeft(left, R0.tolist())
    calibration.setStereoRight(right, R1.tolist())
    if (not np.allclose(calibration.getCameraIntrinsics(left, 640, 480), K, atol=3e-5, rtol=0)
            or not np.allclose(calibration.getDistortionCoefficients(left), D, atol=1e-7, rtol=0)):
        raise ValueError('DepthAI did not retain requested K/D')
    return {'source': str(Path(path).resolve()),
        'sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        'camera_model': model, 'runtime_only': True, 'eeprom_written': False,
        'unchanged_intrinsics': ['CAM_A', 'CAM_C'],
        'camera_camera_extrinsics_source': 'device EEPROM, unchanged',
        'stereo_rectification_recomputed': True,
        'effective_cam_b_K': calibration.getCameraIntrinsics(left, 640, 480),
        'effective_cam_b_D': calibration.getDistortionCoefficients(left)}

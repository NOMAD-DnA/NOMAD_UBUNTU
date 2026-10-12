"""Exercise the actual Jazzy message containers, including CDR read-back."""
from array import array
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

_source = Path(__file__).parents[1] / 'nomad_perception' / 'semantic_input.py'
_spec = importlib.util.spec_from_file_location('semantic_input_v2_under_test', _source)
semantic = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(semantic)


def camera(frame='oak_depth_optical_frame', distortion=None):
    msg = CameraInfo()
    msg.width, msg.height = 3, 2
    msg.header.frame_id = frame
    msg.k = [2., 0., 1., 0., 4., .5, 0., 0., 1.]
    msg.r = np.eye(3, dtype=np.float64).ravel()
    msg.p = np.column_stack((np.asarray(msg.k).reshape(3,3), np.zeros(3))).ravel()
    msg.d = [] if distortion is None else distortion
    return msg


def roundtrip(msg):
    return deserialize_message(serialize_message(msg), type(msg))


@pytest.mark.parametrize('distortion', [[], [0.] * 5])
@pytest.mark.parametrize('serialized', [False, True])
def test_actual_camera_info_accepts_identity_and_zero_distortion(distortion, serialized):
    msg = camera(distortion=distortion)
    if serialized:
        msg = roundtrip(msg)
    assert isinstance(msg, CameraInfo)
    assert isinstance(msg.r, np.ndarray)
    assert isinstance(msg.d, array)
    np.testing.assert_array_equal(semantic._calibration(msg), np.asarray(msg.k))


@pytest.mark.parametrize('rectification', [np.zeros(9), np.diag([1.,1.,-1.]).ravel()])
def test_actual_camera_info_keeps_nonidentity_rectification_rejection(rectification):
    msg = camera()
    msg.r = rectification
    with pytest.raises(ValueError, match='rectification must be identity'):
        semantic._calibration(roundtrip(msg))


@pytest.mark.parametrize('distortion', [[.1, 0., 0., 0., 0.], [float('nan'), 0., 0., 0., 0.]])
def test_actual_camera_info_keeps_nonzero_or_nonfinite_distortion_rejection(distortion):
    with pytest.raises(ValueError, match='must be rectified|D must be a finite'):
        semantic._calibration(roundtrip(camera(distortion=distortion)))


def test_native_gazebo_stereo_translation_metadata_is_retained_for_raw_k_rays():
    msg = camera(distortion=[0.] * 5)
    msg.p[3] = -msg.k[0] * .0375
    saved = np.asarray(msg.p).copy()
    native = roundtrip(msg)
    np.testing.assert_array_equal(semantic._calibration(native), np.asarray(msg.k))
    np.testing.assert_array_equal(native.p, saved)


@pytest.mark.parametrize('fault', ['focal_mismatch', 'nonfinite_p', 'nonfinite_translation', 'last_row_translation', 'nonfinite_r'])
def test_actual_camera_info_rejects_incompatible_or_nonfinite_fields(fault):
    msg = camera(distortion=[0.] * 5)
    if fault=='focal_mismatch':msg.p[0]+=1.
    elif fault=='nonfinite_p':msg.p[0]=float('nan')
    elif fault=='nonfinite_translation':msg.p[3]=float('inf')
    elif fault=='last_row_translation':msg.p[11]=.1
    else:msg.r[0]=float('nan')
    with pytest.raises(ValueError,match='P must contain|intrinsic columns must agree|R must contain'):
        semantic._calibration(roundtrip(msg))


def image(pixels, encoding, frame):
    pixels = np.asarray(pixels)
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp.nanosec = 123
    msg.width, msg.height = pixels.shape[1], pixels.shape[0]
    msg.encoding, msg.is_bigendian = encoding, 0
    msg.step = len(pixels[0].tobytes())
    msg.data = pixels.tobytes()
    return roundtrip(msg)


def test_actual_images_calibration_and_cloud_cdr_roundtrip_preserve_projection():
    depth = image(np.asarray([[1000,2000,0],[3000,8000,9000]], dtype='<u2'),
                  '16UC1', 'oak_depth_optical_frame')
    labels = image(np.asarray([[31,40,0],[23,5,53]], dtype='u1'),
                   'mono8', 'oak_rgb_optical_frame')
    depth_info = roundtrip(camera(distortion=[0.] * 5))
    labels_info = roundtrip(camera('oak_rgb_optical_frame', distortion=[0.] * 5))
    xyz, classes = semantic.project_semantic_points(depth,labels,depth_info,labels_info,stride=1)
    np.testing.assert_allclose(xyz,[[-.5,-.125,1.],[0.,-.25,2.],[-1.5,.375,3.],[0.,1.,8.]])
    np.testing.assert_array_equal(classes, [31,40,23,5])
    labels_info.p[3] = -labels_info.k[0] * .0375
    original_projection = np.asarray(labels_info.p).copy()
    native_xyz,native_classes=semantic.project_semantic_points(depth,labels,depth_info,labels_info,stride=1)
    np.testing.assert_array_equal(native_xyz,xyz)
    np.testing.assert_array_equal(native_classes,classes)
    np.testing.assert_array_equal(labels_info.p,original_projection)
    cloud = roundtrip(semantic.encode_semantic_cloud(xyz,classes,depth.header))
    assert isinstance(cloud, PointCloud2)
    decoded_xyz, decoded_classes = semantic.decode_semantic_cloud(cloud)
    np.testing.assert_array_equal(decoded_xyz,xyz)
    np.testing.assert_array_equal(decoded_classes,classes)

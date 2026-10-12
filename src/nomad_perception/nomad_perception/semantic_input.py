"""Mutable initial camera XYZ + original GOOSE-64 input contract.

These helpers are independent of ROS and honor declared image/cloud strides and
byte order. No odometry or point transformation is performed by this producer.
"""
from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
import math
import time

import numpy as np


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def image_pixels(message):
    formats = {'mono8': ('u1', 1), '8UC1': ('u1', 1),
               'rgb8': ('u1', 3), 'bgr8': ('u1', 3), '8UC3': ('u1', 3),
               '16UC1': ('u2', 1), 'mono16': ('u2', 1), '32FC1': ('f4', 1)}
    if message.encoding not in formats:
        raise ValueError(f'Unsupported image encoding {message.encoding!r}')
    scalar, channels = formats[message.encoding]
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or message.is_bigendian not in (0, 1):
        raise ValueError('Invalid image dimensions or byte order')
    dtype = np.dtype(('>' if message.is_bigendian else '<') + scalar)
    pixel_step = dtype.itemsize * channels
    if step < width * pixel_step or len(message.data) != height * step:
        raise ValueError('Image data does not match dimensions and row stride')
    shape = (height, width) if channels == 1 else (height, width, channels)
    strides = (step, pixel_step) if channels == 1 else (step, pixel_step, dtype.itemsize)
    return np.ndarray(shape, dtype=dtype, buffer=bytes(message.data), strides=strides)


def depth_meters(message):
    if message.encoding not in ('16UC1', '32FC1'):
        raise ValueError('Depth must be 16UC1 millimetres or 32FC1 metres')
    result = image_pixels(message).astype(np.float32)
    if message.encoding == '16UC1':
        result *= np.float32(.001)
    return result


def semantic_class_pixels(message):
    pixels = image_pixels(message)
    if pixels.ndim == 3:
        if not np.array_equal(pixels[:, :, 0], pixels[:, :, 1]) or not np.array_equal(pixels[:, :, 0], pixels[:, :, 2]):
            raise ValueError('Semantic RGB must repeat the integer label in all channels')
        pixels = pixels[:, :, 0]
    if pixels.dtype.kind != 'u' or np.any(pixels >= 64):
        raise ValueError('Labels must be original GOOSE integer IDs 0..63')
    return pixels.astype(np.uint32)


def _calibration(info):
    k = np.asarray(info.k, dtype=np.float64)
    if k.shape != (9,) or not np.all(np.isfinite(k)):
        raise ValueError('CameraInfo K must contain nine finite values')
    if info.width <= 0 or info.height <= 0 or k[0] <= 0 or k[4] <= 0:
        raise ValueError('CameraInfo needs positive dimensions and focal lengths')
    # Raw simulation images use the pinhole K intrinsics, zero distortion and
    # identity rectification. P additionally carries stereo-origin metadata;
    # its finite Tx/Ty are retained, never applied to these raw optical rays.
    # Gazebo Sensors 8.2.2 CameraSensor.cc L741-L751 modifies only published
    # P03 in SetBaseline; L839-L855 explicitly excludes tx/ty from rendering:
    # https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/CameraSensor.cc#L741-L855
    # Gazebo Sim 8.15.0 assigns sibling CameraSensor baselines at L1100-L1127:
    # https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/sensors/Sensors.cc#L1100-L1127
    if not np.allclose(k[[1, 3, 6, 7]], 0, atol=1e-9) or not math.isclose(k[8], 1):
        raise ValueError('Only a pinhole calibration without skew is supported')
    # ROS fixed-size floating arrays are NumPy arrays; their truth value is
    # ambiguous. Convert both fixed and variable container representations
    # before deciding whether an optional calibration field is present.
    rectification = np.asarray(getattr(info, 'r', []), dtype=np.float64)
    if rectification.size and (rectification.shape != (9,) or not np.isfinite(rectification).all()):
        raise ValueError('CameraInfo R must contain nine finite values')
    if rectification.size and not np.allclose(rectification.reshape(3,3),np.eye(3),atol=1e-9):
        raise ValueError('Simulation depth/labels rectification must be identity')
    projection=np.asarray(getattr(info,'p',[]),dtype=float)
    if projection.size:
        if projection.shape!=(12,) or not np.isfinite(projection).all():
            raise ValueError('CameraInfo P must contain twelve finite values')
        if (not np.allclose(projection.reshape(3,4)[:,:3],k.reshape(3,3),rtol=1e-6,atol=1e-5)
                or not math.isclose(projection[11],0.,abs_tol=1e-9)):
            raise ValueError('Raw pinhole P intrinsic columns must agree with K and P[11] must be zero')
    distortion = np.asarray(getattr(info, 'd', []), dtype=np.float64)
    if distortion.ndim!=1 or not np.isfinite(distortion).all():
        raise ValueError('CameraInfo D must be a finite one-dimensional array')
    if distortion.size and not np.allclose(distortion, 0, atol=1e-9):
        raise ValueError('Depth and labels must be rectified before point pairing')
    return k


def project_semantic_points(depth, labels, depth_info, labels_info, *,
                            stride=2, min_depth_m=.4, max_depth_m=8.,
                            depth_frame='oak_depth_optical_frame',
                            labels_frame='oak_rgb_optical_frame', pixel_center_offset_px=0.):
    """Pair one capture's coincident labels and optical-z depth, preserving IDs."""
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError('Sampling stride must be a positive integer')
    if not np.isfinite([min_depth_m, max_depth_m]).all() or not 0 < min_depth_m < max_depth_m:
        raise ValueError('Depth range must be finite, positive and increasing')
    if stamp_ns(depth) != stamp_ns(labels):
        raise ValueError('Depth and labels must share the exact capture stamp')
    if not math.isfinite(pixel_center_offset_px):
        raise ValueError('Pixel center offset must be finite')
    # Calibration is a versioned state, not another exposure. Static stamp zero
    # and prior calibration are valid; future calibration is not silently used.
    if any(stamp_ns(info) > stamp_ns(depth) for info in (depth_info, labels_info)):
        raise ValueError('Calibration is newer than the exposure')
    for message, expected in ((depth, depth_frame), (depth_info, depth_frame),
                              (labels, labels_frame), (labels_info, labels_frame)):
        if message.header.frame_id != expected:
            raise ValueError(f'Camera frame {message.header.frame_id!r} differs from {expected!r}')
    dimensions = {(int(item.width), int(item.height)) for item in (depth, labels, depth_info, labels_info)}
    if len(dimensions) != 1:
        raise ValueError('Depth, labels and CameraInfo dimensions differ')
    k = _calibration(depth_info)
    if not np.allclose(k, _calibration(labels_info), rtol=1e-6, atol=1e-5):
        raise ValueError('Coincident depth and semantic camera intrinsics differ')
    z = depth_meters(depth)[::stride, ::stride]
    classes = semantic_class_pixels(labels)[::stride, ::stride]
    vv, uu = np.mgrid[0:int(depth.height):stride, 0:int(depth.width):stride]
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    z = z[valid]
    xyz = np.column_stack(((uu[valid] + pixel_center_offset_px - k[2]) * z / k[0],
                           (vv[valid] + pixel_center_offset_px - k[5]) * z / k[4], z)).astype(np.float32)
    return xyz, classes[valid].astype(np.uint32)


def semantic_cloud_payload(xyz, class_ids):
    xyz = np.asarray(xyz, dtype=np.float32)
    class_ids = np.asarray(class_ids)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or class_ids.shape != (len(xyz),):
        raise ValueError('XYZ must be N by 3 and classes must have N elements')
    if not np.all(np.isfinite(xyz)):
        raise ValueError('Produced XYZ must be finite')
    if class_ids.dtype.kind not in ('u', 'i') or np.any(class_ids < 0) or np.any(class_ids >= 64):
        raise ValueError('Classes must be original GOOSE integer IDs 0..63')
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('class_id', '<u4')])
    packed = np.empty(len(xyz), dtype=dtype)
    for column, name in enumerate(('x', 'y', 'z')):
        packed[name] = xyz[:, column]
    packed['class_id'] = class_ids
    return packed.tobytes()


def encode_semantic_cloud(xyz, class_ids, header):
    from sensor_msgs.msg import PointCloud2, PointField
    raw = semantic_cloud_payload(xyz, class_ids)
    result = PointCloud2()
    result.header = deepcopy(header)
    result.height, result.width = 1, len(xyz)
    result.fields = [PointField(name=name, offset=offset,
                               datatype=PointField.FLOAT32 if name != 'class_id' else PointField.UINT32,
                               count=1)
                     for name, offset in (('x', 0), ('y', 4), ('z', 8), ('class_id', 12))]
    result.is_bigendian = False
    result.point_step, result.row_step = 16, len(raw)
    result.data = raw
    result.is_dense = True
    return result


def decode_semantic_cloud(message):
    """Read organized/unorganized clouds from declared offsets and row strides."""
    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if width < 0 or height < 1 or point_step < 16 or row_step < width * point_step:
        raise ValueError('Invalid semantic cloud dimensions or strides')
    if message.is_bigendian not in (False, True, 0, 1) or len(message.data) != height * row_step:
        raise ValueError('Cloud byte order/data size is invalid')
    fields = {}
    for field in message.fields:
        if field.name in fields:
            raise ValueError(f'Duplicate cloud field {field.name}')
        fields[field.name] = field
    offsets = []
    for name in ('x', 'y', 'z', 'class_id'):
        field = fields.get(name)
        expected = 6 if name == 'class_id' else 7  # ROS PointField constants
        if field is None or field.datatype != expected or field.count != 1:
            raise ValueError(f'Cloud {name} must be scalar UINT32 or FLOAT32 as specified')
        offset = int(field.offset)
        if offset < 0 or offset + 4 > point_step or any(abs(offset - old) < 4 for old in offsets):
            raise ValueError('Semantic cloud field offset is invalid or overlaps')
        offsets.append(offset)
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z', 'class_id'],
                      'formats': [endian + 'f4'] * 3 + [endian + 'u4'],
                      'offsets': offsets, 'itemsize': point_step})
    if width == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.uint32)
    values = np.ndarray((height, width), dtype=dtype, buffer=bytes(message.data),
                        strides=(row_step, point_step))
    xyz = np.column_stack([values[name].reshape(-1) for name in ('x', 'y', 'z')]).astype(np.float32)
    classes = values['class_id'].reshape(-1).astype(np.uint32)
    if np.any(classes >= 64):
        raise ValueError('Semantic cloud contains a class outside original GOOSE 0..63')
    return xyz, classes


class ExactCaptureSynchronizer:
    """Exact exposure pairing with separately versioned pinhole calibration."""
    def __init__(self, streams=('depth', 'labels', 'depth_info', 'labels_info'),
                 max_pending=3, max_age_wall_sec=.5, on_event=None):
        if max_pending < 1 or not math.isfinite(max_age_wall_sec) or max_age_wall_sec <= 0:
            raise ValueError('Synchronizer queue and wall age must be positive')
        self.queues = {name: OrderedDict() for name in streams}
        self.calibration_streams = {name for name in streams if name.endswith('_info')}
        self.exposure_streams = [name for name in streams if name not in self.calibration_streams]
        self.max_pending, self.max_age = max_pending, max_age_wall_sec
        self.last_stamp = {name: None for name in streams}
        self.dropped = self.resets = 0
        self.on_event = on_event

    def _drop(self, stream, stamp, reason):
        self.queues[stream].pop(stamp, None)
        self.dropped += 1
        if self.on_event:
            self.on_event('synchronization_excluded', stream=stream, stamp_ns=stamp, reason=reason)

    def reset(self):
        for stream, queue in self.queues.items():
            for stamp in list(queue):
                self._drop(stream, stamp, 'clock_reset')
        self.last_stamp = {name: None for name in self.queues}
        self.resets += 1

    def expire(self, now=None, reason='buffer_wall_age_expired'):
        now = time.monotonic() if now is None else now
        for stream in self.exposure_streams:
            for stamp, (_message, received) in list(self.queues[stream].items()):
                if now - received > self.max_age:
                    self._drop(stream, stamp, reason)

    def close(self):
        for stream in self.exposure_streams:
            for stamp in list(self.queues[stream]):
                self._drop(stream, stamp, 'end_boundary_unpaired')

    @property
    def pending_exposures(self):
        return sum(len(self.queues[name]) for name in self.exposure_streams)

    def add(self, stream, message, now=None):
        now = time.monotonic() if now is None else now
        stamp = stamp_ns(message)
        previous = self.last_stamp[stream]
        if stream not in self.calibration_streams and previous is not None and stamp < previous:
            self.reset()
        self.last_stamp[stream] = stamp
        self.expire(now)
        queue = self.queues[stream]
        if stamp in queue:
            self._drop(stream, stamp, 'duplicate_stream_stamp')
        queue[stamp] = (message, now)
        while len(queue) > self.max_pending:
            if stream in self.calibration_streams:
                queue.popitem(last=False)
            else:
                self._drop(stream, next(iter(queue)), 'buffer_capacity_exceeded')
        # A calibration message can complete a waiting exposure, but its stamp
        # never replaces the image stamp. Choose oldest complete exposure.
        common = set.intersection(*(set(self.queues[name]) for name in self.exposure_streams))
        for capture_stamp in sorted(common):
            calibration = {}
            for name in self.calibration_streams:
                eligible = [key for key in self.queues[name] if key <= capture_stamp]
                if not eligible:
                    break
                calibration[name] = self.queues[name][max(eligible)][0]
            if len(calibration) != len(self.calibration_streams):
                continue
            result = {name: self.queues[name].pop(capture_stamp)[0] for name in self.exposure_streams}
            result.update(calibration)
            # Exact but late captures remain available until declared expiration;
            # synchronization completion does not reclassify skipped exposures.
            return result
        return None

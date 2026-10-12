from types import SimpleNamespace as NS
import struct

import numpy as np
import pytest

from nomad_perception.semantic_input import (
    ExactCaptureSynchronizer, decode_semantic_cloud, depth_meters,
    project_semantic_points, semantic_class_pixels, semantic_cloud_payload)


def header(stamp=123, frame='oak_depth_optical_frame'):
    return NS(stamp=NS(sec=0, nanosec=stamp), frame_id=frame)


def image(pixels, encoding, *, frame='oak_depth_optical_frame', stamp=123,
          padding=0, bigendian=False):
    pixels = np.asarray(pixels)
    height, width = pixels.shape[:2]
    active = len(pixels[0].tobytes())
    raw = b''.join(row.tobytes() + bytes([255]) * padding for row in pixels)
    return NS(header=header(stamp, frame), width=width, height=height,
              step=active + padding, is_bigendian=int(bigendian),
              encoding=encoding, data=raw)


def info(width=3, height=2, frame='oak_depth_optical_frame', stamp=123):
    return NS(header=header(stamp, frame), width=width, height=height,
              k=[2., 0., 1., 0., 4., .5, 0., 0., 1.], d=[])


def captures():
    depth = image(np.array([[1000, 2000, 0], [3000, 8000, 9000]], dtype='<u2'),
                  '16UC1', padding=4)
    labels = image(np.array([[31, 40, 0], [23, 5, 53]], dtype='u1'),
                   'mono8', frame='oak_rgb_optical_frame', padding=1)
    return depth, labels, info(), info(frame='oak_rgb_optical_frame')


def test_mm_depth_projection_and_label_pairing_with_row_padding():
    xyz, classes = project_semantic_points(*captures(), stride=1)
    assert classes.tolist() == [31, 40, 23, 5]
    assert np.allclose(xyz, [[-.5, -.125, 1], [0, -.25, 2],
                             [-1.5, .375, 3], [0, 1., 8.]])


def test_big_endian_depth_and_float_metres():
    depth = image(np.array([[1000, 2500]], dtype='>u2'), '16UC1', bigendian=True, padding=2)
    assert np.allclose(depth_meters(depth), [[1., 2.5]])
    float_depth = image(np.array([[1., 2.5]], dtype='>f4'), '32FC1', bigendian=True)
    assert np.allclose(depth_meters(float_depth), [[1., 2.5]])


def test_repeated_semantic_rgb_required_and_original_id_range():
    pixels = np.repeat(np.array([[0, 31, 63]], dtype='u1')[:, :, None], 3, axis=2)
    labels = image(pixels, 'rgb8')
    assert semantic_class_pixels(labels).tolist() == [[0, 31, 63]]
    pixels[0, 1, 1] = 30
    with pytest.raises(ValueError, match='repeat'):
        semantic_class_pixels(image(pixels, 'rgb8'))
    with pytest.raises(ValueError, match='0..63'):
        semantic_class_pixels(image(np.array([[64]], dtype='u1'), 'mono8'))


@pytest.mark.parametrize('change', ['stamp', 'frame', 'intrinsics', 'dimensions', 'distortion'])
def test_misaligned_capture_is_rejected(change):
    depth, labels, depth_info, labels_info = captures()
    if change == 'stamp':
        labels.header.stamp.nanosec += 1
    elif change == 'frame':
        labels.header.frame_id = 'oak_left_optical_frame'
    elif change == 'intrinsics':
        labels_info.k[0] += 1
    elif change == 'dimensions':
        labels_info.width += 1
    else:
        labels_info.d = [.1]
    with pytest.raises(ValueError):
        project_semantic_points(depth, labels, depth_info, labels_info, stride=1)


def test_cloud_payload_does_not_change_class_ids_or_units():
    xyz = np.array([[1., 2., 3.], [-1., 0., .4]], dtype=np.float32)
    raw = semantic_cloud_payload(xyz, np.array([31, 40], dtype=np.uint32))
    assert len(raw) == 32
    assert struct.unpack('<fffI', raw[:16]) == (1., 2., 3., 31)
    with pytest.raises(ValueError):
        semantic_cloud_payload(xyz, np.array([31., 40.]))


def cloud(bigendian):
    endian = '>' if bigendian else '<'
    values = [(1., 2., 3., 31), (4., 5., 6., 40),
              (7., 8., 9., 23), (-1., -2., .5, 5)]
    raw = b''
    for row in range(2):
        for x, y, z, cls in values[row * 2: row * 2 + 2]:
            raw += struct.pack(endian + 'Ifff', cls, z, x, y) + b'pad!pad!'
        raw += b'ROWPAD!!'
    return NS(width=2, height=2, point_step=24, row_step=56,
              is_bigendian=bigendian, data=raw, fields=[
                  NS(name='class_id', offset=0, datatype=6, count=1),
                  NS(name='z', offset=4, datatype=7, count=1),
                  NS(name='x', offset=8, datatype=7, count=1),
                  NS(name='y', offset=12, datatype=7, count=1)])


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_cloud_uses_declared_offsets_and_strides(bigendian):
    xyz, classes = decode_semantic_cloud(cloud(bigendian))
    assert np.allclose(xyz, [[1, 2, 3], [4, 5, 6], [7, 8, 9], [-1, -2, .5]])
    assert classes.tolist() == [31, 40, 23, 5]


@pytest.mark.parametrize('change', ['missing_class', 'wrong_type', 'overlap', 'short_bytes'])
def test_invalid_cloud_contract_is_rejected(change):
    message = cloud(False)
    if change == 'missing_class':
        message.fields.pop(0)
    elif change == 'wrong_type':
        message.fields[0].datatype = 7
    elif change == 'overlap':
        message.fields[0].offset = 9
    else:
        message.data = message.data[:-1]
    with pytest.raises(ValueError):
        decode_semantic_cloud(message)


def test_exact_sync_never_uses_nearby_label_and_remains_bounded():
    sync = ExactCaptureSynchronizer(streams=('depth', 'labels'), max_pending=3)
    for stamp in range(6):
        assert sync.add('depth', NS(header=header(stamp)), now=0.) is None
    assert len(sync.queues['depth']) == 3
    assert sync.add('labels', NS(header=header(6)), now=0.) is None
    capture = sync.add('labels', NS(header=header(5)), now=0.)
    # Backward stream stamp starts a new clock epoch, discarding old pairs.
    assert capture is None
    assert sync.resets == 1
    capture = sync.add('depth', NS(header=header(5)), now=.1)
    assert capture is not None
    assert all(item.header.stamp.nanosec == 5 for item in capture.values())


def test_expired_pair_is_never_used():
    sync = ExactCaptureSynchronizer(streams=('depth', 'labels'), max_age_wall_sec=.5)
    sync.add('depth', NS(header=header()), now=0.)
    assert sync.add('labels', NS(header=header()), now=.6) is None
    assert sync.dropped == 1

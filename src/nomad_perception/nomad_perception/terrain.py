"""RGB-D terrain preferences; no authored world, map or obstacle truth inputs.

Labels: 0 uncertain, 1 dry dirt, 2 grass/field. This is a simulator colour
baseline, not a learned semantic model or a proof of terrain traversability.
"""
import cv2
import numpy as np


def image_array(msg):
    """Decode supported ROS images respecting row padding and byte order."""
    formats = {'bgr8': ('u1', 3), 'rgb8': ('u1', 3),
               '16UC1': ('u2', 1), '32FC1': ('f4', 1)}
    if msg.encoding not in formats:
        raise ValueError('Unsupported image encoding')
    kind, channels = formats[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + kind)
    row_bytes = msg.width * channels * dtype.itemsize
    if (msg.width <= 0 or msg.height <= 0 or msg.step < row_bytes
            or len(msg.data) < msg.step * msg.height):
        raise ValueError('Invalid image dimensions/stride/data')
    shape = (msg.height, msg.width, channels) if channels == 3 else (msg.height, msg.width)
    strides = ((msg.step, channels * dtype.itemsize, dtype.itemsize) if channels == 3
               else (msg.step, dtype.itemsize))
    result = np.ndarray(shape, dtype=dtype, buffer=bytes(msg.data), strides=strides).copy()
    if msg.encoding == 'rgb8':
        result = result[..., ::-1].copy()
    if channels == 1:
        result = result.astype(np.float32)
        if msg.encoding == '16UC1':
            result *= 0.001
    return result


def classify(bgr, dirt_hue=(5, 20), field_hue=(22, 95), min_saturation=30,
             min_value=30):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    valid = (s >= min_saturation) & (v >= min_value)
    labels = np.zeros(h.shape, dtype=np.uint8)
    labels[valid & (h >= dirt_hue[0]) & (h <= dirt_hue[1])] = 1
    labels[valid & (h >= field_hue[0]) & (h <= field_hue[1])] = 2
    return labels


def project_ground(depth, labels, k, rotation, translation, stride=4,
                   max_range=6.0, max_slope=0.52):
    """Project aligned optical depth through stamped TF; reject steep surfaces.

    Local depth normals gate colour preferences (not obstacle clearing). Depth
    discontinuities are rejected before fitting a normal. No interpolation of
    missing depth, camera rays, or road colour into unseen ground is performed.
    """
    k = np.asarray(k).reshape(3, 3)
    if (depth.shape != labels.shape or not np.all(np.isfinite(k))
            or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('Invalid aligned depth/calibration')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    points = np.stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z), axis=-1)
    world = points @ rotation.T + translation
    good = np.isfinite(z) & (z >= 0.4) & (z <= max_range)
    # Interior central differences, with invalid depth neighbours excluded.
    dx = world[1:-1, 2:] - world[1:-1, :-2]
    dy = world[2:, 1:-1] - world[:-2, 1:-1]
    normal = np.cross(dx, dy)
    norm = np.linalg.norm(normal, axis=-1)
    keep = (good[1:-1, 1:-1] & good[1:-1, 2:] & good[1:-1, :-2]
            & good[2:, 1:-1] & good[:-2, 1:-1] & (norm > 1e-8))
    keep &= np.abs(normal[..., 2]) >= np.cos(max_slope)*norm
    keep &= (np.linalg.norm(dx, axis=-1) < 0.5) & (np.linalg.norm(dy, axis=-1) < 0.5)
    # A horizontal surface above the camera cannot be supporting ground.
    keep &= world[1:-1, 1:-1, 2] <= translation[2] + 0.1
    sampled = labels[::stride, ::stride][1:-1, 1:-1]
    return world[1:-1, 1:-1][keep], sampled[keep]


def fuse_costs(global_map, local_map, terrain, kernel, neutral_cost=35):
    """Geometry can add hazards, but can never clear LiDAR/unknown space."""
    costs = np.where(terrain < 0, neutral_cost, terrain).astype(np.uint8)
    # Apply surface cost/hazards across the same clearance footprint as LiDAR.
    footprint = cv2.dilate(costs, kernel).astype(np.int8)
    return tuple(np.where(footprint >= 80, 100,
                         np.where(layer == 0, footprint, layer)).astype(np.int8)
                 for layer in (global_map, local_map))

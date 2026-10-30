"""Depth observations in raw CAM_A optical meters, mapped with the same VIO."""
import numpy as np
from .map_core import deskew, rigid

DEPTH_FRAME = 'oak_cam_a_optical_frame'
SOURCE_COLORS = np.array([[108, 206, 240], [255, 143, 112]], dtype=np.uint8)


def sample_depth(points, stride=4, min_z=.3, max_z=8.):
    """Subsample the organized image before filtering, preserving actual XYZ.

    Limits refer to CAM_A optical Z, not spherical distance. Zero, negative,
    nonfinite and out-of-range depth never become world/map observations.
    """
    if stride < 1 or not 0 < min_z < max_z:
        raise ValueError('Invalid depth sampling settings')
    p = np.asarray(points, dtype=np.float32)
    if p.ndim != 3 or p.shape[2] != 3:
        raise ValueError('Expected an organized HxWx3 depth point cloud')
    p = p[::stride, ::stride].reshape(-1, 3)
    good = np.isfinite(p).all(axis=1) & (p[:, 2] >= min_z) & (p[:, 2] <= max_z)
    return np.ascontiguousarray(p[good])


def depth_to_world(points, stamp_ns, rows, T_imu_cam_a):
    # One acquisition timestamp per stereo depth frame. Interpolate the pose,
    # never extrapolate and never use USB arrival time as acquisition time.
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('Invalid depth cloud')
    return deskew({'stamp_ns': int(stamp_ns), 'points_lidar_m': p,
                   'point_time_offsets_s': np.zeros(len(p))},
                  rows, rigid(T_imu_cam_a), np.eye(4))


def combined_arrays(lidar, camera):
    """Keep both source observations even when they occupy the same voxel."""
    lp, ln = lidar.arrays(); cp, cn = camera.arrays()
    return (np.concatenate([lp, cp]), np.concatenate([ln, cn]),
            np.concatenate([np.zeros(len(lp), np.uint8), np.ones(len(cp), np.uint8)]))

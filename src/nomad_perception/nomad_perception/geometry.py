"""Sensor geometry independent of ROS and of simulator world files."""
import math
import numpy as np


def rotation(q):
    values = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.all(np.isfinite(values)) or norm < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = values / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def tilt(q):
    return math.acos(float(np.clip(rotation(q)[2, 2], -1.0, 1.0)))


def project_scan(scan, transform, usable_range, passable=None, include_height=False):
    """Project full 3D, timestamped scan rays onto the planning XY plane.

    Hits remain obstacles unless the caller confirms a matching observed ground
    or grass surface. A filtered ray ends at the measured point, never beyond it.
    Positive infinity supplies free evidence up to the configured range only.
    """
    if (not scan.ranges or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0
            or not 0 <= scan.range_min < scan.range_max
            or not math.isfinite(scan.range_max)):
        raise ValueError('Invalid LaserScan metadata')
    r = rotation(transform.rotation)
    t = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    if not np.all(np.isfinite(t)):
        raise ValueError('Invalid sensor translation')
    limit = min(usable_range, scan.range_max)
    rays, endpoints = [], []
    for i, distance in enumerate(scan.ranges):
        if math.isnan(distance) or distance < scan.range_min:
            continue
        angle = scan.angle_min + i*scan.angle_increment
        length = min(distance, limit)
        endpoint = t + r @ np.array([length*math.cos(angle), length*math.sin(angle), 0.0])
        rays.append((float(endpoint[0]), float(endpoint[1]), distance < limit))
        endpoints.append(endpoint)
    if passable is not None and endpoints:
        soft = passable(np.asarray(endpoints))
        rays = [(x, y, hit and not bool(surface)) for (x, y, hit), surface in zip(rays, soft)]
    if include_height:
        rays = [(*ray, float(point[2])) for ray, point in zip(rays, endpoints)]
    return (float(t[0]), float(t[1])), rays

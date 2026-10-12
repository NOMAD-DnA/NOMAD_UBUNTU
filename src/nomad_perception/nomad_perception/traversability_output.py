"""Preserve numeric terrain scores through the existing Planning output policies.

The core stores one accumulated map. These are two time/clearance views of it.
LiDAR ray-free evidence remains the existing simulation's separate clearance
evidence; it never supplies S/G or clears a core blocked cell.
"""
import math
import cv2
import numpy as np
from .observed_grid import ObservedGrid


def transform_points(xyz, transform):
    """Apply a full rigid transform once; translation and XYZ are meters."""
    xyz = np.asarray(xyz, dtype=np.float64)
    q = transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    t = transform.translation
    translation = np.array([t.x, t.y, t.z], dtype=np.float64)
    if not np.isfinite(quaternion).all() or not np.isfinite(translation).all() or norm < 1e-12:
        raise ValueError('Invalid TF quaternion/translation')
    x, y, z, w = quaternion / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return (xyz @ rotation.T + translation).astype(np.float32)


class CostViews:
    def __init__(self, resolution=.2, initial_size=80., max_size=160., inflation=.7,
                 camera_ttl=30., local_max_age=2., scan_free_cost=5):
        if (not all(math.isfinite(v) and v > 0 for v in
                    (resolution, initial_size, max_size, camera_ttl, local_max_age))
                or max_size < initial_size or not math.isfinite(inflation) or inflation < 0):
            raise ValueError('Invalid grid/view settings')
        self.grid = ObservedGrid(resolution, initial_size, inflation, max_size)
        if not isinstance(scan_free_cost, int) or not 5 <= scan_free_cost <= 99:
            raise ValueError('Ordinary scan-only output score must be 5..99')
        self.camera_ttl, self.local_max_age = camera_ttl, local_max_age
        self.scan_free_cost = scan_free_cost

    def accept_points(self, xyz):
        """Keep the sparse core inside the configured bounded output map."""
        xyz = np.asarray(xyz)
        if not len(xyz):
            return np.zeros(0, dtype=bool)
        # Grow in existing five-meter blocks, without moving existing cells.
        for p in (np.min(xyz[:, :2], axis=0), np.max(xyz[:, :2], axis=0)):
            self.grid.ensure_bounds(float(p[0]), float(p[1]), margin=0.)
        cx = np.floor((xyz[:, 0]-self.grid.origin_x)/self.grid.resolution)
        cy = np.floor((xyz[:, 1]-self.grid.origin_y)/self.grid.resolution)
        return (cx >= 0) & (cx < self.grid.width) & (cy >= 0) & (cy < self.grid.height)

    def render(self, estimates, now):
        grid = self.grid
        raw = np.full((grid.height, grid.width), -1, dtype=np.int8)
        support_stamps = np.full(raw.shape, -np.inf)
        for (ix, iy), estimate in estimates.items():
            cx, cy = grid.cell((ix+.5)*grid.resolution, (iy+.5)*grid.resolution)
            if 0 <= cx < grid.width and 0 <= cy < grid.height:
                raw[cy, cx] = estimate.final_cost
                support_stamps[cy, cx] = estimate.cost_stamp_s
        age = now-support_stamps
        camera = np.where((age >= 0) & (age <= self.camera_ttl), raw, -1).astype(np.int8)
        blocked = cv2.dilate((camera == 100).astype(np.uint8), grid.kernel) != 0
        scan_age = now-grid.seen
        scan_free = (grid.log_odds < 0) & (scan_age >= 0)
        camera_free = (camera >= 0) & (camera < 100)
        global_map = np.full(raw.shape, -1, dtype=np.int8)
        global_map[scan_free] = self.scan_free_cost  # Same free-ray policy, agreed ordinary floor.
        global_map[camera_free] = camera[camera_free]
        global_map[blocked] = 100
        recent = ((scan_free & (scan_age <= self.local_max_age)) |
                  (camera_free & (age <= self.local_max_age))).astype(np.uint8)
        safe = cv2.erode(recent, grid.kernel, borderType=cv2.BORDER_CONSTANT,
                         borderValue=0) != 0
        local_map = np.full(raw.shape, -1, dtype=np.int8)
        local_map[safe & scan_free] = self.scan_free_cost
        local_map[safe & camera_free] = camera[safe & camera_free]
        local_map[blocked] = 100
        return raw, global_map, local_map

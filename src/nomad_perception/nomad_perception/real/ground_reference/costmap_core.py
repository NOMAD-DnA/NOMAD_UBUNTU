"""LiDAR XYZ adapter around the preserved manual mapping TLS and cost policy.

No camera, semantic labels, world geometry or ray-free evidence is consumed.
The old geometry-only core expects an eligible class token: constant 7 is an
internal admission token for every measured XYZ, NOT a predicted ground label.
"""
from dataclasses import asdict
import math

import numpy as np

from ._geometry.traversability_core import CoreConfig
from ._geometry.quality_mapping_core import QualityMappingCore, PlaneQualityConfig
from ._geometry.cost_policy import GeometryCostPolicy


class LidarSlopeMap:
    def __init__(self, config=None, quality=None, policy=None, max_size_m=160.):
        self.config = config or CoreConfig(geometry_only=True, resolution_m=.1, radius_m=.15)
        self.quality = quality or PlaneQualityConfig(refit_radii_m=(self.config.radius_m,))
        self.policy = policy or GeometryCostPolicy(limit_deg=self.config.slope_limit_deg)
        if not (self.config.geometry_only and self.config.slope_input == 'raw_points'
                and self.config.plane_method == 'tls'):
            raise ValueError('LiDAR cost mapping requires geometry-only raw-point TLS')
        if not math.isfinite(max_size_m) or max_size_m <= 0:
            raise ValueError('max_size_m must be finite and positive')
        self.core = QualityMappingCore(self.config, self.quality)
        self.half_cells = max(1, math.ceil(max_size_m / self.config.resolution_m / 2))
        self.last_stamp = -math.inf
        self.frames = self.points = self.out_of_bounds = 0
        self.estimates = {}

    def ingest(self, points, stamp):
        points = np.asarray(points, dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
                or not math.isfinite(stamp)):
            raise ValueError('Expected finite Nx3 points and timestamp')
        if stamp <= self.last_stamp:
            raise ValueError('Repeated or out-of-order LiDAR observation')
        bound = self.half_cells * self.config.resolution_m
        keep = np.all((points[:, :2] >= -bound) & (points[:, :2] < bound), axis=1)
        self.out_of_bounds += int(np.count_nonzero(~keep))
        points = points[keep]
        # This does not perform semantic segmentation or add a semantic score.
        self.core.ingest(points, np.full(len(points), 7, dtype=np.uint8), stamp)
        self.last_stamp = stamp
        self.frames += 1
        self.points += len(points)

    def snapshot(self):
        if math.isfinite(self.last_stamp):
            self.estimates = self.core.snapshot(self.last_stamp)
        keys = np.array(list(self.estimates), dtype=np.int64).reshape(-1, 2)
        if len(keys):
            lo = np.maximum(keys.min(axis=0)-1, -self.half_cells)
            hi = np.minimum(keys.max(axis=0)+2, self.half_cells)
        else:
            lo, hi = np.array([-1, -1]), np.array([1, 1])
        shape = tuple((hi-lo)[::-1])
        cost = np.full(shape, -1, dtype=np.int8)
        slope = np.full(shape, np.nan, dtype=np.float32)
        height = np.full(shape, np.nan, dtype=np.float32)
        confidence = np.zeros(shape, dtype=np.float32)
        observed = np.zeros(shape, dtype=bool)
        if len(keys):
            rows, columns = (keys-lo).T[::-1]
            items = list(self.estimates.values())
            angles = np.array([np.nan if p.slope_deg is None else p.slope_deg for p in items])
            raw_q = np.array([p.q_g for p in items])
            effective, _ = self.policy.correct_confidence(keys, angles, raw_q)
            # Preserve the existing angle-power costs and 20 degree blocking.
            base = np.array([100 if p.geometry_cost is None or p.geometry_cost >= 100
                             else 0 for p in items], dtype=np.int8)
            converted = self.policy.apply(base, angles, effective)
            # A missing fit is UNKNOWN, rather than confirmed steep terrain.
            valid = np.array([p.slope_valid for p in items]) & np.isfinite(angles)
            converted[~valid] = -1
            cost[rows, columns] = converted
            slope[rows, columns] = angles
            confidence[rows, columns] = effective
            height[rows, columns] = [np.nan if p.height is None else p.height for p in items]
            observed[rows, columns] = True
        return dict(cost=cost, slope_deg=slope, height_m=height, confidence=confidence,
                    observed=observed, origin_xy=lo*self.config.resolution_m,
                    resolution_m=np.array(self.config.resolution_m))

    def metadata(self):
        return dict(schema='nomad-lidar-costmap-v1',
                    input_provider='timestamp-aligned measured LiDAR XYZ per scan',
                    semantic_input=False, ground_segmentation=False, world_geometry_input=False,
                    eligibility_token='constant 7 admits XYZ to the legacy geometry-only API; no class prediction',
                    candidate_settings=asdict(self.config), plane_quality=self.core.quality_metadata(),
                    cost_policy=self.policy.metadata(), unknown_cost=-1, blocked_cost=100,
                    uncertain_output_cost=-1, raw_window_s=self.config.raw_window_s,
                    persistent_estimates=True, received_frames=self.frames, input_points=self.points,
                    out_of_bounds_points=self.out_of_bounds, last_stamp_s=(self.last_stamp
                    if math.isfinite(self.last_stamp) else None), core_counters=dict(self.core.diagnostics))


def cost_rgb(cost):
    """Shared exact palette for the RViz image and saved PNG (rows stay XY)."""
    cost = np.asarray(cost)
    if cost.ndim != 2 or np.any((cost < -1) | (cost > 100)):
        raise ValueError('cost must be a 2D -1..100 grid')
    values = np.clip(cost, 0, 99)
    palette = np.column_stack([np.interp(np.arange(100), [0, 33, 66, 99], colors)
                               for colors in ((245, 86, 255, 232), (245, 196, 228, 72),
                                              (245, 184, 94, 53))]).astype(np.uint8)
    rgb = palette[values]
    rgb[cost < 0] = (85, 90, 100)
    rgb[cost == 100] = (10, 10, 10)
    return rgb

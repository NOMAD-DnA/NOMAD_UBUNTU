"""Accumulated raw-point ground geometry and temporal semantic votes."""

from collections import deque
import numpy as np
from .traversability_core import CoreConfig
from .quality_mapping_core import QualityMappingCore, PlaneQualityConfig
from .traversability_output import CostViews


class Layers:
    def __init__(
        self, resolution=0.2, size=80.0, max_size=160.0, ground_ids=(8,), min_points=10
    ):
        self.views = CostViews(resolution, size, max_size)
        self.core = QualityMappingCore(
            CoreConfig(
                geometry_only=True,
                resolution_m=resolution,
                radius_m=0.3,
                slope_input="raw_points",
                plane_method="tls",
                per_cell_max_points=128,
                global_max_points=100000,
            ),
            PlaneQualityConfig(),
            allow_height_refit=False,
        )
        self.ground_ids = ground_ids
        self.min_points = min_points
        self.votes = {}
        self.records = {}
        self.frames = deque()
        self.last_stamp = -np.inf

    def ingest(self, xyz, classes, stamp):
        if stamp <= self.last_stamp:
            raise ValueError("Non-monotonic capture; restart map after clock reset")
        self.last_stamp = stamp
        valid = self.views.accept_points(xyz)
        xyz = xyz[valid]
        classes = classes[valid]
        keys = np.floor(xyz[:, :2] / self.views.grid.resolution).astype(np.int64)
        unique, counts = np.unique(
            np.column_stack([keys, classes]), axis=0, return_counts=True
        )
        contribution = {}
        for (x, y, c), count in zip(unique, counts):
            k = (int(x), int(y))
            v = self.votes.setdefault(k, np.zeros(10, dtype=np.int64))
            v[c] += count
            contribution[(k, int(c))] = int(count)
        self.frames.append((stamp, contribution))
        while self.frames and self.frames[0][0] < stamp - 10:
            _, old = self.frames.popleft()
            for (k, c), count in old.items():
                self.votes[k][c] -= count
                if not self.votes[k].any():
                    del self.votes[k]
        for k in contribution:
            key = k[0]
            v = self.votes[key]
            if v.sum() >= self.min_points:
                self.records[key] = int(v.argmax())
        ground = np.isin(classes, self.ground_ids)
        self.core.ingest(
            xyz[ground], np.full(int(ground.sum()), 31, dtype=np.uint32), stamp
        )
        return self.core.snapshot(stamp)

    def arrays(self, estimates):
        g = self.views.grid
        shape = (g.height, g.width)
        height = np.full(shape, np.nan, dtype=np.float32)
        slope = height.copy()
        classes = np.full(shape, -1, dtype=np.int16)

        def indices(keys):
            xy = (
                np.asarray(keys, dtype=np.float64).reshape(-1, 2) + 0.5
            ) * g.resolution
            ij = np.floor((xy - [g.origin_x, g.origin_y]) / g.resolution).astype(
                np.int64
            )
            valid = (
                (ij[:, 0] >= 0)
                & (ij[:, 0] < g.width)
                & (ij[:, 1] >= 0)
                & (ij[:, 1] < g.height)
            )
            return ij, valid

        if self.records:
            ij, valid = indices(list(self.records))
            classes[ij[valid, 1], ij[valid, 0]] = np.asarray(
                list(self.records.values()), dtype=np.int16
            )[valid]
        if estimates:
            ij, valid = indices(list(estimates))
            entries = list(estimates.values())
            h = np.asarray(
                [
                    e.height if e.height_valid and e.height is not None else np.nan
                    for e in entries
                ],
                dtype=np.float32,
            )
            s = np.asarray(
                [
                    e.slope_deg if e.slope_valid and e.slope_deg is not None else np.nan
                    for e in entries
                ],
                dtype=np.float32,
            )
            height[ij[valid, 1], ij[valid, 0]] = h[valid]
            slope[ij[valid, 1], ij[valid, 0]] = s[valid]
        return height, slope, classes

"""Observed, connected ground surfaces for classifying 3D laser endpoints.

No authored map is used. Missing cells break connectivity; a disconnected flat
rock top is not ground. A fresh depth stream may reuse recent verified surfaces.
"""
from collections import deque
import math
import numpy as np


class GroundEvidence:
    def __init__(self, resolution=.2, max_age=.3, height_tolerance=.05,
                 slope_limit=.35, roughness_limit=.025, slope_enabled=True,
                 cell_local=False, memory_seconds=0.):
        self.resolution = resolution
        self.max_age = max_age
        self.height_tolerance = height_tolerance
        self.slope_limit = slope_limit
        self.slope_enabled = slope_enabled
        self.roughness_limit = roughness_limit
        self.cell_local = cell_local
        self.memory_seconds = memory_seconds
        self.history = {}
        self.rejected_cells = set()
        self.cells = {}
        self.stamp = -math.inf

    def clear(self):
        self.cells = {}
        self.history = {}
        self.rejected_cells = set()
        self.stamp = -math.inf

    def update(self, points, base_rotation, base_translation, stamp):
        if stamp < self.stamp:
            self.clear()
        self.cells = {}
        self.history = {key: record for key, record in self.history.items()
                        if 0 <= stamp-record[1] <= self.memory_seconds}
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) == 0:
            self.clear()
            return 0
        keys, inverse = np.unique(np.floor(points[:, :2]/self.resolution).astype(int),
                                  axis=0, return_inverse=True)
        groups = np.split(np.argsort(inverse, kind='stable'), np.cumsum(np.bincount(inverse))[:-1])
        lookup = {tuple(k): g for k, g in zip(keys, groups)}
        previous = self.history.copy()
        # A new observation supersedes old evidence, including rejected fits
        # (e.g. a rock/trunk now occupies a previously visible ground cell).
        for key in lookup:
            self.history.pop(key, None)
        surfaces, candidates = {}, []
        for key, group in lookup.items():
            if len(group) < 3:
                continue
            indices = np.concatenate([lookup[k] for k in
                [(key[0]+dx, key[1]+dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)] if k in lookup])
            centre_xy = (np.asarray(key)+.5)*self.resolution
            patch = points[indices]
            patch = patch[np.linalg.norm(patch[:, :2]-centre_xy, axis=1) <= .30]
            if self.cell_local:
                # Fit the cell itself: nearby trunks/rocks must not distort the
                # ground plane or turn neighbouring ground into a rough hazard.
                own_patch = points[group]
                own_xy = own_patch[:, :2]-own_patch[:, :2].mean(axis=0)
                if (len(own_patch) >= 6
                        and np.linalg.eigvalsh(own_xy.T @ own_xy/len(own_xy))[0] >= .0004):
                    patch = own_patch
            if len(patch) < 6:
                continue
            centre = patch.mean(axis=0)
            xy = patch[:, :2]-centre[:2]
            if np.linalg.eigvalsh(xy.T @ xy/len(xy))[0] < .0004:
                continue
            gradient = np.linalg.lstsq(xy, patch[:, 2]-centre[2], rcond=None)[0]
            residual = patch[:, 2]-centre[2]-xy @ gradient
            if ((self.slope_enabled and math.atan(np.linalg.norm(gradient)) >= self.slope_limit)
                    or np.sqrt(np.mean(residual**2)) >= self.roughness_limit
                    or np.max(np.abs(residual)) >= self.height_tolerance):
                continue
            # Also reject a tall point in the actual cell outside the fitting disc.
            own = points[group]
            if np.max(np.abs(own[:, 2]-centre[2]-(own[:, :2]-centre[:2]) @ gradient)) >= self.height_tolerance:
                continue
            z = centre[2]+(centre_xy-centre[:2]) @ gradient
            surfaces[key] = (centre_xy, float(z), gradient,
                             own[:, :2].min(axis=0), own[:, :2].max(axis=0))
            # The base_link origin is the ground-contact reference in NOMAD.
            # Seed directly observed near ground within the contact envelope.
            # Do not extrapolate a distant tangent plane back to the chassis:
            # a curved depression can have a tilted vehicle and level ground ahead.
            local = (np.r_[centre_xy, z]-base_translation) @ base_rotation
            if (.35 <= local[0] <= 1.5 and abs(local[1]) <= .6
                    and abs(local[2]) <= .15):
                candidates.append((key, local))
        # Start from the nearest low surface band, not every nearby flat top.
        seeds = []
        if candidates:
            nearest = min(local[0] for _, local in candidates)
            band = [(key, local) for key, local in candidates if local[0] <= nearest+.25]
            lowest = min(local[2] for _, local in band)
            seeds = [key for key, local in band if local[2] <= lowest+.05]
        connected = set(seeds)
        for key, surface in surfaces.items():
            old = previous.get(key)
            if old is None:
                continue
            xy, z, gradient, lo, hi = surface
            old_xy, old_z, old_gradient, old_lo, old_hi = old[0]
            # Reattach directly observed ground to its previously confirmed
            # surface, never to a new disconnected elevated rock top.
            corners = np.array([lo, hi, [lo[0], hi[1]], [hi[0], lo[1]]])
            overlap = np.all(np.minimum(hi, old_hi) >= np.maximum(lo, old_lo))
            delta = z+(corners-xy) @ gradient-old_z-(corners-old_xy) @ old_gradient
            if overlap and np.max(np.abs(delta)) <= .04:
                connected.add(key)
        queue = deque(connected)
        while queue:
            key = queue.popleft()
            xy, z, gradient, _, _ = surfaces[key]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbour = (key[0]+dx, key[1]+dy)
                if neighbour in connected or neighbour not in surfaces:
                    continue
                nxy, nz, ng, _, _ = surfaces[neighbour]
                delta = nxy-xy
                # Both fitted surfaces must meet; no height jump over a step.
                if (abs(nz-z-delta @ gradient) > .04
                        or abs(z-nz+delta @ ng) > .04):
                    continue
                connected.add(neighbour)
                queue.append(neighbour)
        self.cells = {key: surfaces[key] for key in connected}
        self.rejected_cells = set(lookup)-connected
        self.history.update({key: (surface, stamp) for key, surface in self.cells.items()})
        self.stamp = stamp
        return len(self.cells)

    def matches(self, points, stamp):
        result = np.zeros(len(points), dtype=bool)
        if not 0 <= stamp-self.stamp <= self.max_age:
            return result
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        valid = np.flatnonzero(np.all(np.isfinite(points), axis=1))
        if not len(valid):
            return result
        keys, inverse = np.unique(np.floor(points[valid, :2]/self.resolution).astype(int),
                                  axis=0, return_inverse=True)
        groups = np.split(np.argsort(inverse, kind='stable'), np.cumsum(np.bincount(inverse))[:-1])
        # Accumulated hit boxes may have thousands of corners per cell. Test
        # these together, without dropping any height or changing the margin.
        for key_array, group in zip(keys, groups):
            key = tuple(key_array)
            indices = valid[group]
            patch = points[indices]
            matched = np.zeros(len(patch), dtype=bool)
            # Test adjacent cells only within the same 4 cm observation margin,
            # avoiding a discontinuity at a grid boundary, not extending FOV.
            for dx, dy in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
                candidate = (key[0]+dx, key[1]+dy)
                record = self.history.get(candidate)
                if record is None:
                    continue
                surface, observed_at = record
                if not 0 <= stamp-observed_at <= max(self.max_age, self.memory_seconds):
                    continue
                # A currently observed non-ground cell vetoes neighbouring
                # ground support. Only genuinely unobserved edges can bridge.
                if candidate != key and key in self.rejected_cells:
                    continue
                xy, z, gradient, lo, hi = surface
                covered = np.all((patch[:, :2] >= lo-.04) & (patch[:, :2] <= hi+.04), axis=1)
                matched |= covered & (np.abs(patch[:, 2]-z-(patch[:, :2]-xy) @ gradient)
                                      <= self.height_tolerance)
                if np.all(matched):
                    break
            result[indices] = matched
        return result

    def object_costs(self, points, obstacle_height=.12, min_samples=3):
        """Depth obstacles above connected ground; terrain shape has no cost.

        A reference is valid only in a directly observed ground cell or within
        0.45 m of one. No reference means unknown, never free. All directly
        observed points participate, so a solid in a ground cell still wins.
        """
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        keys, inverse = np.unique(np.floor(points[:, :2]/self.resolution).astype(int),
                                  axis=0, return_inverse=True)
        centres, costs = [], []
        groups = np.split(np.argsort(inverse, kind='stable'),
                          np.cumsum(np.bincount(inverse))[:-1]) if len(points) else []
        for key_array, group in zip(keys, groups):
            if len(group) < min_samples:
                continue
            key = tuple(key_array)
            centre = (key_array+.5)*self.resolution
            refs = []
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    ref = self.cells.get((key[0]+dx, key[1]+dy))
                    if ref is not None and np.linalg.norm(ref[0]-centre) <= .45:
                        refs.append(ref)
            cost = -1
            if refs:
                own = points[group]
                predictions = np.array([z+(own[:, :2]-xy) @ gradient
                                        for xy, z, gradient, _, _ in refs])
                # Disagreeing references near a discontinuity do not establish
                # ground. LiDAR remains responsible for unclassified returns.
                consistent = np.ptp(predictions, axis=0) <= .10
                above = own[:, 2]-np.median(predictions, axis=0)
                if np.count_nonzero(consistent & (above > obstacle_height)) >= min_samples:
                    cost = 100
                elif key in self.cells:
                    cost = 0
            centres.append(centre)
            costs.append(cost)
        return np.asarray(centres).reshape(-1, 2), np.asarray(costs, dtype=np.int8)

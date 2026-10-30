"""Two representative strategies, forward-only recalculation, angle-only output.

Both paths retain the original measured-point window, stage-3 within-source
height-span check, TLS quality gates, height fallback and persistent estimates.
Neither path compares camera and LiDAR representative heights (old stage 4).
"""
from dataclasses import asdict, replace
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from .core import quaternion_matrix
from .fusion_core import FusedSlopeMap, representatives, weighted_tls
from ._geometry.traversability_core import CellEstimate, fit_plane
from ._geometry.quality_mapping_core import spread_quality, contains_target


MODES = ('sensor_separate', 'pooled_raw')


def refit_eight_neighbors(key, heights, config, quality):
    """Fit available heights of the eight adjacent cells, excluding the target.

    Keep the original connectivity, height discontinuity, support, RMS and
    target-height acceptance rules. The target height is validation only.
    """
    if key not in heights or not np.isfinite(heights[key]).all():
        return None
    candidates = {(key[0] + dx, key[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                  if (key[0] + dx, key[1] + dy) in heights
                  and np.isfinite(heights[(key[0] + dx, key[1] + dy)]).all()}
    connected, queue = {key}, [key]
    while queue:
        current = queue.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            other = (current[0] + dx, current[1] + dy)
            if other not in candidates:
                continue
            if abs(heights[other][0] - heights[current][0]) > quality.max_connected_height_jump_m:
                return None
            if other not in connected:
                connected.add(other);queue.append(other)
    keys = sorted(connected - {key})
    if len(keys) < quality.min_height_cells:
        return None
    resolution = config.resolution_m
    center = (np.asarray(key) + .5) * resolution
    xyz = np.array([((x + .5) * resolution, (y + .5) * resolution, heights[(x, y)][0]) for x, y in keys])
    if not spread_quality(xyz, quality)[0] or not contains_target(xyz[:, :2], center):
        return None
    fitted = fit_plane(xyz, config)
    if fitted is None or fitted.residual_m > quality.max_refit_rms_m:
        return None
    nz = abs(fitted.normal[2])
    target = np.r_[center, heights[key][0]]
    if nz < 1e-8 or abs(np.dot(fitted.normal, target) + fitted.offset_m) / nz > quality.max_target_error_m:
        return None
    return fitted


class ForwardROI:
    """Union of sensor-origin horizontal forward half-disks, in odom."""
    def __init__(self, mounts, radius_m=2.):
        if not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError('Forward radius must be positive')
        self.mounts = np.asarray(mounts, dtype=float)
        if self.mounts.shape != (2, 4, 4) or not np.isfinite(self.mounts).all():
            raise ValueError('Two finite sensor mount matrices required')
        self.radius_m = radius_m
        self.pose = None
        self.stamp_s = None

    def set_pose(self, stamp_s, position, quaternion):
        rotation = quaternion_matrix(quaternion)
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all() or not math.isfinite(stamp_s):
            raise ValueError('Finite reference pose required')
        self.origins = np.array([(rotation @ m[:3, 3] + position)[:2] for m in self.mounts])
        # Laser +X and camera optical +Z are their physical forward axes.
        self.forward = np.array([(rotation @ self.mounts[0, :3, 0])[:2],
                                 (rotation @ self.mounts[1, :3, 2])[:2]])
        lengths = np.linalg.norm(self.forward, axis=1)
        if np.any(lengths < 1e-8):
            raise ValueError('Sensor forward axis has no horizontal projection')
        self.forward /= lengths[:, None]
        self.pose = np.r_[position, quaternion].astype(float)
        self.stamp_s = float(stamp_s)

    def contains(self, xy, margin=0.):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        if self.pose is None:
            return np.zeros(len(xy), bool)
        delta = xy[:, None, :] - self.origins[None, :, :]
        distance2 = np.sum(delta * delta, axis=2)
        ahead = np.sum(delta * self.forward[None, :, :], axis=2)
        return np.any((distance2 <= (self.radius_m + margin)**2 + 1e-10)
                      & (ahead >= -margin - 1e-10), axis=1)


class ForwardSlopeMap(FusedSlopeMap):
    def __init__(self, config, quality, mounts, mode, forward_radius_m=2., max_size_m=160.):
        if mode not in MODES:
            raise ValueError('Unknown representative strategy')
        super().__init__(config, quality, max_size_m=max_size_m)
        self.mode = mode
        self.roi = ForwardROI(mounts, forward_radius_m)
        self.previous_active = set()
        self.calculated = {}
        self.timing_count = 0
        self.timing_sum_ms = 0.
        self.last_ms = 0.
        self.last_calculated_count = 0

    def _prepare(self):
        # The support halo is not an output ROI: a boundary cell may use points
        # just beyond 2m while the cell itself must pass the strict ROI test.
        pieces = [[], []]
        keys = list(self.samples)
        if keys:
            centers = (np.asarray(keys) + .5) * self.config.resolution_m
            selected = self.roi.contains(centers, self.config.radius_m + self.config.resolution_m / math.sqrt(2))
            for key, keep in zip(keys, selected):
                if keep:
                    for source in (0, 1):
                        pieces[source].extend(p for _, p in self.samples[key][source])
        source_data = []
        for source in (0, 1):
            xyz = np.concatenate(pieces[source]).astype(float) if pieces[source] else np.empty((0, 3))
            means, bins, counts = representatives(xyz, self.quality.bin_size_m, 2)
            low = np.full(len(means), np.inf)
            high = np.full(len(means), -np.inf)
            if len(xyz):
                _, inverse = np.unique(np.floor(xyz[:, :2] / self.quality.bin_size_m).astype(np.int64), axis=0, return_inverse=True)
                np.minimum.at(low, inverse, xyz[:, 2])
                np.maximum.at(high, inverse, xyz[:, 2])
            source_data.append((means, bins, counts, high - low))
        means = np.concatenate([v[0] for v in source_data])
        bins = np.concatenate([v[1] for v in source_data])
        counts = np.concatenate([v[2] for v in source_data])
        spans = np.concatenate([v[3] for v in source_data])
        if self.mode == 'pooled_raw' and len(means):
            # Algebraically identical to concatenating every retained raw point
            # and taking its bin mean. Raw density is intentionally influential.
            keys, inverse = np.unique(bins, axis=0, return_inverse=True)
            total = np.bincount(inverse, weights=counts)
            means = np.column_stack([np.bincount(inverse, weights=means[:, i] * counts) / total for i in range(3)])
            # Stage 3 remains WITHIN each original sensor in both variants.
            # Unioning sensor Z ranges here would silently restore old stage 4.
            within_source_span = np.zeros(len(keys))
            np.maximum.at(within_source_span, inverse, spans)
            bins, counts, spans = keys, total, within_source_span
        self.prepared = (means, bins, counts, spans, cKDTree(means[:, :2]) if len(means) else None)

    def _fit(self, key):
        center = (np.asarray(key) + .5) * self.config.resolution_m
        means, bins, counts, spans, tree = self.prepared
        indices = tree.query_ball_point(center, self.config.radius_m + 1e-9) if tree is not None else []
        reps = means[indices]
        raw_count = int(counts[indices].sum())
        conflict = bool(np.any(spans[indices] > self.quality.max_connected_height_jump_m))
        unique_bins, inverse = np.unique(bins[indices], axis=0, return_inverse=True)
        fitted = None
        if not conflict and raw_count >= self.quality.min_raw_points and len(unique_bins) >= self.quality.min_xy_bins:
            weights = (1. / np.bincount(inverse)[inverse] if self.mode == 'sensor_separate' else np.ones(len(reps)))
            fitted = weighted_tls(reps, weights, self.config, self.quality)
            if fitted is not None:
                self.counts['direct_tls_success'] += 1
        if conflict:
            self.counts['stage3_within_sensor_conflicts'] += 1
            self.heights.pop(key, None)
        central = self.samples.get(key, {})
        centralpoints = [p for group in central.values() for _, p in group]
        # Stage 6 stays identical across the two variants.
        if centralpoints and not conflict:
            medians = [float(np.median(np.concatenate([p for _, p in group])[:, 2]))
                       for group in central.values() if group]
            self.heights[key] = (float(np.mean(medians)), self.latest)
        if fitted is None and not conflict:
            self.counts['height_refit_attempts'] += 1
            fitted = refit_eight_neighbors(key, self.heights, self.config, self.quality)
            if fitted is not None:
                self.counts['height_refit_success'] += 1
        old = self.estimates.get(key)
        if fitted is None and old is not None and old.normal is not None and not conflict:
            allpoints = np.concatenate(centralpoints) if centralpoints else np.empty((0, 3))
            if not len(allpoints) or np.median(np.abs(allpoints @ old.normal + old.plane_offset_m)) <= self.quality.max_target_error_m:
                self.estimates[key] = replace(old, slope_reused=True)
                self.counts['previous_slope_reused'] += 1
                return
        if fitted is None:
            self.estimates[key] = CellEstimate(height=self.heights.get(key, (None,))[0], total_points=raw_count)
            return
        q = min(1., len(unique_bins) / self.config.raw_reference_points, fitted.q_space)
        # No threshold, blocking decision, cost conversion or neighbor correction.
        self.estimates[key] = CellEstimate(height=self.heights.get(key, (None,))[0], slope_valid=True,
            slope_deg=fitted.slope_deg, q_g=q, normal=fitted.normal, plane_offset_m=fitted.offset_m,
            fit_residual_m=fitted.residual_m, total_points=raw_count)

    def snapshot(self):
        started = time.perf_counter()
        active = set()
        observed = list(self.observed)
        if observed:
            centers = (np.asarray(observed) + .5) * self.config.resolution_m
            active = {key for key, keep in zip(observed, self.roi.contains(centers)) if keep}
        reach = math.ceil(self.config.radius_m / self.config.resolution_m)
        changed = active - self.previous_active
        # Only inspect dirty cells close enough to influence the ROI. Distant
        # inputs remain retained, without preparing or fitting distant cells.
        dirty = list(self.dirty)
        if dirty:
            mask = self.roi.contains((np.asarray(dirty) + .5) * self.config.resolution_m,
                                     self.config.radius_m + 2 * self.config.resolution_m)
            for (x, y), keep in zip(dirty, mask):
                if keep:
                    changed.update((x + dx, y + dy) for dx in range(-reach, reach + 1)
                                   for dy in range(-reach, reach + 1) if (x + dx, y + dy) in active)
        if changed:
            self._prepare()
            for key in sorted(changed):
                self._fit(key)
                self.calculated[key] = (self.roi.stamp_s, self.roi.pose.copy())
            self.counts['calculated_cells'] += len(changed)
        if self.roi.pose is not None:
            self.previous_active = active
            self.dirty.clear()
        self.last_calculated_count = len(changed)
        result = self._angle_layers()
        self.last_ms = (time.perf_counter() - started) * 1000
        if changed:
            self.timing_count += 1
            self.timing_sum_ms += self.last_ms
        return result

    def _angle_layers(self):
        keys = np.asarray(list(self.estimates), dtype=np.int64).reshape(-1, 2)
        lo = keys.min(axis=0) - 1 if len(keys) else np.array([-1, -1])
        hi = keys.max(axis=0) + 2 if len(keys) else np.array([1, 1])
        shape = tuple((hi - lo)[::-1])
        output = dict(slope_deg=np.full(shape, np.nan, np.float32), height_m=np.full(shape, np.nan, np.float32),
                      confidence=np.zeros(shape, np.float32), observed=np.zeros(shape, bool),
                      slope_valid=np.zeros(shape, bool), slope_reused=np.zeros(shape, bool),
                      last_calculated_stamp_s=np.full(shape, np.nan),
                      last_calculated_base_pose=np.full((*shape, 7), np.nan),
                      origin_xy=lo * self.config.resolution_m, resolution_m=np.array(self.config.resolution_m))
        for key, item in self.estimates.items():
            column, row = np.asarray(key) - lo
            output['slope_deg'][row, column] = item.slope_deg if item.slope_valid else np.nan
            output['height_m'][row, column] = item.height if item.height is not None else np.nan
            output['confidence'][row, column] = item.q_g
            output['observed'][row, column] = True
            output['slope_valid'][row, column] = item.slope_valid
            output['slope_reused'][row, column] = item.slope_reused
            output['last_calculated_stamp_s'][row, column], output['last_calculated_base_pose'][row, column] = self.calculated[key]
        return output

    def metadata(self):
        return dict(schema='nomad-slope-angle-v1', representative_mode=self.mode,
                    cost_computed=False, cross_sensor_representative_conflict_check=False,
                    stage3_conflict_scope=f'raw Z span within each original sensor and {self.quality.bin_size_m * 100:g}cm XY bin, in both modes',
                    height_summary='mean of per-source cell median Z, unchanged across variants',
                    height_refit_neighborhood='eight immediately adjacent cells, excluding central target; minimum six observed neighbor heights; target used only for height validation',
                    forward_radius_m=self.roi.radius_m, roi='union of sensor-origin horizontal forward half-disks',
                    support_halo_m=self.config.radius_m, persistent_estimates=True,
                    candidate_settings=asdict(self.config), plane_quality=asdict(self.quality),
                    core_counters=dict(self.counts), timing=dict(samples=self.timing_count,
                    mean_ms=self.timing_sum_ms / self.timing_count if self.timing_count else 0., last_ms=self.last_ms,
                    scope='target selection, representative preparation, TLS, height storage/refit, persistence and angle layers; excludes raw ingestion/rendering/IO'))

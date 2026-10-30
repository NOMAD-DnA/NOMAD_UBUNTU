"""Reuse query/bin/XY geometry work without changing the frozen acceptance rules.

The reference implementation stays in slope_comparison_core.py. Raw ingestion,
expiry/caps, per-cell fit order, TLS arithmetic and old-plane reuse are preserved.
Caches are rebuilt each snapshot, except XY-only height-refit geometry. Fitted
planes, heights and acceptance decisions remain specific to each target cell.
"""
from dataclasses import replace
import math
import time
import numpy as np
from scipy.spatial import cKDTree

from .slope_comparison_core import ForwardSlopeMap
from .fusion_core import weighted_tls
from ._geometry.traversability_core import CellEstimate, fit_plane
from ._geometry.quality_mapping_core import spread_quality, contains_target


class OptimizedForwardSlopeMap(ForwardSlopeMap):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._neighbor_indices = {}
        self._refit_xy = {}
        self.verify_neighbor_order = False

    def _prepare(self):
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
            if not np.isfinite(xyz).all():
                raise ValueError('Nonfinite fusion XYZ')
            # One grouping provides means, population and Z span together.
            bins, inverse, counts = np.unique(
                np.floor(xyz[:, :2] / self.quality.bin_size_m).astype(np.int64),
                axis=0, return_inverse=True, return_counts=True)
            means = np.column_stack([np.bincount(inverse, weights=xyz[:, i]) / counts for i in range(3)])
            low, high = np.full(len(means), np.inf), np.full(len(means), -np.inf)
            np.minimum.at(low, inverse, xyz[:, 2]); np.maximum.at(high, inverse, xyz[:, 2])
            source_data.append((means, bins, counts, high-low))
        means = np.concatenate([v[0] for v in source_data])
        bins = np.concatenate([v[1] for v in source_data])
        counts = np.concatenate([v[2] for v in source_data])
        spans = np.concatenate([v[3] for v in source_data])
        if self.mode == 'pooled_raw' and len(means):
            keys, inverse = np.unique(bins, axis=0, return_inverse=True)
            total = np.bincount(inverse, weights=counts)
            means = np.column_stack([np.bincount(inverse, weights=means[:, i]*counts)/total for i in range(3)])
            within_source_span = np.zeros(len(keys))
            np.maximum.at(within_source_span, inverse, spans)
            bins, counts, spans = keys, total, within_source_span
        _, self._bin_ids = np.unique(bins, axis=0, return_inverse=True)
        self.prepared = means, bins, counts, spans, cKDTree(means[:, :2]) if len(means) else None

    def _query_targets(self, keys):
        tree = self.prepared[-1]
        centers = (np.asarray(keys) + .5) * self.config.resolution_m
        # Scalar reference queries return tree order, not sorted point indices.
        # Preserve that order to preserve every TLS floating-point accumulation.
        indices = tree.query_ball_point(centers, self.config.radius_m+1e-9,
                                        return_sorted=False) if tree is not None else [[] for _ in keys]
        self._neighbor_indices = dict(zip(keys, indices))
        if self.verify_neighbor_order and tree is not None:
            for key, center in zip(keys, centers):
                assert list(self._neighbor_indices[key]) == tree.query_ball_point(center, self.config.radius_m+1e-9), key

    def _refit_heights(self, key):
        heights, quality = self.heights, self.quality
        def finite(value):
            return value is not None and math.isfinite(value[0]) and math.isfinite(value[1])
        if not finite(heights.get(key)):
            return None
        candidates = {(key[0]+dx, key[1]+dy) for dx in (-1,0,1) for dy in (-1,0,1)
                      if finite(heights.get((key[0]+dx, key[1]+dy)))}
        connected, queue = {key}, [key]
        while queue:
            current = queue.pop()
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                other = current[0]+dx, current[1]+dy
                if other not in candidates:
                    continue
                if abs(heights[other][0]-heights[current][0]) > quality.max_connected_height_jump_m:
                    return None
                if other not in connected:
                    connected.add(other); queue.append(other)
        keys = tuple(sorted(connected-{key}))
        if len(keys) < quality.min_height_cells:
            return None
        cached = self._refit_xy.get(key)
        if cached is None or cached[0] != keys:
            r = self.config.resolution_m
            center = (np.asarray(key)+.5)*r
            xy = np.array([((x+.5)*r,(y+.5)*r) for x,y in keys])
            ok = spread_quality(np.column_stack((xy,np.zeros(len(xy)))),quality)[0] and contains_target(xy,center)
            cached = keys, xy, center, ok
            self._refit_xy[key] = cached
        _, xy, center, ok = cached
        if not ok:
            return None
        xyz = np.column_stack((xy, [heights[k][0] for k in keys]))
        fitted = fit_plane(xyz, self.config)
        if fitted is None or fitted.residual_m > quality.max_refit_rms_m:
            return None
        nz = abs(fitted.normal[2]); target = np.r_[center, heights[key][0]]
        if nz < 1e-8 or abs(np.dot(fitted.normal,target)+fitted.offset_m)/nz > quality.max_target_error_m:
            return None
        return fitted

    def _fit(self, key):
        means, bins, counts, spans, tree = self.prepared
        indices = self._neighbor_indices.get(key)
        if indices is None:
            center = (np.asarray(key)+.5)*self.config.resolution_m
            indices = tree.query_ball_point(center,self.config.radius_m+1e-9) if tree is not None else []
        reps = means[indices]
        raw_count = int(counts[indices].sum())
        conflict = bool(np.any(spans[indices] > self.quality.max_connected_height_jump_m))
        # Local populations matter: the other sensor's representative in the
        # same XY bin may lie OUTSIDE this target's circle. Global weights fail.
        histogram = np.bincount(self._bin_ids[indices])
        bin_count = int(np.count_nonzero(histogram))
        fitted = None
        if not conflict and raw_count >= self.quality.min_raw_points and bin_count >= self.quality.min_xy_bins:
            weights = 1./histogram[self._bin_ids[indices]] if self.mode=='sensor_separate' else np.ones(len(reps))
            fitted = weighted_tls(reps,weights,self.config,self.quality)
            if fitted is not None:
                self.counts['direct_tls_success'] += 1
        if conflict:
            self.counts['stage3_within_sensor_conflicts'] += 1
            self.heights.pop(key,None)
        central = self.samples.get(key,{})
        centralpoints = [p for group in central.values() for _,p in group]
        if centralpoints and not conflict:
            medians = [float(np.median(np.concatenate([p for _,p in group])[:,2]))
                       for group in central.values() if group]
            self.heights[key] = float(np.mean(medians)), self.latest
        if fitted is None and not conflict:
            self.counts['height_refit_attempts'] += 1
            fitted = self._refit_heights(key)
            if fitted is not None:
                self.counts['height_refit_success'] += 1
        old = self.estimates.get(key)
        if fitted is None and old is not None and old.normal is not None and not conflict:
            allpoints = np.concatenate(centralpoints) if centralpoints else np.empty((0,3))
            if not len(allpoints) or np.median(np.abs(allpoints@old.normal+old.plane_offset_m)) <= self.quality.max_target_error_m:
                self.estimates[key] = replace(old,slope_reused=True)
                self.counts['previous_slope_reused'] += 1
                return
        if fitted is None:
            self.estimates[key] = CellEstimate(height=self.heights.get(key,(None,))[0],total_points=raw_count)
            return
        q = min(1.,bin_count/self.config.raw_reference_points,fitted.q_space)
        self.estimates[key] = CellEstimate(height=self.heights.get(key,(None,))[0],slope_valid=True,
            slope_deg=fitted.slope_deg,q_g=q,normal=fitted.normal,plane_offset_m=fitted.offset_m,
            fit_residual_m=fitted.residual_m,total_points=raw_count)

    def snapshot(self):
        started = time.perf_counter()
        active = set(); observed = list(self.observed)
        if observed:
            centers = (np.asarray(observed)+.5)*self.config.resolution_m
            active = {key for key,keep in zip(observed,self.roi.contains(centers)) if keep}
        reach = math.ceil(self.config.radius_m/self.config.resolution_m)
        changed = active-self.previous_active
        dirty = list(self.dirty)
        if dirty:
            mask = self.roi.contains((np.asarray(dirty)+.5)*self.config.resolution_m,
                                     self.config.radius_m+2*self.config.resolution_m)
            for (x,y),keep in zip(dirty,mask):
                if keep:
                    changed.update((x+dx,y+dy) for dx in range(-reach,reach+1)
                                   for dy in range(-reach,reach+1) if (x+dx,y+dy) in active)
        if changed:
            self._prepare()
            keys = sorted(changed); self._query_targets(keys)
            # Height refits see the same sequentially updated neighbors.
            for key in keys:
                self._fit(key)
                self.calculated[key] = self.roi.stamp_s, self.roi.pose.copy()
            self._neighbor_indices.clear()
            self.counts['calculated_cells'] += len(changed)
        if self.roi.pose is not None:
            self.previous_active = active; self.dirty.clear()
        self.last_calculated_count = len(changed)
        result = self._angle_layers()
        self.last_ms = (time.perf_counter()-started)*1000
        if changed:
            self.timing_count += 1; self.timing_sum_ms += self.last_ms
        return result

    def metadata(self):
        return {**super().metadata(), 'calculation_optimization': {
            'name': 'exact_neighborhood_and_xy_reuse_20261029',
            'batch_neighbor_search': True, 'query_return_sorted': False,
            'local_sensor_weights': True, 'height_refit_order': 'sorted target keys, unchanged',
            'height_refit_cache': 'XY geometry only; heights and planes recomputed'}}

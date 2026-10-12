"""Opt-in plane acceptance and ground-height refitting for geometry mapping.

The original TraversabilityCore and its experiment defaults remain unchanged.
Both scalar and batched raw fits pass through the same acceptance boundary.
Spatial covariance and bounded neighborhood selection follow the principles in
Demantke et al. (2011), DOI 10.5194/isprsarchives-XXXVIII-5-W12-97-2011.
Thresholds below are local engineering settings, not published optimal values.
"""
from collections import OrderedDict, Counter
from dataclasses import dataclass, asdict, replace
import math
import numpy as np

from .traversability_core import TraversabilityCore, fit_plane


@dataclass(frozen=True)
class PlaneQualityConfig:
    bin_size_m: float = 0.05
    min_raw_points: int = 12
    min_xy_bins: int = 6
    min_minor_std_m: float = 0.01
    min_xy_variance_ratio: float = 0.02
    refit_radii_m: tuple = (0.3, 0.45, 0.6)
    min_height_cells: int = 6
    max_refit_rms_m: float = 0.01
    max_target_error_m: float = 0.02
    max_connected_height_jump_m: float = 0.12
    min_conflict_points: int = 3
    diagnostic_max_cases: int = 256
    diagnostic_max_points: int = 128

    def __post_init__(self):
        for name in ('min_raw_points', 'min_xy_bins', 'min_height_cells',
                     'min_conflict_points', 'diagnostic_max_cases', 'diagnostic_max_points'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be a positive integer')
        for name in ('bin_size_m', 'min_minor_std_m', 'min_xy_variance_ratio',
                     'max_refit_rms_m', 'max_target_error_m', 'max_connected_height_jump_m'):
            value = getattr(self, name)
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.min_xy_variance_ratio > 1 or self.min_height_cells < 3 or self.min_xy_bins < 3:
            raise ValueError('invalid plane geometry requirements')
        radii = tuple(self.refit_radii_m)
        if (not radii or any(not math.isfinite(r) or r <= 0 for r in radii)
                or any(b <= a for a, b in zip(radii, radii[1:]))):
            raise ValueError('refit radii must be finite, positive and strictly increasing')
        object.__setattr__(self, 'refit_radii_m', radii)


def spread_quality(xyz, quality):
    """Physical XY spread; units do not change when the query radius grows."""
    xy = np.asarray(xyz, dtype=np.float64)[:, :2]
    if len(xy) < 3:
        return False, 0., 0.
    centered = xy - xy.mean(axis=0)
    eigen = np.linalg.eigvalsh(centered.T @ centered / len(xy))
    minor = math.sqrt(max(0., float(eigen[0])))
    ratio = max(0., float(eigen[0])) / float(eigen[1]) if eigen[1] > 0 else 0.
    return (minor >= quality.min_minor_std_m and ratio >= quality.min_xy_variance_ratio), minor, ratio


def bin_representatives(xyz, size):
    """Equal weight per XY bin; repeated captures cannot dominate a fit."""
    xyz = np.asarray(xyz, dtype=np.float64)
    if not len(xyz):
        return np.empty((0, 3))
    _, inverse = np.unique(np.floor(xyz[:, :2] / size).astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    return np.column_stack([np.bincount(inverse, weights=xyz[:, axis]) / counts for axis in range(3)])


def contains_target(xy, target):
    """Convex hull test without a SciPy runtime dependency or extrapolation."""
    points = sorted(set(map(tuple, np.asarray(xy, dtype=float))))
    if len(points) < 3:
        return False
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    return len(hull) >= 3 and all(cross(a, b, target) >= -1e-10 for a, b in zip(hull, hull[1:]+hull[:1]))


class QualityMappingCore(TraversabilityCore):
    def __init__(self, config, quality=None, *, allow_height_refit=True):
        if not (config.geometry_only and config.slope_input == 'raw_points' and config.plane_method == 'tls'):
            raise ValueError('QualityMappingCore requires geometry-only raw-point TLS')
        self.quality = quality or PlaneQualityConfig()
        self.allow_height_refit = allow_height_refit
        self.quality_records = {}
        self.rejected_support = OrderedDict()
        self._dependents = {}
        self._dependencies = {}
        self._accepted = set()
        super().__init__(config)
        if self.quality.refit_radii_m[0] < config.radius_m:
            raise ValueError('height refit must start at or beyond the base radius')

    def reset(self):
        super().reset()
        self.quality_records.clear()
        self.rejected_support.clear()
        self._dependents.clear()
        self._dependencies.clear()
        self._accepted.clear()

    def _mark_changed(self, key):
        super()._mark_changed(key)
        self._dirty.update(k for k in self._dependents.get(key, ()) if k in self._cells)

    def _watch_heights(self, key, enabled):
        for other in self._dependencies.pop(key, ()):
            targets = self._dependents[other]
            targets.discard(key)
            if not targets:
                del self._dependents[other]
        if not enabled:
            return
        reach = math.ceil(self.quality.refit_radii_m[-1] / self.config.resolution_m)
        others = {(key[0]+dx, key[1]+dy) for dx in range(-reach, reach+1)
                  for dy in range(-reach, reach+1)
                  if math.hypot(dx, dy)*self.config.resolution_m <= self.quality.refit_radii_m[-1]+1e-9}
        self._dependencies[key] = others
        for other in others:
            self._dependents.setdefault(other, set()).add(key)

    def _latest_conflicts(self, key, fitted):
        points = self._cells[key].points
        if not len(points):
            return False
        latest = points[points['stamp'] == points['stamp'].max()]['xyz'].astype(np.float64)
        if len(latest) < self.quality.min_conflict_points:
            return False
        normal = np.asarray(fitted.normal)
        # Compare at measured XY, not at a nominal cell center on an incline.
        if abs(normal[2]) < 1e-8:
            return False
        vertical = (latest @ normal + fitted.offset_m) / normal[2]
        return abs(float(np.median(vertical))) > self.quality.max_target_error_m

    def refit_heights(self, key, heights):
        """Refit observed, connected height cells; central evidence is required.

        heights maps cell indices to (median_z, capture_stamp). Stored heights
        are measurement summaries, never inferred heights from another refit.
        """
        if key not in heights or not np.isfinite(heights[key]).all():
            return None, np.empty((0, 3)), np.empty(0), 0., 'missing_target_height'
        resolution = self.config.resolution_m
        center = (np.asarray(key, dtype=float)+.5)*resolution
        last_reason = 'insufficient_height_cells'
        for radius in self.quality.refit_radii_m:
            reach = math.ceil(radius/resolution)
            candidates = {(key[0]+dx, key[1]+dy) for dx in range(-reach, reach+1)
                          for dy in range(-reach, reach+1)
                          if math.hypot(dx, dy)*resolution <= radius+1e-9
                          and (key[0]+dx, key[1]+dy) in heights
                          and np.isfinite(heights[(key[0]+dx, key[1]+dy)]).all()}
            connected, queue, discontinuity = {key}, [key], False
            while queue:
                current = queue.pop()
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    other = (current[0]+dx, current[1]+dy)
                    if (other in candidates and
                            abs(heights[other][0]-heights[current][0]) > self.quality.max_connected_height_jump_m):
                        discontinuity = True
                    if (other in candidates and other not in connected and
                            abs(heights[other][0]-heights[current][0]) <= self.quality.max_connected_height_jump_m):
                        connected.add(other)
                        queue.append(other)
            if discontinuity:
                last_reason = 'height_discontinuity'
                # Enlarging this support would blend a known height boundary.
                return None, np.empty((0, 3)), np.empty(0), 0., last_reason
            keys = sorted(connected)
            if len(keys) < self.quality.min_height_cells:
                continue
            xyz = np.asarray([((k[0]+.5)*resolution, (k[1]+.5)*resolution, heights[k][0]) for k in keys])
            usable, _, _ = spread_quality(xyz, self.quality)
            surrounding = xyz[np.any(np.abs(xyz[:, :2]-center) > 1e-9, axis=1), :2]
            if not usable or not contains_target(surrounding, center):
                last_reason = 'height_support_extrapolation_or_degenerate'
                continue
            fitted = fit_plane(xyz, self.config)
            if fitted is None or fitted.residual_m > self.quality.max_refit_rms_m:
                last_reason = 'height_surface_not_planar'
                continue
            target = np.r_[center, heights[key][0]]
            nz = abs(fitted.normal[2])
            if nz < 1e-8 or abs(np.dot(fitted.normal, target)+fitted.offset_m)/nz > self.quality.max_target_error_m:
                last_reason = 'target_height_inconsistent'
                continue
            return fitted, xyz, np.array([heights[k][1] for k in keys]), radius, 'accepted'
        return None, np.empty((0, 3)), np.empty(0), 0., last_reason

    def _save_rejected(self, key, xyz, stamps, raw):
        if raw is not None and raw.slope_deg < self.config.slope_limit_deg:
            return
        indices = np.linspace(0, len(xyz)-1, min(len(xyz), self.quality.diagnostic_max_points), dtype=int)
        self.rejected_support.pop(key, None)
        self.rejected_support[key] = (np.asarray(xyz)[indices].copy(), np.asarray(stamps)[indices].copy())
        while len(self.rejected_support) > self.quality.diagnostic_max_cases:
            self.rejected_support.popitem(last=False)

    def _make_estimate(self, key, support_xyz, support_stamps, fitted_override=None, fit_supplied=False):
        raw = fitted_override if fit_supplied else fit_plane(support_xyz, self.config)
        previous = self._cells[key].last_plane
        valid, minor, ratio = spread_quality(support_xyz, self.quality)
        fitted = None
        bins = 0
        reason = 'raw_spread_insufficient'
        if raw is not None and len(support_xyz) >= self.quality.min_raw_points and valid:
            representatives = bin_representatives(support_xyz, self.quality.bin_size_m)
            bins = len(representatives)
            rep_valid, _, _ = spread_quality(representatives, self.quality)
            if bins >= self.quality.min_xy_bins and rep_valid:
                fitted = fit_plane(representatives, self.config)
                reason = 'accepted' if fitted is not None else 'raw_normal_degenerate'
            else:
                reason = 'raw_bins_insufficient'
        elif len(support_xyz) < self.quality.min_raw_points:
            reason = 'raw_points_insufficient'
        if fitted is not None and self._latest_conflicts(key, fitted):
            fitted, reason = None, 'new_capture_height_conflict'
        source, chosen_xyz, chosen_stamps, radius = 'raw', support_xyz, support_stamps, self.config.radius_m
        self._watch_heights(key, self.allow_height_refit and fitted is None)
        if fitted is None and self.allow_height_refit:
            self._save_rejected(key, support_xyz, support_stamps, raw)
            # All summaries have been refreshed by TraversabilityCore.snapshot.
            reach = math.ceil(self.quality.refit_radii_m[-1]/self.config.resolution_m)
            heights = {}
            for dx in range(-reach, reach+1):
                for dy in range(-reach, reach+1):
                    other = (key[0]+dx, key[1]+dy)
                    cell = self._cells.get(other)
                    if cell is not None and cell.last_height is not None:
                        heights[other] = (cell.last_height[1], cell.last_height[3])
            fitted, chosen_xyz, chosen_stamps, radius, refit_reason = self.refit_heights(key, heights)
            if fitted is not None and self._latest_conflicts(key, fitted):
                fitted, refit_reason = None, 'new_capture_height_conflict'
            source = 'height_refit'
            if fitted is None:
                reason += ':' + refit_reason
        if fitted is not None:
            reference = self.config.raw_reference_points if source == 'raw' else self.quality.min_height_cells
            q_count = min(1., len(chosen_xyz)/reference)
            q_g = min(q_count, fitted.q_space)
            self._cells[key].last_plane = (fitted, len(chosen_xyz), q_count, q_g,
                                          float(np.min(chosen_stamps)), float(np.max(chosen_stamps)))
            self._accepted.add(key)
        elif previous is not None and not self._latest_conflicts(key, previous[0]):
            source, radius = 'retained', self.quality_records.get(key, {}).get('radius_m', self.config.radius_m)
            self._cells[key].last_plane = previous
        else:
            source, radius = 'uncertain', 0.
            self._cells[key].last_plane = None
        estimate = super()._make_estimate(key, support_xyz, support_stamps, fitted_override=None, fit_supplied=True)
        record = {'source': source, 'reason': reason, 'radius_m': radius,
                  'raw_slope_deg': np.nan if raw is None else raw.slope_deg,
                  'raw_q_g': 0. if raw is None else min(1., len(support_xyz)/self.config.raw_reference_points, raw.q_space),
                  'raw_minor_std_m': minor, 'raw_xy_variance_ratio': ratio, 'raw_xy_bins': bins}
        self.quality_records[key] = record
        self.diagnostics['quality_' + source] += 1
        return replace(estimate, slope_reused=source == 'retained')

    def snapshot(self, now_s, targets=None):
        self._accepted.clear()
        result = super().snapshot(now_s, targets)
        self.last_snapshot_new_fit_keys = frozenset(self._accepted)
        self.last_snapshot_reused_keys = frozenset(
            k for k in self.last_snapshot_fit_keys if self.quality_records[k]['source'] == 'retained')
        return result

    def quality_arrays(self, cells):
        records = [self.quality_records.get(tuple(k), {}) for k in cells]
        codes = {'raw': 1, 'height_refit': 2, 'retained': 3, 'uncertain': 4}
        arrays = {'plane_source': np.array([codes.get(r.get('source'), 0) for r in records], dtype=np.uint8),
                  'plane_rejection_reason': np.array([r.get('reason', '') for r in records], dtype='U100')}
        for field in ('radius_m', 'raw_slope_deg', 'raw_q_g', 'raw_minor_std_m', 'raw_xy_variance_ratio', 'raw_xy_bins'):
            arrays['plane_' + field] = np.array([r.get(field, np.nan) for r in records])
        return arrays

    def diagnostic_arrays(self):
        keys = list(self.rejected_support)
        sizes = [len(self.rejected_support[k][0]) for k in keys]
        return {'diagnostic_raw_cells': np.array(keys, dtype=np.int64).reshape(-1, 2),
                'diagnostic_raw_offsets': np.r_[0, np.cumsum(sizes)].astype(np.int64),
                'diagnostic_raw_xyz': np.concatenate([self.rejected_support[k][0] for k in keys]) if keys else np.empty((0, 3)),
                'diagnostic_raw_capture_stamps_s': np.concatenate([self.rejected_support[k][1] for k in keys]) if keys else np.empty(0)}

    def quality_metadata(self):
        return {'enabled': True, 'allow_height_refit': self.allow_height_refit, **asdict(self.quality),
                'current_cell_sources': dict(Counter(r['source'] for r in self.quality_records.values())),
                'source_codes': {'1': 'raw', '2': 'height_refit', '3': 'retained', '4': 'uncertain'},
                'uncertain_cost': 100, 'uncertain_is_confirmed_blocked': False,
                'diagnostic_raw_cases': len(self.rejected_support)}

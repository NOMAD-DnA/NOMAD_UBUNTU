"""Camera-only spatial rejection and equal-frame voxel fusion.

This is not TSDF, ICP, calibration refinement, or ground-truth accuracy
correction. Thresholds are engineering
defaults, not measured OAK confidence or uncertainty values.
"""
from dataclasses import dataclass, asdict
from collections import deque
import time
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class SORSettings:
    """Selected offline experiment: 80/1.0, 3 cm, raw-point weighting."""
    min_z_m: float = .3
    max_z_m: float = 4.
    voxel_m: float = .03
    sor_neighbors: int = 80
    sor_std_ratio: float = 1.

    def __post_init__(self):
        if (not np.isfinite([self.min_z_m, self.max_z_m, self.voxel_m, self.sor_std_ratio]).all()
                or not 0 < self.min_z_m < self.max_z_m or self.voxel_m <= 0
                or not isinstance(self.sor_neighbors, (int, np.integer))
                or self.sor_neighbors < 2 or self.sor_std_ratio <= 0):
            raise ValueError('Invalid SOR settings')


def range_sor(camera, world, cfg):
    """Match compare_sor.sor: self excluded, ddof=0, <=, small frames unchanged."""
    camera, world = np.asarray(camera, float), np.asarray(world, float)
    if camera.ndim != 2 or camera.shape[1] != 3 or world.shape != camera.shape:
        raise ValueError('Expected matching camera/world Nx3 arrays')
    if not np.isfinite(world).all():
        raise ValueError('Nonfinite world point')
    keep = (np.isfinite(camera).all(axis=1) & (camera[:, 2] >= cfg.min_z_m)
            & (camera[:, 2] <= cfg.max_z_m))
    p = world[keep]
    range_kept = len(p)
    if len(p) > cfg.sor_neighbors:
        distances = cKDTree(p).query(p, k=cfg.sor_neighbors + 1, workers=1)[0][:, 1:].mean(axis=1)
        p = p[distances <= distances.mean() + cfg.sor_std_ratio * distances.std(ddof=0)]
    return p, range_kept


class SORDepth:
    """Per-capture SOR before map accumulation; never filters the growing map.

    No repeated-frame or radius rejection: those were disabled in the selected
    experiment. Retain sums/counts of raw surviving points, not equal-frame means.
    """
    def __init__(self, settings=None, frame_period_s=.5):
        from .map_core import Voxels
        self.settings = settings or SORSettings()
        if not np.isfinite(frame_period_s) or frame_period_s <= 0:
            raise ValueError('Invalid depth frame period')
        self.model = Voxels(self.settings.voxel_m)
        self.last_stamp = None
        self.frames = self.input_points = self.range_kept = self.sor_kept = 0
        self.latest_points = self.over_budget_frames = 0
        self.frame_budget_ms = frame_period_s * 1000
        self.timings_ms = deque(maxlen=256)
        self._cache = None

    def add_frame(self, camera, world, stamp_ns):
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise ValueError('Each independent depth frame must have an increasing timestamp')
        start = time.perf_counter()
        p, ranged = range_sor(camera, world, self.settings)
        self.model.add(p)
        self.last_stamp = int(stamp_ns)
        self.frames += 1
        self.input_points += len(camera)
        self.range_kept += ranged
        self.sor_kept += len(p)
        self.latest_points = len(p)
        self._cache = None
        elapsed_ms = (time.perf_counter() - start) * 1000
        self.timings_ms.append(elapsed_ms)
        self.over_budget_frames += elapsed_ms > self.frame_budget_ms
        return p

    def arrays(self):
        if self._cache is None:
            self._cache = self.model.arrays()
        return self._cache

    def report(self):
        return dict(enabled=True, profile='selected_sor',
            algorithm='camera optical range + per-frame SOR + raw-observation voxel mean',
            settings=asdict(self.settings), frames=self.frames, input_points=self.input_points,
            range_kept_points=self.range_kept, sor_kept_points=self.sor_kept,
            latest_output_points=self.latest_points, candidate_voxels=len(self.model.cells),
            output_voxels=len(self.model.cells), repeated_frame_filter=False, radius_filter=False,
            observations_meaning='raw SOR-surviving point observations contributing to this voxel',
            sor_self_excluded=True, std_ddof=0, neighbor_query_workers=1,
            timing=dict(scope='range + SOR + voxel accumulation; excludes ROS, disk, display and terrain',
                recent_frames=len(self.timings_ms), frame_budget_ms=self.frame_budget_ms,
                last_ms=self.timings_ms[-1] if self.timings_ms else None,
                recent_mean_ms=float(np.mean(self.timings_ms)) if self.timings_ms else None,
                recent_p95_ms=float(np.percentile(self.timings_ms, 95)) if self.timings_ms else None,
                over_budget_frames=int(self.over_budget_frames)),
            evaluation_tolerance_m=.05, evaluation_tolerance_used_for_filter=False,
            accuracy_verified=False, pose_refinement=False, lidar_used_for_filter=False)


@dataclass(frozen=True)
class FilterSettings:
    min_z_m: float = .3
    max_z_m: float = 4.
    voxel_m: float = .05
    sor_neighbors: int = 12
    sor_std_ratio: float = 1.5
    min_frames: int = 2
    radius_m: float = .10
    min_neighbors: int = 3  # Excludes the point itself.

    def __post_init__(self):
        if (not np.isfinite([self.min_z_m, self.max_z_m, self.voxel_m,
                             self.sor_std_ratio, self.radius_m]).all()
                or not 0 < self.min_z_m < self.max_z_m or self.voxel_m <= 0
                or self.sor_neighbors < 2 or self.sor_std_ratio <= 0
                or self.min_frames < 2 or self.radius_m <= 0 or self.min_neighbors < 1):
            raise ValueError('Invalid camera filter settings')


def frame_voxels(camera, world, cfg):
    """Shared range/SOR/voxel stage for legacy maps and online slope support."""
    p, range_kept = range_sor(camera, world, cfg)
    keys, inverse = np.unique(np.floor(p / cfg.voxel_m).astype(np.int64), axis=0, return_inverse=True)
    n = np.bincount(inverse)
    means = np.column_stack([np.bincount(inverse, weights=p[:, j]) / n for j in range(3)])
    return keys, means, range_kept, len(p)


class FilteredDepth:
    def __init__(self, settings=None):
        self.settings = settings or FilterSettings()
        self.cells = {}
        self.last_stamp = None
        self.input_points = self.range_kept = self.sor_kept = self.frames = 0
        self._cache = None

    def add_frame(self, camera, world, stamp_ns):
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise ValueError('Each independent depth frame must have an increasing timestamp')
        keys, means, range_kept, sor_kept = frame_voxels(camera, world, self.settings)
        self.last_stamp = int(stamp_ns)
        self.frames += 1
        self.input_points += len(camera)
        self.range_kept += range_kept
        self.sor_kept += sor_kept
        if len(means):
            # A frame contributes exactly one centroid per voxel regardless of
            # its pixel density. It cannot confirm its own many noisy pixels.
            for key, mean in zip(map(tuple, keys), means):
                if key in self.cells:
                    self.cells[key][:3] += mean
                    self.cells[key][3] += 1
                else:
                    self.cells[key] = np.r_[mean, 1.]
        self._cache = None

    def arrays(self):
        if self._cache is not None:
            return self._cache
        if not self.cells:
            self._cache = (np.empty((0, 3)), np.empty(0, np.uint32))
            return self._cache
        values = np.array(list(self.cells.values()))
        values = values[values[:, 3] >= self.settings.min_frames]
        p = values[:, :3] / values[:, 3, None]
        n = values[:, 3].astype(np.uint32)
        if len(p):
            neighbors = cKDTree(p).query_ball_point(p, self.settings.radius_m, return_length=True) - 1
            good = neighbors >= self.settings.min_neighbors
            p, n = p[good], n[good]
        self._cache = p, n
        return self._cache

    def report(self):
        p, _ = self.arrays()
        return dict(enabled=True, algorithm='range + per-frame SOR + equal-frame voxel mean + repeated-frame/radius support',
                    settings=asdict(self.settings), frames=self.frames, input_points=self.input_points,
                    range_kept_points=self.range_kept, sor_kept_points=self.sor_kept,
                    candidate_voxels=len(self.cells), output_voxels=len(p),
                    observations_meaning='distinct acquisition frames contributing to this voxel',
                    accuracy_verified=False, pose_refinement=False, lidar_used_for_filter=False)


class OnlineDepthFilter:
    """Causal version of the same filter for capture-time slope observations.

    Retain at most ten seconds and use equal-frame voxel means to check repeated
    and spatial support. Emit THIS frame's own voxel centroids, not a smoothed
    archive with a new timestamp. The slope core performs its existing temporal
    XY averaging, so past measurements are not averaged and counted twice.
"""
    def __init__(self, settings=None, window_s=10., max_representatives=500_000):
        self.settings = settings or FilterSettings()
        if not np.isfinite(window_s) or window_s <= 0 or max_representatives < 1:
            raise ValueError('Invalid online camera retention limits')
        self.window_ns = round(window_s * 1e9)
        self.max_representatives = max_representatives
        self.history = deque()
        self.retained = self.frames = self.input_points = self.range_kept = self.sor_kept = 0
        self.last_stamp = None
        self.capacity_evictions = self.latest_points = self.confirmed_voxels = 0

    def add_frame(self, camera, world, stamp_ns):
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise ValueError('Each independent depth frame must have an increasing timestamp')
        keys, means, ranged, sor = frame_voxels(camera, world, self.settings)
        if len(means) > self.max_representatives:
            raise ValueError('One camera frame exceeds the online retention budget')
        self.last_stamp = int(stamp_ns)
        self.frames += 1
        self.input_points += len(camera)
        self.range_kept += ranged
        self.sor_kept += sor
        if len(means):
            self.history.append((int(stamp_ns), keys, means))
            self.retained += len(means)
        cutoff = stamp_ns - self.window_ns
        while self.history and (self.history[0][0] < cutoff or self.retained > self.max_representatives):
            old_stamp, old_keys, _ = self.history.popleft()
            if old_stamp >= cutoff:
                self.capacity_evictions += len(old_keys)
            self.retained -= len(old_keys)
        empty = dict(points_world_m=np.empty((0, 3)), voxel_keys=np.empty((0, 3), np.int64),
            support_frames=np.empty(0, np.uint32), support_first_stamp_ns=np.empty(0, np.int64),
            support_last_stamp_ns=np.empty(0, np.int64), stamp_ns=np.int64(stamp_ns))
        self.latest_points = self.confirmed_voxels = 0
        if not self.history or not len(keys):
            return empty
        all_keys = np.concatenate([k for _, k, _ in self.history])
        all_means = np.concatenate([p for _, _, p in self.history])
        stamps = np.concatenate([np.full(len(k), t, np.int64) for t, k, _ in self.history])
        unique, inverse, counts = np.unique(all_keys, axis=0, return_inverse=True, return_counts=True)
        averaged = np.column_stack([np.bincount(inverse, weights=all_means[:, j]) / counts for j in range(3)])
        first = np.full(len(unique), np.iinfo(np.int64).max, np.int64)
        last = np.full(len(unique), -1, np.int64)
        np.minimum.at(first, inverse, stamps)
        np.maximum.at(last, inverse, stamps)
        supported = counts >= self.settings.min_frames
        candidates = np.flatnonzero(supported)
        if len(candidates):
            neighbors = cKDTree(averaged[candidates]).query_ball_point(
                averaged[candidates], self.settings.radius_m, return_length=True) - 1
            supported[candidates] = neighbors >= self.settings.min_neighbors
        self.confirmed_voxels = int(supported.sum())
        # last==stamp is exact membership in this independent capture, since
        # every voxel has at most one centroid per frame and ordered stamps.
        current_indices = inverse[-len(keys):]
        keep = supported[current_indices]
        selected = current_indices[keep]
        self.latest_points = int(keep.sum())
        return dict(points_world_m=means[keep], voxel_keys=keys[keep],
            support_frames=counts[selected].astype(np.uint32), support_first_stamp_ns=first[selected],
            support_last_stamp_ns=last[selected], stamp_ns=np.int64(stamp_ns))

    def report(self):
        return dict(enabled=True, algorithm='range + SOR + per-frame voxel centroid + repeated-frame/radius support',
            settings=asdict(self.settings), window_s=self.window_ns / 1e9,
            max_retained_frame_representatives=self.max_representatives,
            frames=self.frames, input_points=self.input_points, range_kept_points=self.range_kept,
            sor_kept_points=self.sor_kept, retained_frames=len(self.history),
            retained_frame_representatives=self.retained, confirmed_voxels=self.confirmed_voxels,
            latest_output_points=self.latest_points, capacity_evictions=self.capacity_evictions,
            output_policy='current-frame centroids only; original capture stamp plus explicit support-time range; no cumulative-map replay',
            temporal_mean='recent equal-frame voxel means used for neighbor support; output positions use only current frame; subsequent temporal XY averaging remains in slope core',
            accuracy_verified=False, pose_refinement=False, lidar_used_for_filter=False)

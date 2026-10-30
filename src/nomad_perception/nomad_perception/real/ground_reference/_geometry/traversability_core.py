"""Bounded semantic / height / slope mapping independent of ROS.

The coordinates accepted here are metres in ``odom``.  Camera projection and
capture-time TF belong to the ROS boundary, not to this class.  Cell keys are
absolute grid indices ``floor(odom_xy / resolution)``; the output adapter owns
the finite OccupancyGrid origin and extent.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import Counter
import heapq
import math
from typing import Iterable, Mapping

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # A bounded grid-neighbour fallback is also available.
    cKDTree = None


GROUND_CLASS_IDS = frozenset((3, 5, 7, 9, 11, 21, 23, 24, 31))
EXCLUDED_CLASS_IDS = frozenset((0, 8, 53, 56))
CLASS_COSTS = np.full(64, 100.0, dtype=np.float32)
for _class_id, _cost in {
    2: 80, 3: 30, 5: 35, 7: 5, 9: 5, 11: 5, 17: 95,
    18: 45, 21: 5, 22: 90, 23: 5, 24: 25, 26: 90,
    30: 75, 31: 20, 50: 35, 51: 65, 52: 85, 59: 95, 62: 90,
}.items():
    CLASS_COSTS[_class_id] = _cost
for _class_id in EXCLUDED_CLASS_IDS:
    CLASS_COSTS[_class_id] = np.nan
CLASS_COSTS.flags.writeable = False
GROUND_MASK = np.isin(np.arange(64), list(GROUND_CLASS_IDS))
ELIGIBLE_MASK = np.isfinite(CLASS_COSTS)


@dataclass(frozen=True)
class CoreConfig:
    geometry_only: bool = False
    resolution_m: float = 0.2
    radius_m: float = 0.5
    slope_input: str = "raw_points"
    height_method: str = "median"
    plane_method: str = "tls"
    fusion_method: str = "weighted"
    semantic_weight: float = 0.3
    geometry_weight: float = 0.7
    semantic_confidence_alpha: float = 1.0
    geometry_confidence_alpha: float = 1.0
    semantic_min_points: int = 20
    height_min_ground_points: int = 10
    semantic_reference_points: int = 20
    raw_reference_points: int = 30
    cell_reference_count: int = 6
    raw_window_s: float = 10.0
    per_cell_max_points: int = 512
    global_max_points: int = 1_000_000
    near_tie_share_difference: float = 0.10
    space_reference_fraction: float = 0.10
    xy_rank_ratio: float = 1e-6
    tls_gap_ratio: float = 1e-8
    slope_limit_deg: float = 20.0
    sampling_seed: int = 20261008

    def __post_init__(self):
        if not isinstance(self.geometry_only,bool):raise ValueError("geometry_only must be bool")
        for name in ("resolution_m", "radius_m", "raw_window_s",
                     "space_reference_fraction", "xy_rank_ratio", "tls_gap_ratio",
                     "slope_limit_deg"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("semantic_min_points", "height_min_ground_points",
                     "semantic_reference_points", "raw_reference_points",
                     "cell_reference_count", "per_cell_max_points", "global_max_points"):
            value = getattr(self, name)
            if not isinstance(value, (int, np.integer)) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name, choices in {
            "slope_input": ("raw_points", "cell_heights"),
            "height_method": ("mean", "median"),
            "plane_method": ("ols", "tls"),
            "fusion_method": ("probabilistic", "weighted"),
        }.items():
            if getattr(self, name) not in choices:
                raise ValueError(f"{name} must be one of {choices}")
        if not 0 <= self.near_tie_share_difference <= 1:
            raise ValueError("near_tie_share_difference must be in [0, 1]")
        if not 0 < self.slope_limit_deg <= 90:
            raise ValueError("slope_limit_deg must be in (0, 90]")
        if (not math.isfinite(self.semantic_weight) or
                not math.isfinite(self.geometry_weight) or
                self.semantic_weight < 0 or self.geometry_weight < 0 or
                not math.isclose(self.semantic_weight + self.geometry_weight, 1., abs_tol=1e-9)):
            raise ValueError("fusion weights must be nonnegative and sum to one")
        if not isinstance(self.sampling_seed, (int, np.integer)) or self.sampling_seed < 0:
            raise ValueError("sampling_seed must be a nonnegative integer")
        for name in ("semantic_confidence_alpha", "geometry_confidence_alpha"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class PlaneFit:
    slope_deg: float
    normal: tuple[float, float, float]
    offset_m: float
    residual_m: float
    q_space: float


def fit_plane(points: np.ndarray, config: CoreConfig) -> PlaneFit | None:
    """Fit an identifiable plane with equal weights; return None if degenerate.

    OLS minimizes vertical residuals using an SVD least-squares solve. TLS
    minimizes perpendicular residuals. Both center and uniformly scale XYZ.
    ``residual_m`` is the solver's own RMS objective in physical metres.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("plane points must have shape (N, 3)")
    if len(points) < 3 or not np.isfinite(points).all():
        return None
    center = points.mean(axis=0)
    centered = (points - center) / config.radius_m
    xy = centered[:, :2]
    cov_xy = xy.T @ xy / len(xy)
    xy_eigenvalues = np.linalg.eigvalsh(cov_xy)
    # The specified singular-value ratio is equivalent to the square-root
    # covariance eigenvalue ratio, without decomposing an N by 2 matrix twice.
    if (xy_eigenvalues[1] <= 0 or xy_eigenvalues[0] <=
            config.xy_rank_ratio ** 2 * xy_eigenvalues[1]):
        return None
    q_space = float(np.clip(max(0., xy_eigenvalues[0]) /
                            config.space_reference_fraction, 0., 1.))
    if config.plane_method == "ols":
        coefficients, _, rank, _ = np.linalg.lstsq(xy, centered[:, 2], rcond=None)
        if rank != 2 or not np.isfinite(coefficients).all():
            return None
        normal = np.array((-coefficients[0], -coefficients[1], 1.), dtype=np.float64)
        residual = np.sqrt(np.mean((centered[:, 2] - xy @ coefficients) ** 2)) * config.radius_m
    else:
        covariance = centered.T @ centered / len(centered)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if (eigenvalues[2] <= 0 or
                eigenvalues[1] - eigenvalues[0] <= config.tls_gap_ratio * eigenvalues[2]):
            return None
        normal = eigenvectors[:, 0]
        # Evaluate the residual directly. sqrt(lambda_min) can amplify a few
        # ulps of eigensolver round-off on an exactly planar observation.
        residual = math.sqrt(float(np.mean((centered @ normal) ** 2))) * config.radius_m
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal *= -1
    slope = math.degrees(math.atan2(float(np.hypot(normal[0], normal[1])), abs(float(normal[2]))))
    return PlaneFit(float(slope), tuple(float(v) for v in normal),
                    float(-np.dot(normal, center)), float(residual), q_space)


def _fit_tls_batch(xyz, index_lists, config):
    """Equivalent TLS fits with bounded batched reductions and eigensolves.

    Anchoring each support set before summing keeps moment calculations stable
    at large world translations. This avoids constructing N separate tiny
    matrix decompositions and repeatedly walking long support arrays in Python.
    """
    sizes = np.asarray([len(indices) for indices in index_lists], dtype=np.intp)
    result = [None] * len(index_lists)
    usable = np.flatnonzero(sizes >= 3)
    if not len(usable):
        return result
    counts = sizes[usable]
    starts = np.r_[0, np.cumsum(counts)[:-1]]
    indices = np.concatenate([np.asarray(index_lists[i], dtype=np.intp) for i in usable])
    anchors = np.asarray(xyz[indices[starts]], dtype=np.float64)
    group = np.repeat(np.arange(len(usable)), counts)
    delta = (np.asarray(xyz[indices], dtype=np.float64) - anchors[group]) / config.radius_m
    mean = np.add.reduceat(delta, starts, axis=0) / counts[:, None]
    # Center before forming products rather than subtracting large scatter
    # terms; this also handles weak geometry without cancellation.
    centered = delta - mean[group]
    covariance = np.empty((len(usable), 3, 3), dtype=np.float64)
    for i in range(3):
        for j in range(i, 3):
            values = np.add.reduceat(centered[:, i] * centered[:, j], starts) / counts
            covariance[:, i, j] = values
            covariance[:, j, i] = values
    xy_eigenvalues = np.linalg.eigvalsh(covariance[:, :2, :2])
    eigenvalues, normals = np.linalg.eigh(covariance)
    normals = normals[:, :, 0]
    normals[normals[:, 2] < 0] *= -1
    valid = ((xy_eigenvalues[:, 1] > 0) &
             (xy_eigenvalues[:, 0] > config.xy_rank_ratio ** 2 * xy_eigenvalues[:, 1]) &
             (eigenvalues[:, 2] > 0) &
             (eigenvalues[:, 1] - eigenvalues[:, 0] > config.tls_gap_ratio * eigenvalues[:, 2]))
    residual_projection = np.einsum("ij,ij->i", centered, normals[group])
    residuals = np.sqrt(np.add.reduceat(residual_projection ** 2, starts) / counts) * config.radius_m
    q_space = np.clip(np.maximum(0., xy_eigenvalues[:, 0]) / config.space_reference_fraction, 0., 1.)
    slopes = np.degrees(np.arctan2(np.hypot(normals[:, 0], normals[:, 1]), np.abs(normals[:, 2])))
    centers = anchors + mean * config.radius_m
    offsets = -np.einsum("ij,ij->i", centers, normals)
    for j in np.flatnonzero(valid):
        result[int(usable[j])] = PlaneFit(float(slopes[j]), tuple(float(v) for v in normals[j]),
                                         float(offsets[j]), float(residuals[j]), float(q_space[j]))
    return result


def geometry_cost(slope_deg: float, slope_limit_deg: float = 20.) -> float:
    if not math.isfinite(slope_deg) or slope_deg < 0:
        raise ValueError("slope must be finite and nonnegative")
    if not math.isfinite(slope_limit_deg) or slope_limit_deg <= 0:
        raise ValueError("slope limit must be finite and positive")
    if slope_deg >= slope_limit_deg:
        return 100.
    return float(min(99, math.floor(5. + 95. * slope_deg / slope_limit_deg + .5)))


def fuse_costs(semantic: float | None, geometry: float | None, q_s: float,
               q_g: float, config: CoreConfig) -> tuple[int, float | None, float | None]:
    """Hard constraints precede confidence/fusion; ordinary rounding never blocks."""
    if semantic is None or geometry is None:
        return 100, None if semantic is None else semantic, None if geometry is None else geometry
    if (not np.isfinite((semantic, geometry, q_s, q_g)).all() or
            not 0 <= semantic <= 100 or not 0 <= geometry <= 100 or
            not 0 <= q_s <= 1 or not 0 <= q_g <= 1):
        raise ValueError("costs/q must be finite and within their declared range")
    if semantic >= 100 or geometry >= 100:
        return 100, semantic, geometry
    adjusted_s = min(99., semantic * (1. + config.semantic_confidence_alpha * (1. - q_s)))
    adjusted_g = min(99., geometry * (1. + config.geometry_confidence_alpha * (1. - q_g)))
    if config.fusion_method == "probabilistic":
        combined = adjusted_s + adjusted_g - adjusted_s * adjusted_g / 100.
    else:
        combined = config.semantic_weight * adjusted_s + config.geometry_weight * adjusted_g
    return min(99, max(0, math.floor(combined + .5))), float(adjusted_s), float(adjusted_g)


@dataclass(frozen=True)
class CellEstimate:
    final_cost: int = 100
    semantic_cost: float | None = None
    geometry_cost: float | None = None
    semantic_adjusted: float | None = None
    geometry_adjusted: float | None = None
    representative_class: int | None = None
    height: float | None = None
    height_mean: float | None = None
    height_median: float | None = None
    state: str = "observed_insufficient"
    semantic_valid: bool = False
    height_valid: bool = False
    slope_valid: bool = False
    slope_deg: float | None = None
    total_points: int = 0
    semantic_points: int = 0
    ground_points: int = 0
    semantic_support_points: int = 0
    height_support_points: int = 0
    plane_support: int = 0
    q_s: float = 0.
    q_g: float = 0.
    q_count: float = 0.
    q_space: float = 0.
    fit_residual_m: float | None = None
    semantic_reused: bool = False
    height_reused: bool = False
    slope_reused: bool = False
    observation_stamp_s: float | None = None
    semantic_stamp_s: float | None = None
    height_stamp_s: float | None = None
    slope_stamp_s: float | None = None
    geometry_stamp_s: float | None = None
    cost_stamp_s: float | None = None
    support_oldest_stamp_s: float | None = None
    support_newest_stamp_s: float | None = None
    normal: tuple[float, float, float] | None = None
    plane_offset_m: float | None = None


# Dense retained payload: 3x float32 XYZ + uint32 ID + float64 capture time = 24 B.
POINT_DTYPE = np.dtype([("xyz", "<f4", (3,)), ("class_id", "<u4"), ("stamp", "<f8")])


class _Cell:
    __slots__ = ("points", "version", "estimate", "last_observation", "semantic_hist",
                 "last_semantic", "last_height", "last_plane", "representative", "representative_stamp")

    def __init__(self):
        self.points = np.empty(0, dtype=POINT_DTYPE)
        self.version = 0
        self.estimate: CellEstimate | None = None
        self.last_observation: float | None = None
        self.semantic_hist: Mapping[int, int] = {}
        self.last_semantic = None
        self.last_height = None
        self.last_plane = None
        self.representative = None
        self.representative_stamp = None


class TraversabilityCore:
    """Sparse bounded observations with per-trial persistent usable estimates.

    Only cells actually observed get an estimate. Missing keys mean never
    observed, hence -1 at the output boundary. Dirty neighbourhoods are refit
    after observations or retention evictions; unchanged historical cells are
    never refit merely because a timer fires.
    """

    def __init__(self, config: CoreConfig | None = None):
        self.config = config or CoreConfig()
        self._cells: dict[tuple[int, int], _Cell] = {}
        self._oldest_heap: list[tuple[float, tuple[int, int], int]] = []
        self._dirty: set[tuple[int, int]] = set()
        self._summary_dirty: set[tuple[int, int]] = set()
        self._active_count = 0
        self._last_time: float | None = None
        self._last_capture: float | None = None
        self._eviction_serial = 0
        self.diagnostics = Counter()
        self.last_snapshot_fit_keys = frozenset()
        self.last_snapshot_new_fit_keys = frozenset()
        self.last_snapshot_reused_keys = frozenset()
        reach = math.ceil(self.config.radius_m / self.config.resolution_m + math.sqrt(2) / 2)
        # Any point in an altered cell can affect these cell-centred circles.
        self._offsets = [(i, j) for i in range(-reach, reach + 1)
                         for j in range(-reach, reach + 1)
                         if math.hypot(max(abs(i) - .5, 0), max(abs(j) - .5, 0)) *
                         self.config.resolution_m <= self.config.radius_m + 1e-12]

    @property
    def active_point_count(self) -> int:
        return self._active_count

    @property
    def observed_cell_count(self) -> int:
        return len(self._cells)

    @property
    def retained_payload_bytes(self) -> int:
        return self._active_count * POINT_DTYPE.itemsize

    def reset(self):
        self._cells.clear()
        self._oldest_heap.clear()
        self._dirty.clear()
        self._summary_dirty.clear()
        self._active_count = 0
        self._last_time = None
        self._last_capture = None
        self._eviction_serial = 0
        self.diagnostics.clear()
        self.last_snapshot_fit_keys = frozenset()
        self.last_snapshot_new_fit_keys = frozenset()
        self.last_snapshot_reused_keys = frozenset()

    def _advance_time(self, stamp_s: float, observation=False):
        if not math.isfinite(stamp_s):
            raise ValueError("capture time must be finite")
        previous = self._last_capture if observation else self._last_time
        if previous is not None and stamp_s < previous - 1e-9:
            self.reset()
            self.diagnostics["clock_backward_resets"] += 1
        if observation:
            self._last_capture = stamp_s
        self._last_time = stamp_s if self._last_time is None else max(stamp_s, self._last_time)
        self._expire(self._last_time)

    def _mark_changed(self, key):
        self._summary_dirty.add(key)
        x, y = key
        for dx, dy in self._offsets:
            neighbour = (x + dx, y + dy)
            if neighbour in self._cells:
                self._dirty.add(neighbour)

    def _set_points(self, key, points):
        cell = self._cells[key]
        self._active_count += len(points) - len(cell.points)
        cell.points = points
        cell.version += 1
        if len(points):
            heapq.heappush(self._oldest_heap, (float(points["stamp"][0]), key, cell.version))
        self._mark_changed(key)

    def _peek_oldest(self):
        while self._oldest_heap:
            stamp, key, version = self._oldest_heap[0]
            cell = self._cells[key]
            if cell.version != version or not len(cell.points):
                heapq.heappop(self._oldest_heap)
                continue
            return stamp, key
        return None

    def _compact_heap(self):
        active_cells = sum(bool(len(c.points)) for c in self._cells.values())
        if len(self._oldest_heap) > max(1024, 2 * active_cells):
            self._oldest_heap = [(float(c.points["stamp"][0]), k, c.version)
                                 for k, c in self._cells.items() if len(c.points)]
            heapq.heapify(self._oldest_heap)

    def _expire(self, now_s):
        cutoff = now_s - self.config.raw_window_s
        while (oldest := self._peek_oldest()) is not None and oldest[0] < cutoff:
            _, key = oldest
            points = self._cells[key].points
            retained = points[points["stamp"] >= cutoff].copy()
            self.diagnostics["time_evicted_points"] += len(points) - len(retained)
            self._set_points(key, retained)
        self._compact_heap()

    def _sample_indices(self, count, keep, key, stamp_s, serial=0):
        if keep >= count:
            return np.arange(count)
        # Explicit unsigned seed components: Python hash randomization and row
        # order do not change the seed between candidates/processes.
        stamp_bits = np.float64(stamp_s).view(np.uint64).item()
        seed = np.random.SeedSequence((int(self.config.sampling_seed),
                                      int(key[0]) & 0xffffffff,
                                      int(key[1]) & 0xffffffff,
                                      stamp_bits & 0xffffffff, stamp_bits >> 32, int(serial)))
        return np.sort(np.random.default_rng(seed).choice(count, size=keep, replace=False))

    def _keep_latest(self, points, keep, key):
        if len(points) <= keep:
            return points
        # Cut between capture groups. If a group straddles the cap, uniformly
        # sample that group's pixels, rather than keeping a last image row.
        cutoff_index = len(points) - keep
        stamp = float(points["stamp"][cutoff_index])
        first = int(np.searchsorted(points["stamp"], stamp, side="left"))
        last = int(np.searchsorted(points["stamp"], stamp, side="right"))
        newer = points[last:]
        within = self._sample_indices(last - first, keep - len(newer), key, stamp)
        return np.concatenate((points[first:last][within], newer))

    def _enforce_global_cap(self):
        while self._active_count > self.config.global_max_points:
            oldest = self._peek_oldest()
            if oldest is None:
                raise RuntimeError("retention accounting lost its oldest active group")
            stamp, key = oldest
            points = self._cells[key].points
            end = int(np.searchsorted(points["stamp"], stamp, side="right"))
            remove = min(end, self._active_count - self.config.global_max_points)
            keep = end - remove
            chosen = self._sample_indices(end, keep, key, stamp, self._eviction_serial)
            self._eviction_serial += 1
            retained = np.concatenate((points[:end][chosen], points[end:]))
            self.diagnostics["global_cap_evicted_points"] += remove
            self._set_points(key, retained)
        self._compact_heap()

    def ingest(self, xyz_odom, class_ids, stamp_s: float) -> set[tuple[int, int]]:
        xyz = np.asarray(xyz_odom)
        ids = np.asarray(class_ids)
        if xyz.ndim != 2 or xyz.shape[1] != 3 or ids.ndim != 1 or len(ids) != len(xyz):
            raise ValueError("XYZ must be (N,3), class_ids must be paired (N,)")
        # Do not truncate non-integral floating class IDs into valid classes.
        try:
            finite_xyz = np.isfinite(xyz).all(axis=1)
            valid_id = np.isfinite(ids) & (ids >= 0) & (ids < 64) & (ids == np.floor(ids))
        except TypeError as exc:
            raise ValueError("XYZ and class IDs must be numeric") from exc
        self._advance_time(float(stamp_s), observation=True)
        self.diagnostics["received_points"] += len(xyz)
        self.diagnostics["invalid_xyz_points"] += int(np.count_nonzero(~finite_xyz))
        self.diagnostics["invalid_class_points"] += int(np.count_nonzero(~valid_id))
        mask = finite_xyz & valid_id
        if self.config.geometry_only:
            # Semantic membership remains the input selector; no semantic cost
            # or nonground point competes for Stage1 ground retention limits.
            ground=np.zeros(len(ids),bool)
            ground[valid_id]=GROUND_MASK[np.asarray(ids[valid_id],dtype=np.intp)]
            mask &= ground
            self.diagnostics["geometry_only_excluded_nonground"] += int(np.count_nonzero(valid_id & ~ground))
        # Very large finite coordinates cannot be represented by int64 keys.
        scaled = np.asarray(xyz[mask], dtype=np.float64)[:, :2] / self.config.resolution_m
        safe = np.all((scaled >= -(2. ** 63)) & (scaled < 2. ** 63), axis=1)
        self.diagnostics["out_of_index_range_points"] += int(np.count_nonzero(~safe))
        xyz = np.asarray(xyz[mask][safe], dtype=np.float32)
        ids = np.asarray(ids[mask][safe], dtype=np.uint32)
        # Float32 overflow would otherwise silently poison retained statistics.
        representable = np.isfinite(xyz).all(axis=1)
        self.diagnostics["storage_overflow_points"] += int(np.count_nonzero(~representable))
        xyz, ids, scaled = xyz[representable], ids[representable], scaled[safe][representable]
        if not len(xyz):
            if self.config.geometry_only:self.diagnostics["ingested_frames"] += 1
            return set()
        keys = np.floor(scaled).astype(np.int64)
        order = np.lexsort((keys[:, 1], keys[:, 0]))
        sorted_keys = keys[order]
        boundaries = np.r_[0, np.flatnonzero(np.any(np.diff(sorted_keys, axis=0), axis=1)) + 1, len(order)]
        changed = set()
        for start, stop in zip(boundaries[:-1], boundaries[1:]):
            key = tuple(int(v) for v in sorted_keys[start])
            indices = order[start:stop]
            cell = self._cells.setdefault(key, _Cell())
            cell.last_observation = float(stamp_s)
            new_points = np.empty(len(indices), dtype=POINT_DTYPE)
            new_points["xyz"] = xyz[indices]
            new_points["class_id"] = ids[indices]
            new_points["stamp"] = stamp_s
            merged = np.concatenate((cell.points, new_points))
            retained = self._keep_latest(merged, self.config.per_cell_max_points, key)
            self.diagnostics["per_cell_cap_evicted_points"] += len(merged) - len(retained)
            self._set_points(key, retained)
            changed.add(key)
        # New target cells must also see old neighbours, even if none of those
        # neighbours were touched by this particular frame.
        self._dirty.update(changed)
        self._expire(self._last_time)
        self._enforce_global_cap()
        self.diagnostics["accepted_points"] += len(xyz)
        self.diagnostics["ingested_frames"] += 1
        return changed

    def _representative(self, histogram, total):
        maximum = max(histogram.values())
        candidates = [cls for cls, count in histogram.items()
                      if (maximum - count) / total <= self.config.near_tie_share_difference + 1e-12]
        non_ground = [cls for cls in candidates
                      if cls not in GROUND_CLASS_IDS and cls not in EXCLUDED_CLASS_IDS]
        if non_ground:
            candidates = non_ground
        return min(candidates, key=lambda cls: (
            -float(CLASS_COSTS[cls]) if cls not in EXCLUDED_CLASS_IDS else 1.,
            -histogram[cls], cls))

    def _update_summary(self, key):
        cell = self._cells[key]
        points = cell.points
        ids = points["class_id"]
        if not self.config.geometry_only:
            counts = np.bincount(ids.astype(np.intp), minlength=64)
            histogram = {int(c): int(counts[c]) for c in np.flatnonzero(counts)}
            cell.semantic_hist = dict(histogram)
            if len(points) >= self.config.semantic_min_points:
                cell.representative = self._representative(histogram, len(points))
                cell.representative_stamp = float(points["stamp"].max())
            eligible = ELIGIBLE_MASK[ids]
            if np.any(eligible):
                count = int(np.count_nonzero(eligible))
                cost = float(CLASS_COSTS[ids[eligible]].sum(dtype=np.float64) / count)
                cell.last_semantic = (cost, min(1., count / self.config.semantic_reference_points),
                                      count, float(points["stamp"][eligible].max()))
        ground = GROUND_MASK[ids]
        ground_count = int(np.count_nonzero(ground))
        if ground_count >= self.config.height_min_ground_points:
            z = points["xyz"][ground, 2].astype(np.float64)
            cell.last_height = (float(z.mean()), float(np.median(z)), ground_count,
                                float(points["stamp"][ground].max()))

    def _raw_index(self):
        pieces = [c.points[GROUND_MASK[c.points["class_id"]]]
                  for c in self._cells.values() if len(c.points)]
        pieces = [p for p in pieces if len(p)]
        points = np.concatenate(pieces) if pieces else np.empty(0, dtype=POINT_DTYPE)
        tree = cKDTree(points["xyz"][:, :2]) if cKDTree is not None and len(points) else None
        return points, tree

    def _height_index(self):
        keys, xyz, stamps = [], [], []
        for key, cell in self._cells.items():
            if cell.last_height is None:
                continue
            mean, median, _, stamp = cell.last_height
            xyz.append(((key[0] + .5) * self.config.resolution_m,
                        (key[1] + .5) * self.config.resolution_m,
                        mean if self.config.height_method == "mean" else median))
            keys.append(key)
            stamps.append(stamp)
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        stamps = np.asarray(stamps, dtype=np.float64)
        tree = cKDTree(xyz[:, :2]) if cKDTree is not None and len(xyz) else None
        return xyz, stamps, tree

    def _make_estimate(self, key, support_xyz, support_stamps, fitted_override=None, fit_supplied=False):
        cell = self._cells[key]
        points, ids = cell.points, cell.points["class_id"]
        semantic_count = 0 if self.config.geometry_only else int(np.count_nonzero(ELIGIBLE_MASK[ids]))
        ground_count = int(np.count_nonzero(GROUND_MASK[ids]))
        semantic_valid = cell.representative is not None
        height_valid = cell.last_height is not None
        state = ("both_valid" if semantic_valid and height_valid else
                 "semantic_only" if semantic_valid else "height_only" if height_valid else "observed_insufficient")
        height_mean = height_median = height_stamp = None
        height_support = 0
        if height_valid:
            height_mean, height_median, height_support, height_stamp = cell.last_height
        semantic_cost = semantic_stamp = None
        semantic_support, q_s = 0, 0.
        if cell.last_semantic is not None:
            semantic_cost, q_s, semantic_support, semantic_stamp = cell.last_semantic
        fitted = fitted_override if fit_supplied else fit_plane(support_xyz, self.config)
        plane_reused = False
        if fitted is not None:
            reference = (self.config.raw_reference_points if self.config.slope_input == "raw_points"
                         else self.config.cell_reference_count)
            q_count = min(1., len(support_xyz) / reference)
            q_g = min(q_count, fitted.q_space)
            cell.last_plane = (fitted, len(support_xyz), q_count, q_g,
                               float(support_stamps.min()), float(support_stamps.max()))
        elif cell.last_plane is not None:
            plane_reused = True
        slope_deg = geometry = residual = normal = offset = None
        support_count = 0
        q_count = q_g = q_space = 0.
        oldest = newest = slope_stamp = None
        if cell.last_plane is not None:
            fitted, support_count, q_count, q_g, oldest, newest = cell.last_plane
            slope_deg, residual, q_space = fitted.slope_deg, fitted.residual_m, fitted.q_space
            normal, offset = fitted.normal, fitted.offset_m
            geometry = geometry_cost(slope_deg, self.config.slope_limit_deg)
            slope_stamp = newest
        if self.config.geometry_only:
            adjusted_s=None
            adjusted_g=(100. if geometry is None or geometry>=100 else min(99.,geometry*(2.-q_g)))
            final=int(math.floor(adjusted_g+.5))
        else:
            final, adjusted_s, adjusted_g = fuse_costs(semantic_cost, geometry, q_s, q_g, self.config)
        # A hard decision depends only on its blocking reasons, not on an
        # unrelated old passable component. A fresh S=100 must remain visible
        # even when its retained G is older than the output TTL (and vice versa).
        # If more than one independently blocks, the freshest such evidence is
        # sufficient. Missing-component blocking is supported by the physical
        # observation that made this an observed rather than unknown cell.
        blocking_stamps = []
        if semantic_cost is not None and semantic_cost >= 100:
            blocking_stamps.append(semantic_stamp)
        if geometry is not None and geometry >= 100:
            blocking_stamps.append(slope_stamp)
        if semantic_cost is None or geometry is None:
            blocking_stamps.append(cell.last_observation)
        if blocking_stamps:
            cost_stamp = max(stamp for stamp in blocking_stamps if stamp is not None)
        elif semantic_stamp is not None and slope_stamp is not None:
            # Ordinary fusion needs both costs; preserve the older component's
            # capture age rather than refreshing it by recomposition.
            cost_stamp = min(semantic_stamp, slope_stamp)
        else:
            cost_stamp = cell.last_observation
        return CellEstimate(
            final_cost=final, semantic_cost=semantic_cost, geometry_cost=geometry,
            semantic_adjusted=adjusted_s, geometry_adjusted=adjusted_g,
            representative_class=cell.representative,
            height=height_mean if self.config.height_method == "mean" else height_median,
            height_mean=height_mean, height_median=height_median, state=state,
            semantic_valid=semantic_valid, height_valid=height_valid,
            slope_valid=cell.last_plane is not None, slope_deg=slope_deg,
            total_points=len(points), semantic_points=semantic_count, ground_points=ground_count,
            semantic_support_points=semantic_support, height_support_points=height_support,
            plane_support=support_count, q_s=q_s, q_g=q_g, q_count=q_count, q_space=q_space,
            fit_residual_m=residual, semantic_reused=semantic_cost is not None and semantic_count == 0,
            height_reused=height_valid and ground_count < self.config.height_min_ground_points,
            slope_reused=plane_reused, observation_stamp_s=cell.last_observation,
            semantic_stamp_s=semantic_stamp, height_stamp_s=height_stamp,
            slope_stamp_s=slope_stamp, geometry_stamp_s=slope_stamp, cost_stamp_s=cost_stamp,
            support_oldest_stamp_s=oldest, support_newest_stamp_s=newest,
            normal=normal, plane_offset_m=offset)

    def snapshot(self, now_s: float, targets: Iterable[tuple[int, int]] | None = None) -> dict[tuple[int, int], CellEstimate]:
        self._advance_time(float(now_s))
        self.last_snapshot_fit_keys = frozenset()
        self.last_snapshot_new_fit_keys = frozenset()
        self.last_snapshot_reused_keys = frozenset()
        new_fit_keys, reused_keys = set(), set()
        if targets is not None:
            requested = {tuple(int(v) for v in key) for key in targets}
        else:
            requested = None
        # All primitive summaries are brought current before any neighbour fit.
        # The order of target evaluation therefore cannot affect cell-mode fits.
        for key in self._summary_dirty:
            self._update_summary(key)
        self._summary_dirty.clear()
        dirty = self._dirty if requested is None else self._dirty & requested
        if dirty:
            if self.config.slope_input == "raw_points":
                raw, tree = self._raw_index()
                # One contiguous float64 conversion is cheaper than converting
                # each overlapping support array from a strided float32 view.
                xyz = np.asarray(raw["xyz"], dtype=np.float64, order="C")
                stamps = raw["stamp"]
            else:
                xyz, stamps, tree = self._height_index()
            keys = sorted(dirty)
            centers = (np.asarray(keys, dtype=np.float64) + .5) * self.config.resolution_m
            # Bound query-list and moment temporary allocations independently
            # of the number of historical cells in the complete trial.
            for chunk_start in range(0, len(keys), 16):
                chunk_keys = keys[chunk_start:chunk_start + 16]
                chunk_centers = centers[chunk_start:chunk_start + 16]
                if tree is not None:
                    neighbours = tree.query_ball_point(chunk_centers, self.config.radius_m + 1e-12,
                                                       return_sorted=False)
                else:
                    neighbours = [np.flatnonzero(np.sum((xyz[:, :2] - center) ** 2, axis=1) <=
                                                self.config.radius_m ** 2 + 1e-12) for center in chunk_centers]
                neighbours = [np.asarray(indices, dtype=np.intp) for indices in neighbours]
                if self.config.plane_method == "tls":
                    fitted_batch = _fit_tls_batch(xyz, neighbours, self.config)
                else:
                    fitted_batch = [fit_plane(xyz[indices], self.config) for indices in neighbours]
                for key, indices, fitted in zip(chunk_keys, neighbours, fitted_batch):
                    if fitted is not None:
                        new_fit_keys.add(key)
                    elif self._cells[key].last_plane is not None:
                        reused_keys.add(key)
                    self._cells[key].estimate = self._make_estimate(
                        key, xyz[indices], stamps[indices], fitted_override=fitted, fit_supplied=True)
            self.last_snapshot_fit_keys = frozenset(keys)
            self.last_snapshot_new_fit_keys = frozenset(new_fit_keys)
            self.last_snapshot_reused_keys = frozenset(reused_keys)
            self.diagnostics["plane_fit_attempts"] += len(keys)
            self._dirty.difference_update(dirty.copy())
        if requested is None:
            return {key: cell.estimate for key, cell in self._cells.items() if cell.estimate is not None}
        return {key: self._cells[key].estimate for key in requested
                if key in self._cells and self._cells[key].estimate is not None}

    def class_counts(self, key: tuple[int, int]) -> dict[int, int]:
        """Return active retained histogram, never removed historical counts."""
        cell = self._cells.get(key)
        if cell is None:
            return {}
        if key in self._summary_dirty:
            self._update_summary(key)
        return dict(cell.semantic_hist)

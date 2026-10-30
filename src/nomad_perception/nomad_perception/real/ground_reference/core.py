"""Timestamped 6-DoF scan projection and bounded voxel accumulation.

No ROS types are used here. Quaternions are always (x, y, z, w), points
are row vectors, and timestamps are integer nanoseconds in one clock domain.
"""

from bisect import bisect_left
from dataclasses import dataclass
import math

import numpy as np


class PoseUnavailableError(ValueError):
    """A scan timestamp has no exact pose or sufficiently close bracket."""


class ClockRollbackError(ValueError):
    """The observed simulation clock went backwards; the pose buffer reset."""


def _vector(values, length, name):
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain {length} finite values')
    return result


def normalize_quaternion(quaternion):
    quaternion = _vector(quaternion, 4, 'quaternion')
    scale = float(np.max(np.abs(quaternion)))
    if scale == 0:
        raise ValueError('quaternion has zero norm')
    # Scaling first avoids overflow/underflow for finite non-unit input.
    quaternion = quaternion / scale
    return quaternion / np.linalg.norm(quaternion)


def quaternion_slerp(first, second, fraction):
    """Interpolate the shortest unit-quaternion arc, including antipodal input."""
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError('interpolation fraction must be in [0, 1]')
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(first + fraction * (second - first))
    angle = math.acos(dot)
    return (math.sin((1.0 - fraction) * angle) * first
            + math.sin(fraction * angle) * second) / math.sin(angle)


def quaternion_matrix(quaternion):
    """Return the active 3x3 rotation from a child into its parent frame."""
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def _timestamp(value):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError('timestamp must be integer nanoseconds')
    return int(value)


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    position: tuple
    quaternion: tuple


class PoseBuffer:
    """Sorted bounded poses with exact matching or bracket interpolation only.

    ``add`` accepts reordered messages, replacing a duplicate timestamp. Clock
    rollback is a separate event: call ``observe_clock`` from a /clock callback
    and clear the map/session when it raises. Reordered odometry alone must not
    be mistaken for a simulation reset.
    """

    def __init__(self, history_seconds=30.0, max_gap_seconds=0.2, max_samples=10000):
        if not math.isfinite(history_seconds) or history_seconds <= 0:
            raise ValueError('history_seconds must be positive')
        if not math.isfinite(max_gap_seconds) or max_gap_seconds <= 0:
            raise ValueError('max_gap_seconds must be positive')
        if int(max_samples) != max_samples or max_samples < 2:
            raise ValueError('max_samples must be an integer >= 2')
        self.history_ns = round(history_seconds * 1e9)
        self.max_gap_ns = round(max_gap_seconds * 1e9)
        self.max_samples = int(max_samples)
        self._stamps = []
        self._poses = []
        self._clock_ns = None

    def __len__(self):
        return len(self._stamps)

    @property
    def earliest_stamp_ns(self):
        return self._stamps[0] if self._stamps else None

    @property
    def latest_stamp_ns(self):
        return self._stamps[-1] if self._stamps else None

    def clear(self):
        self._stamps.clear()
        self._poses.clear()
        self._clock_ns = None

    def observe_clock(self, stamp_ns):
        stamp_ns = _timestamp(stamp_ns)
        previous = self._clock_ns
        if previous is not None and stamp_ns < previous:
            self.clear()
            self._clock_ns = stamp_ns
            raise ClockRollbackError(f'clock moved backwards: {previous} -> {stamp_ns}')
        self._clock_ns = stamp_ns

    def add(self, stamp_ns, position, quaternion):
        stamp_ns = _timestamp(stamp_ns)
        position = tuple(_vector(position, 3, 'position'))
        quaternion = tuple(normalize_quaternion(quaternion))
        pose = PoseSample(stamp_ns, position, quaternion)
        index = bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            self._poses[index] = pose
        else:
            self._stamps.insert(index, stamp_ns)
            self._poses.insert(index, pose)
        cutoff = self._stamps[-1] - self.history_ns
        remove = max(bisect_left(self._stamps, cutoff), len(self._stamps) - self.max_samples)
        if remove > 0:
            del self._stamps[:remove]
            del self._poses[:remove]

    def lookup(self, stamp_ns):
        stamp_ns = _timestamp(stamp_ns)
        index = bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            return self._poses[index]
        if index == 0 or index == len(self._stamps):
            raise PoseUnavailableError(f'no pose bracket for {stamp_ns} ns')
        before, after = self._poses[index - 1:index + 1]
        gap = after.stamp_ns - before.stamp_ns
        if gap > self.max_gap_ns:
            raise PoseUnavailableError(f'pose bracket gap {gap / 1e9:.6f}s exceeds limit')
        fraction = (stamp_ns - before.stamp_ns) / gap
        position = ((1 - fraction) * np.asarray(before.position)
                    + fraction * np.asarray(after.position))
        quaternion = quaternion_slerp(before.quaternion, after.quaternion, fraction)
        return PoseSample(stamp_ns, tuple(position), tuple(quaternion))


def _points(points):
    result = np.asarray(points, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or not np.all(np.isfinite(result)):
        raise ValueError('points must be a finite Nx3 array')
    return result


def transform_points(points, translation, quaternion):
    """Apply parent<-child rotation and translation to child-frame points."""
    points = _points(points)
    return points @ quaternion_matrix(quaternion).T + _vector(translation, 3, 'translation')


def project_scan(ranges, angle_min, angle_increment, range_min, range_max):
    """Return valid scan-frame XYZ points and their original ray indices.

    Bounds are inclusive, as in the LaserScan message contract. Infinite/no-hit,
    NaN, negative and out-of-range measurements never become map points.
    """
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment):
        raise ValueError('scan angles must be finite')
    if not math.isfinite(range_min) or range_min < 0:
        raise ValueError('range_min must be finite and nonnegative')
    if math.isnan(range_max) or range_max < range_min:
        raise ValueError('range_max must be >= range_min')
    values = np.asarray(ranges, dtype=np.float64)
    if values.ndim != 1:
        raise ValueError('ranges must be one dimensional')
    indices = np.flatnonzero(np.isfinite(values) & (values >= range_min) & (values <= range_max))
    angles = angle_min + indices * angle_increment
    valid = values[indices]
    return np.column_stack((valid * np.cos(angles), valid * np.sin(angles),
                            np.zeros(len(indices)))), indices


def transform_scan(points_scan, ray_indices, stamp_ns, time_increment, poses,
                   translation, quaternion):
    """Apply base<-scan then timestamped world<-base transforms.

    LaserScan's header is the first ray time. A nonzero time_increment evaluates
    a different interpolated 6-DoF pose for every surviving ray. All required
    poses are checked before returning; an incomplete sweep is never partially
    accumulated. Gazebo's snapshot scans normally use time_increment=0.
    """
    stamp_ns = _timestamp(stamp_ns)
    if not math.isfinite(time_increment) or time_increment < 0:
        raise ValueError('time_increment must be finite and nonnegative')
    base_points = transform_points(points_scan, translation, quaternion)
    indices = np.asarray(ray_indices)
    if (indices.shape != (len(base_points),) or not np.issubdtype(indices.dtype, np.integer)
            or np.any(indices < 0)):
        raise ValueError('ray_indices must be nonnegative integers, one per point')
    if not len(base_points):
        return base_points
    if time_increment == 0:
        pose = poses.lookup(stamp_ns)
        return transform_points(base_points, pose.position, pose.quaternion)
    stamps = [stamp_ns + round(int(index) * time_increment * 1e9) for index in indices]
    samples = {time: poses.lookup(time) for time in set(stamps)}
    world_points = np.empty_like(base_points)
    for index, time in enumerate(stamps):
        pose = samples[time]
        world_points[index] = quaternion_matrix(pose.quaternion) @ base_points[index] + pose.position
    return world_points


@dataclass(frozen=True)
class VoxelSnapshot:
    points: np.ndarray
    keys: np.ndarray
    counts: np.ndarray
    voxel_size: float
    accepted_points: int
    dropped_points: int


class VoxelMap:
    """Centroid per floor(xyz / resolution), with an explicit capacity limit.

    Existing cells continue to accumulate after the capacity is reached. New
    cells are rejected and their observation count is added to dropped_points.
    Cells are never silently removed. Callers serialize access to this class.
    """

    def __init__(self, voxel_size=0.05, max_voxels=2000000):
        if not math.isfinite(voxel_size) or voxel_size <= 0:
            raise ValueError('voxel_size must be positive')
        if int(max_voxels) != max_voxels or max_voxels < 1:
            raise ValueError('max_voxels must be a positive integer')
        self.voxel_size = float(voxel_size)
        self.max_voxels = int(max_voxels)
        self._voxels = {}
        self.accepted_points = 0
        self.dropped_points = 0

    @property
    def voxel_count(self):
        return len(self._voxels)

    def clear(self):
        self._voxels.clear()
        self.accepted_points = 0
        self.dropped_points = 0

    def add(self, points):
        points = _points(points)
        if not len(points):
            return 0
        scaled = np.floor(points / self.voxel_size)
        # float64 cannot exactly represent the int64 upper boundary. Reject
        # extreme input before conversion instead of aliasing voxel identities.
        if np.any(np.abs(scaled) >= 2**62):
            raise ValueError('voxel coordinates exceed the supported integer range')
        keys, inverse, counts = np.unique(scaled.astype(np.int64), axis=0,
                                          return_inverse=True, return_counts=True)
        sums = np.column_stack([np.bincount(inverse, weights=points[:, axis],
                                            minlength=len(keys)) for axis in range(3)])
        accepted = 0
        for key_array, point_sum, count in zip(keys, sums, counts):
            key = tuple(int(value) for value in key_array)
            count = int(count)
            if key in self._voxels:
                old_sum, old_count = self._voxels[key]
                self._voxels[key] = (old_sum + point_sum, old_count + count)
            elif len(self._voxels) < self.max_voxels:
                self._voxels[key] = (point_sum, count)
            else:
                self.dropped_points += count
                continue
            accepted += count
        self.accepted_points += accepted
        return accepted

    def snapshot(self):
        keys = sorted(self._voxels)
        counts = np.array([self._voxels[key][1] for key in keys], dtype=np.uint64)
        points = np.array([self._voxels[key][0] / self._voxels[key][1] for key in keys],
                          dtype=np.float64).reshape(-1, 3)
        return VoxelSnapshot(points, np.asarray(keys, dtype=np.int64).reshape(-1, 3),
                             counts, self.voxel_size, self.accepted_points, self.dropped_points)

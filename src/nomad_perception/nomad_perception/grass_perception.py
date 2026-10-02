"""Conservative low-grass RGB-D baseline, without simulator object identities.

Green colour alone is insufficient: require nearby directly observed supporting
ground, a modest height above it, and repeated observations. This is not a
trained semantic classifier; tall/occluded or ambiguous vegetation stays solid.
"""
import itertools

import cv2
import numpy as np


def low_grass(points, bgr, hue=(28, 95), max_height=0.60):
    """Classify sampled RGB-D points; return grass flags and support-plane costs.

Fit only nearby non-green lower surface samples. Require good 2D coverage and
small residuals; never use a grass canopy itself as supporting ground.
"""
    points = np.asarray(points)
    result = np.zeros(len(points), dtype=bool)
    if not len(points):
        return result
    hsv = cv2.cvtColor(np.asarray(bgr, dtype=np.uint8).reshape(-1, 1, 3), cv2.COLOR_BGR2HSV)[:, 0]
    green = ((hsv[:, 0] >= hue[0]) & (hsv[:, 0] <= hue[1])
             & (hsv[:, 1] >= 60) & (hsv[:, 2] >= 35))
    bins = np.floor(points[:, :2]/0.4).astype(int)
    groups = {}
    for index, key in enumerate(map(tuple, bins)):
        groups.setdefault(key, []).append(index)
    nongreen = {key: np.asarray(indices)[~green[indices]] for key, indices in groups.items()}
    for key, indices in groups.items():
        target = np.asarray(indices)[green[indices]]
        if not len(target):
            continue
        neighbours = [nongreen[(key[0]+dx, key[1]+dy)]
                      for dx, dy in itertools.product(range(-2, 3), repeat=2)
                      if (key[0]+dx, key[1]+dy) in nongreen]
        ids = np.concatenate(neighbours)
        if len(ids) < 12:
            continue
        support = points[ids]
        # Lower envelope excludes trunks/rocks without relying on world geometry.
        support = support[support[:, 2] <= np.quantile(support[:, 2], .4)+.05]
        if len(support) < 12:
            continue
        centre = support.mean(axis=0)
        xy = support[:, :2]-centre[:2]
        if np.linalg.eigvalsh(xy.T @ xy/len(xy))[0] < .01:
            continue
        gradient = np.linalg.lstsq(xy, support[:, 2]-centre[2], rcond=None)[0]
        residual = support[:, 2]-centre[2]-xy @ gradient
        if np.sqrt(np.mean(residual**2)) > .025 or np.linalg.norm(gradient) > np.tan(.30):
            continue
        p = points[target]
        height = p[:, 2]-(centre[2]+(p[:, :2]-centre[:2]) @ gradient)
        # No far extrapolation of the supporting plane into unseen space.
        covered = np.all((p[:, :2] >= support[:, :2].min(axis=0)-.1)
                         & (p[:, :2] <= support[:, :2].max(axis=0)+.1), axis=1)
        result[target] = covered & (height >= .04) & (height <= max_height)
    return result


class GrassEvidence:
    """Short-lived, repeated 3D evidence; a solid observation vetoes grass.

Each voxel stores only observed points. Querying a ray endpoint never opens the
unobserved region behind grass, and a same-voxel solid return wins immediately.
"""
    def __init__(self, voxel=.10, ttl=3.0, confirmations=2):
        self.voxel, self.ttl, self.confirmations = voxel, ttl, confirmations
        self.records = {}
        self.last_stamp = -np.inf

    def update(self, points, grass, stamp):
        if stamp <= self.last_stamp:
            return np.zeros(len(points), dtype=bool)
        self.last_stamp = stamp
        self.records = {k: v for k, v in self.records.items() if 0 <= stamp-v[1] <= self.ttl}
        keys = np.floor(points/self.voxel).astype(int)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        totals = np.bincount(inverse)
        positives = np.bincount(inverse, weights=grass, minlength=len(unique))
        # Group once. Re-scanning the full depth cloud per voxel is quadratic
        # and blocks odometry/clock callbacks at the default two-pixel stride.
        means = np.column_stack([np.bincount(inverse, weights=points[:, axis],
                                            minlength=len(unique)) for axis in range(3)])
        means /= totals[:, None]
        confirmed = np.zeros(len(unique), dtype=bool)
        for i, key in enumerate(map(tuple, unique)):
            previous = self.records.get(key)
            candidate = totals[i] >= 3 and positives[i] == totals[i]
            count = (previous[2]+1 if previous else 1) if candidate else 0
            self.records[key] = (means[i], stamp, count)
            confirmed[i] = count >= self.confirmations
        return confirmed[inverse] & grass

    def matches(self, points, stamp):
        matches = np.zeros(len(points), dtype=bool)
        for i, point in enumerate(points):
            if not np.all(np.isfinite(point)):
                continue
            key = np.floor(point/self.voxel).astype(int)
            found, veto = False, False
            for offset in itertools.product((-1, 0, 1), repeat=3):
                item = self.records.get(tuple(key+offset))
                if item is None or not 0 <= stamp-item[1] <= self.ttl:
                    continue
                if np.linalg.norm(item[0]-point) > self.voxel:
                    continue
                found |= item[2] >= self.confirmations
                veto |= item[2] == 0
            matches[i] = found and not veto
        return matches

"""Depth-only local surface costs in a gravity-aligned planning frame.

These geometric thresholds are provisional, not vehicle capability estimates.
Missing depth is unknown; it is never interpolated into supporting ground.
"""
import numpy as np


def depth_points(depth, k, rotation, translation, stride=4, max_range=6.0):
    k = np.asarray(k, dtype=float).reshape(3, 3)
    if (depth.ndim != 2 or not np.all(np.isfinite(k))
            or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('Invalid depth calibration')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    good = np.isfinite(z) & (z >= 0.4) & (z <= max_range)
    z, u, v = z[good], u[good], v[good]
    optical = np.column_stack(((u-k[0, 2])*z/k[0, 0],
                               (v-k[1, 2])*z/k[1, 1], z))
    return optical @ rotation.T + translation


def surface_costs(points, resolution=0.2, patch_radius=0.35, min_samples=6,
                  slope_limit=0.35, roughness_limit=0.04, step_limit=0.18,
                  slope_enabled=True):
    """Return observed XY cell centres, costs and [slope, RMS, residual span].

Fit z=ax+by+c over a metric neighbourhood, then measure orthogonal residuals.
Only cells with enough direct samples are reported. Rank-deficient tall surfaces
are obstacles; rank-deficient flat/sparse samples remain unknown. The residual
span is a discontinuity proxy, not a measured traversable step or drop height.
"""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)]
    cells = np.floor(points[:, :2]/resolution).astype(np.int64)
    unique, inverse = np.unique(cells, axis=0, return_inverse=True)
    order = np.argsort(inverse, kind='stable')
    groups = np.split(order, np.cumsum(np.bincount(inverse))[:-1]) if len(points) else []
    lookup = {tuple(cell): group for cell, group in zip(unique, groups)}
    radius = int(np.ceil(patch_radius/resolution))
    centres, costs, metrics = [], [], []
    for cell, group in zip(unique, groups):
        if len(group) < min_samples:
            continue
        centre = (cell+0.5)*resolution
        neighbours = [lookup[(cell[0]+dx, cell[1]+dy)]
                      for dy in range(-radius, radius+1)
                      for dx in range(-radius, radius+1)
                      if (cell[0]+dx, cell[1]+dy) in lookup]
        patch = points[np.concatenate(neighbours)]
        patch = patch[np.linalg.norm(patch[:, :2]-centre, axis=1) <= patch_radius]
        if len(patch) < min_samples:
            continue
        # Bound per-cell work without stochastic changes between frames.
        if len(patch) > 256:
            patch = patch[np.linspace(0, len(patch)-1, 256, dtype=int)]
        xy = patch[:, :2]-patch[:, :2].mean(axis=0)
        z = patch[:, 2]-patch[:, 2].mean()
        eigenvalues = np.linalg.eigvalsh(xy.T @ xy/len(xy))
        if eigenvalues[0] < (resolution*0.1)**2:
            if np.ptp(z) < step_limit:
                continue
            slope, roughness, span, cost = np.pi/2, 0., float(np.ptp(z)), 100
        else:
            gradient = np.linalg.lstsq(xy, z, rcond=None)[0]
            slope = float(np.arctan(np.linalg.norm(gradient)))
            residual = (z-xy @ gradient)/np.sqrt(1+gradient @ gradient)
            roughness = float(np.sqrt(np.mean(residual**2)))
            span = float(np.ptp(residual))
            severity = max(slope/slope_limit if slope_enabled else 0.,
                           roughness/roughness_limit, span/step_limit)
            cost = 100 if severity >= 1 else int(round(79*severity))
        centres.append(centre)
        costs.append(cost)
        metrics.append((slope, roughness, span))
    return (np.asarray(centres).reshape(-1, 2), np.asarray(costs, dtype=np.int8),
            np.asarray(metrics).reshape(-1, 3))

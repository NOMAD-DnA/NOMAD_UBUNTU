"""Unmixed measured features; do not infer them from an inflated costmap."""
import math
import numpy as np
SEMANTIC = np.array([40,70,35,15,30,25,70,70,5,40], dtype=float) / 79.

def planning_layers(layers, arrays, slope_limit):
    if not math.isfinite(slope_limit) or slope_limit <= 0:
        raise ValueError('slope_limit must be positive')
    g = layers.views.grid
    slope = np.full((g.height,g.width), np.nan, dtype=np.float32)
    terrain = slope.copy()
    cache = getattr(layers, 'planning_feature_cache', {})
    # Same global memory policy as navigation costs, independent feature validity.
    for k, votes in layers.votes.items():
        if votes.sum() >= layers.min_points:
            previous = cache.get(k, (np.nan,np.nan))
            cache[k] = (previous[0], float(votes @ SEMANTIC / votes.sum()))
    for y,x in zip(*np.where(np.isfinite(arrays[1]))):
        k = (math.floor((g.origin_x+(x+.5)*g.resolution)/g.resolution),
             math.floor((g.origin_y+(y+.5)*g.resolution)/g.resolution))
        previous = cache.get(k, (np.nan,np.nan))
        cache[k] = (min(max(float(arrays[1][y,x]),0.)/slope_limit,1.)**2, previous[1])
    layers.planning_feature_cache = cache
    for k,(s,t) in cache.items():
        x,y = g.cell((k[0]+.5)*g.resolution,(k[1]+.5)*g.resolution)
        if 0<=x<g.width and 0<=y<g.height:
            slope[y,x],terrain[y,x] = s,t
    return slope,terrain

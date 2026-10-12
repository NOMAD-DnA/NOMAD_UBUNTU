"""Shared pre-inflation geometry grids for online and fixed-work execution."""
import math
import numpy as np


def geometry_cost(item):
    if item.geometry_cost is None or item.geometry_cost >= 100:
        return 100
    return min(99, int(math.floor(min(99., item.geometry_cost*(2.-item.q_g))+.5)))


def geometry_layers(estimates, grid):
    cost = np.full((grid.height, grid.width), -1, dtype=np.int8)
    angles = np.full(cost.shape, np.nan, dtype=np.float32)
    for (ix, iy), item in estimates.items():
        cx, cy = grid.cell((ix+.5)*grid.resolution, (iy+.5)*grid.resolution)
        if 0 <= cx < grid.width and 0 <= cy < grid.height:
            if item.slope_deg is not None:
                angles[cy, cx] = item.slope_deg
            cost[cy, cx] = geometry_cost(item)
    return cost, angles


def sparse_state(estimates, core, stamp_s, fit_calculation_stamps=None):
    """Read-back diagnostic state. Called after online publish timing is captured."""
    cells = np.asarray(sorted(estimates), dtype=np.int64).reshape(-1, 2)
    names = ('slope_deg', 'observation_stamp_s', 'slope_stamp_s', 'support_oldest_stamp_s',
             'support_newest_stamp_s', 'plane_support', 'q_g', 'q_count', 'q_space', 'fit_residual_m', 'plane_offset_m')
    data = {name:np.full(len(cells),np.nan) for name in names}
    normals=np.full((len(cells),3),np.nan)
    costs=np.full(len(cells),-1,np.int16)
    valid=np.zeros(len(cells),bool);new=np.zeros(len(cells),bool);failed=np.zeros(len(cells),bool)
    for i,key in enumerate(map(tuple,cells)):
        item=estimates[key]
        for name in names:
            value=getattr(item,name,None)
            if value is not None:data[name][i]=value
        if item.normal is not None:normals[i]=item.normal
        costs[i]=geometry_cost(item)
        valid[i]=bool(item.slope_valid and item.slope_deg is not None)
        new[i]=key in core.last_snapshot_new_fit_keys
        failed[i]=key in core.last_snapshot_fit_keys and not new[i]
    data['fit_calculation_stamp_s']=np.asarray([(fit_calculation_stamps or {}).get(tuple(c),np.nan) for c in cells])
    data['slope_deg']=data['slope_deg'].astype(np.float32)
    data.update(cells=cells,normal_xyz=normals,output_cost=costs,valid=valid,new_fit=new,reused=valid & ~new,
                fit_failed=failed,support_count=data.pop('plane_support'),stamp_s=float(stamp_s))
    return data

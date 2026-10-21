"""Remember direct RGBD support, separately from occupancy safety decisions."""
import math
import numpy as np


def support_grid(history, grid, stamp, hold_seconds=10., expire_seconds=30., evidence_window=3.):
    if not all(math.isfinite(v) for v in (stamp,hold_seconds,expire_seconds,evidence_window)) or not 0<=hold_seconds<expire_seconds or evidence_window<=0:
        raise ValueError('invalid observation support memory durations')
    out=np.zeros((grid.height,grid.width),dtype=np.int8)
    for (ix,iy),entries in history.items():
        # Anchor evidence to the LAST capture, not now. Otherwise old samples
        # disappear first and erase support even while the hold period is active.
        captures=sorted({t for t,_ in entries if math.isfinite(t) and t<=stamp+1e-6})
        if not captures:continue
        latest=captures[-1];age=max(0.,stamp-latest)
        if age>=expire_seconds:continue
        times=[t for t in captures if latest-t<=evidence_window+1e-6]
        cx=math.floor(((ix+.5)*grid.resolution-grid.origin_x)/grid.resolution)
        cy=math.floor(((iy+.5)*grid.resolution-grid.origin_y)/grid.resolution)
        if 0<=cx<grid.width and 0<=cy<grid.height:
            strength=min(len(times)/3.,1.)*min((times[-1]-times[0])/.5,1.)
            freshness=1. if age<=hold_seconds else (expire_seconds-age)/(expire_seconds-hold_seconds)
            out[cy,cx]=round(100*strength*freshness)
    return out

"""Direct RGBD cell observation support, NOT a free-volume certificate."""
import math
import numpy as np

def support_grid(history, grid, stamp):
    out=np.zeros((grid.height,grid.width),dtype=np.int8)
    for (ix,iy),entries in history.items():
        times=[t for t,_ in entries if 0<=stamp-t<=2.]
        if not times:continue
        cx=math.floor(((ix+.5)*grid.resolution-grid.origin_x)/grid.resolution)
        cy=math.floor(((iy+.5)*grid.resolution-grid.origin_y)/grid.resolution)
        if 0<=cx<grid.width and 0<=cy<grid.height:
            strength=min(len(times)/3.,1.)*min((times[-1]-times[0])/.5,1.)
            freshness=max(0.,1.-(stamp-times[-1])/2.)
            out[cy,cx]=round(100*strength*freshness)
    return out

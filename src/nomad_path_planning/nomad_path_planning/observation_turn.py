"""Penalize turns into weakly observed cells; never alter occupancy."""
import math
import numpy as np

def turn_weights(grid, support, start, weight):
    n=grid['width']*grid['height'];s=np.asarray(support,dtype=float)
    if s.shape!=(n,) or np.any(~np.isfinite(s)) or np.any((s<0)|(s>100)):
        raise ValueError('invalid observation support')
    if not math.isfinite(weight) or weight<0:raise ValueError('invalid turn weight')
    y,x=np.indices((grid['height'],grid['width']))
    dx=grid['origin_x']+(x.ravel()+.5)*grid['resolution']-start[0]
    dy=grid['origin_y']+(y.ravel()+.5)*grid['resolution']-start[1]
    angle=np.abs(np.arctan2(np.sin(np.arctan2(dy,dx)-start[2]),np.cos(np.arctan2(dy,dx)-start[2])))
    angular=np.clip((angle-math.radians(15))/math.radians(60),0.,1.)
    extra=weight*angular*(1.-s/100.)
    result=dict(grid);result['cell_weights']=(np.asarray(grid['cell_weights'])+extra).tolist()
    return result

def near_path_support(path,grid,support,minimum=.35):
    # Camera is level at z=.29m, x=.46m: near-ground footprint is blind.
    # Require repeated support in the visible forward segment; collision checks
    # still cover the complete path, including the blind near field.
    if not path:return False,0.
    distance=0.;values=[]
    for a,b in zip(path,path[1:]):
        distance+=math.dist(a[:2],b[:2])
        if distance<1.1:continue
        if distance>2.0:break
        ix=math.floor((b[0]-grid['origin_x'])/grid['resolution']);iy=math.floor((b[1]-grid['origin_y'])/grid['resolution'])
        values.append(support[iy*grid['width']+ix]/100. if 0<=ix<grid['width'] and 0<=iy<grid['height'] else 0.)
    if not values:return True,1. # Very short final approach retains ordinary collision checks.
    ratio=sum(v>=minimum for v in values)/len(values)
    return ratio>=.6,ratio

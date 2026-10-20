"""Soft cost for nearby unknown cells outside the current forward sector.

Uses only observed occupancy and the current fused pose, not world truth or a
camera visibility model. Values of zero mean no extra penalty, not proven safety.
"""
import math
import numpy as np


def unknown_direction_cost(grid, start, weight=0., radius=6., free_angle=math.pi/6):
    if (not all(math.isfinite(v) for v in (weight,radius,free_angle,*start))
            or weight<0 or radius<=0 or not 0<=free_angle<math.pi/2):
        raise ValueError('Invalid unknown-direction cost parameters')
    n=grid['width']*grid['height']
    if weight==0:return np.zeros(n,dtype=float)
    y,x=np.indices((grid['height'],grid['width']))
    dx=grid['origin_x']+(x+.5)*grid['resolution']-start[0]
    dy=grid['origin_y']+(y+.5)*grid['resolution']-start[1]
    distance=np.hypot(dx,dy)
    angle=np.abs(np.arctan2(np.sin(np.arctan2(dy,dx)-start[2]),
                           np.cos(np.arctan2(dy,dx)-start[2])))
    angular=np.clip((angle-free_angle)/(math.pi/2-free_angle),0.,1.)
    near=np.clip(1.-distance/radius,0.,1.)
    unknown=np.asarray(grid['data']).reshape(grid['height'],grid['width'])<0
    return (weight*unknown*angular*near).ravel()

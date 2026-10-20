"""J=sum ds*(1+wO*pO+wS*pS+wT*pT+wC*rho_R); occupancy is a hard constraint.

Missing measured features have a fixed explicit prior, not a fabricated label.
Unknown occupancy uses a fixed exploration surcharge above missing-feature cost.
Distance is from confirmed source-cell centres, NOT vehicle-body clearance.
"""
import math
import numpy as np
import cv2
from .proximity import proximity_costs

def cluster_density(grid, radius=4., influence_radius=0., saturation=1.):
    """Observed solid area / full disk area. Unknown is not free evidence.

    Only source cells ==100 count, never inflation (80..99). Fixed denominator
    keeps unknown and map-edge coverage from artificially boosting density.
    Evaluated on unknown too: known nearby obstacles remain relevant there.
    """
    if not math.isfinite(influence_radius) or influence_radius<0 or not math.isfinite(saturation) or not 0<saturation<=1:
        raise ValueError('Invalid influence radius/saturation')
    res=grid['resolution']
    if not math.isfinite(radius) or radius<=0 or not math.isfinite(res) or res<=0:
        raise ValueError('Invalid cluster radius/resolution')
    n=int(math.ceil(radius/res)); y,x=np.ogrid[-n:n+1,-n:n+1]
    kernel=((x*res)**2+(y*res)**2<=radius**2+1e-12).astype(np.float32)
    kernel/=kernel.sum()
    source=(np.asarray(grid['data']).reshape(grid['height'],grid['width'])==100).astype(np.float32)
    if influence_radius>0 and source.any():
        distance=cv2.distanceTransform((source==0).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*res
        source=(distance<=influence_radius+1e-7).astype(np.float32)
    density=cv2.filter2D(source,-1,kernel,borderType=cv2.BORDER_CONSTANT)
    return np.clip(density/saturation,0.,1.).ravel()


def compose(grid, slope, terrain, *, obstacle_weight=4., slope_weight=1.,
            terrain_weight=1., radius=2., missing_cost=1., unknown_surcharge=4.5,
            cluster_weight=0., cluster_radius=4., cluster_influence=0., cluster_saturation=1.):
    if not math.isfinite(cluster_weight) or cluster_weight<0 or not math.isfinite(cluster_radius) or cluster_radius<=0:
        raise ValueError('Invalid cluster parameters')
    if not math.isfinite(cluster_influence) or cluster_influence<0 or not math.isfinite(cluster_saturation) or not 0<cluster_saturation<=1:
        raise ValueError('Invalid cluster influence/saturation')
    values=(obstacle_weight,slope_weight,terrain_weight,radius,missing_cost,unknown_surcharge)
    if not all(math.isfinite(v) for v in values) or min(values[:3])<0 or radius<=0 or not 0<=missing_cost<=1 or unknown_surcharge<0:
        raise ValueError('Invalid multi-factor cost parameters')
    n=grid['width']*grid['height']
    data=np.asarray(grid['data']); s=np.asarray(slope,dtype=float);t=np.asarray(terrain,dtype=float)
    if any(a.shape!=(n,) for a in (data,s,t)) or np.any((data<-1)|(data>100)):
        raise ValueError('Cost layer size/value mismatch')
    for a in (s,t):
        if np.any(np.isinf(a)) or np.any(np.isfinite(a)&((a<0)|(a>1))):
            raise ValueError('Feature must be [0,1] or NaN')
    prox=np.asarray(proximity_costs(data,grid['width'],grid['height'],grid['resolution'],radius,1.))
    cost=1.+obstacle_weight*prox+slope_weight*np.where(np.isnan(s),missing_cost,s)+terrain_weight*np.where(np.isnan(t),missing_cost,t)
    # Unknown is neither a measured flat surface nor a free shortcut. Keep a
    # fixed surcharge, held constant across weight trials and reported separately.
    cost[data<0]=1.+(slope_weight+terrain_weight)*missing_cost+unknown_surcharge
    if cluster_weight:
        cost+=cluster_weight*cluster_density(grid,cluster_radius,cluster_influence,cluster_saturation)
    cost[data>=80]=np.inf
    return cost.tolist()

def validate_weights(data, values):
    weights=list(values)
    if len(weights)!=len(data):raise ValueError('Weight size mismatch')
    for v,w in zip(data,weights):
        if v>=80:
            if w!=math.inf:raise ValueError('Cannot soften blocked cells')
        elif not math.isfinite(w) or w<1:
            raise ValueError('Traversable weights must be finite and >=1')
    return weights

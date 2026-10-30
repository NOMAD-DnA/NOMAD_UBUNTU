"""Separate measured slope/semantic features for NOMAD dev, without ROS or TF.

User policy: 0 <= theta < 20 gives 4*theta points; theta >= 20 gives 100.
The float feature arrays divide points by 100. Occupancy has a separate hard
mask: ordinary values stay below 80, even when a soft feature approaches 80.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np
from .observation_support import support_metrics


@dataclass(frozen=True)
class CostPolicy:
    local_max_age_s: float = 30.
    obstacle_cost_radius_m: float = 1.2
    support_hold_s: float = 10.
    support_expire_s: float = 30.
    max_cells: int = 4_000_000

    def __post_init__(self):
        values=(self.local_max_age_s,self.obstacle_cost_radius_m,self.support_hold_s,self.support_expire_s)
        if (not all(math.isfinite(v) for v in values) or min(values[:2])<=0
                or not 0<=self.support_hold_s<self.support_expire_s
                or type(self.max_cells) is not int or self.max_cells<=0):
            raise ValueError('Invalid Planning cost output policy')


def slope_scores(degrees):
    """Keep fractional points. Invalid/negative angles are unknown, never flat."""
    angle=np.asarray(degrees,dtype=float)
    result=np.full(angle.shape,np.nan,dtype=np.float32)
    valid=np.isfinite(angle)&(angle>=0)
    result[valid]=np.where(angle[valid]>=20.,100.,4.*angle[valid])
    return result


def _integer(value, name):
    if isinstance(value,bool) or not isinstance(value,(int,np.integer)):
        raise ValueError(name+' must be an integer')
    return int(value)


def build_cost_snapshot(live, semantics, policy=None):
    """Convert one worker snapshot and semantic observations on the same XY grid.

    Frame names are preserved. This function cannot turn a raw-VIO world map
    into odom. Expired/missing evidence never becomes zero-cost free space.
    """
    policy=policy or CostPolicy()
    data=live['modes']['fused']; metadata=live['metadata']
    resolution=float(data['resolution_m'])
    if not math.isfinite(resolution) or resolution<=0:
        raise ValueError('Invalid cost grid resolution')
    if not math.isclose(resolution,float(semantics['resolution_m']),rel_tol=0,abs_tol=1e-9):
        raise ValueError('Slope and semantic resolutions differ')
    frame=metadata['frame_id']
    if not isinstance(frame,str) or not frame or semantics['frame']!=frame:
        raise ValueError('Slope and semantic frames differ; a measured transform is required')
    columns=data['columns']
    required=('ix','iy','slope_deg','last_observed_stamp_ns')
    if any(k not in columns for k in required):
        raise ValueError('Slope snapshot lacks direct observation timestamps')
    indices={k:columns.index(k) for k in required}
    slopes={}
    for row in data['cells']:
        key=(_integer(row[indices['ix']],'ix'),_integer(row[indices['iy']],'iy'))
        stamp=row[indices['last_observed_stamp_ns']]
        if stamp is None: continue
        stamp=_integer(stamp,'slope observation stamp')
        angle=row[indices['slope_deg']]
        slopes[key]=(float(angle) if angle is not None else math.nan,stamp)
    semantic_rows={(_integer(r['ix'],'ix'),_integer(r['iy'],'iy')):r for r in semantics['cells']}
    keys=set(slopes)|set(semantic_rows)
    if not keys:
        raise ValueError('No measured cells for cost output')
    xmin=min(k[0] for k in keys); xmax=max(k[0] for k in keys)
    ymin=min(k[1] for k in keys); ymax=max(k[1] for k in keys)
    shape=(ymax-ymin+1,xmax-xmin+1)
    if shape[0]*shape[1]>policy.max_cells:
        raise ValueError('Planning cost grid exceeds configured cell limit')
    angles=np.full(shape,np.nan); slope_stamp=np.zeros(shape,dtype=np.int64)
    semantic_score=np.full(shape,np.nan,dtype=np.float32)
    semantic_stamp=np.zeros(shape,dtype=np.int64)
    camera_stamp=np.zeros(shape,dtype=np.int64)
    for (x,y),(angle,stamp) in slopes.items():
        if stamp>0:
            angles[y-ymin,x-xmin]=angle; slope_stamp[y-ymin,x-xmin]=stamp
    for (x,y),row in semantic_rows.items():
        index=(y-ymin,x-xmin)
        observed=_integer(row.get('last_semantic_ns',0),'semantic observation stamp')
        camera_stamp[index]=max(0,_integer(row.get('camera_observed_ns',0),'camera observation stamp'))
        value=row['cost']
        if value is not None:
            if type(value) is not int or not 0<=value<=100:
                raise ValueError('Semantic cost must be a configured integer 0..100 or null')
            if observed>0:
                semantic_score[index]=value; semantic_stamp[index]=observed
    score=slope_scores(angles)
    score[slope_stamp<=0]=np.nan
    capture=max(int(slope_stamp.max(initial=0)),int(semantic_stamp.max(initial=0)),
                int(camera_stamp.max(initial=0)))
    if capture<=0:
        raise ValueError('Positive source capture time required')
    blocked=(score==100)|(semantic_score==100)
    known=np.isfinite(score)&np.isfinite(semantic_score)
    occupancy=np.full(shape,-1,dtype=np.int8)
    # Match dev's ordinary map blend for LPP. GPP gets the UNMIXED features.
    occupancy[known]=np.minimum(79,np.rint(79*(.7*score[known]/100.+.3*semantic_score[known]/100.))).astype(np.int8)
    occupancy[blocked]=100
    fresh=((capture-slope_stamp)/1e9<=policy.local_max_age_s)&((capture-semantic_stamp)/1e9<=policy.local_max_age_s)
    local=np.where(known&fresh,occupancy,-1).astype(np.int8)
    local[blocked]=100
    if blocked.any():
        distance=cv2.distanceTransform((~blocked).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
        proximity=np.rint(100*np.clip(1-(distance/policy.obstacle_cost_radius_m)**2,0,1)).astype(np.int8)
        occupancy=np.where(occupancy>=0,np.maximum(occupancy,proximity),occupancy).astype(np.int8)
        local=np.where(local>=0,np.maximum(local,proximity),local).astype(np.int8)
    support=np.zeros(shape,dtype=np.int8)
    camera_count=np.zeros(shape,dtype=np.int32)
    camera_span=np.zeros(shape,dtype=np.int64)
    for (x,y),row in semantic_rows.items():
        index=(y-ymin,x-xmin)
        # Legacy snapshots have at most one known capture, never invented repeats.
        captures=row.get('camera_capture_ns',[int(camera_stamp[index])] if camera_stamp[index]>0 else [])
        value,count,span=support_metrics(captures,capture,
            hold_s=policy.support_hold_s,expire_s=policy.support_expire_s)
        support[index]=value;camera_count[index]=count;camera_span[index]=span
    return dict(frame=frame,capture_ns=capture,resolution_m=resolution,
                origin_xy=np.array([xmin,ymin],dtype=float)*resolution,
                slope_score=score,semantic_score=semantic_score,
                slope_cost=(score/100.).astype(np.float32),terrain_cost=(semantic_score/100.).astype(np.float32),
                global_costmap=occupancy,local_costmap=local,camera_support=support,
                slope_observed_ns=slope_stamp,semantic_observed_ns=semantic_stamp,
                camera_observed_ns=camera_stamp,camera_capture_count=camera_count,
                camera_capture_span_ns=camera_span)


def ros_messages(snapshot):
    """Create the exact dev wire types; preserve the supplied frame and stamp."""
    from array import array
    from nav_msgs.msg import OccupancyGrid
    from nomad_interfaces.msg import PlanningCostLayers
    from .planning_transport import set_stamp
    shape=snapshot['global_costmap'].shape
    def grid(values):
        msg=OccupancyGrid();msg.header.frame_id=snapshot['frame']
        set_stamp(msg.header,snapshot['capture_ns'])
        msg.info.height,msg.info.width=shape
        msg.info.resolution=float(snapshot['resolution_m'])
        msg.info.origin.position.x,msg.info.origin.position.y=map(float,snapshot['origin_xy'])
        msg.info.origin.orientation.w=1.
        msg.data=array('b',np.asarray(values,dtype=np.int8).ravel().tolist())
        return msg
    global_map=grid(snapshot['global_costmap']);local_map=grid(snapshot['local_costmap'])
    layers=PlanningCostLayers();layers.grid=global_map
    layers.slope_cost=array('f',snapshot['slope_cost'].ravel())
    layers.terrain_cost=array('f',snapshot['terrain_cost'].ravel())
    return global_map,local_map,layers,grid(snapshot['camera_support'])

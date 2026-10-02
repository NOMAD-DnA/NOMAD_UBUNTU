"""Checks at externally replaceable module boundaries."""
import math


def valid_pose(pose):
    p,q=pose.position,pose.orientation
    return (all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
            and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<.02)


def valid_grid(msg,frame):
    i=msg.info
    q=i.origin.orientation
    return (msg.header.frame_id==frame and i.width>0 and i.height>0
            and math.isfinite(i.resolution) and i.resolution>0
            and i.width*i.height==len(msg.data)
            and all(math.isfinite(v) for v in (i.origin.position.x,i.origin.position.y,q.x,q.y,q.z,q.w))
            and abs(q.x)+abs(q.y)+abs(q.z)<1e-6 and abs(abs(q.w)-1.)<1e-6
            and min(msg.data)>=-1 and max(msg.data)<=100)

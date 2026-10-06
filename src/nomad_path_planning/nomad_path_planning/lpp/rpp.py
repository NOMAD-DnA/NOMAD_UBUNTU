"""Forward regulated pure pursuit in rear-axle coordinates.

Curvature, map cost and remaining distance regulate speed. The checked arc and
its speed/steering stay together in PlannedMotion. No local obstacle detour.
"""
import math
from .rollout import AckermannRollout
from nomad_path_planning.route_guard import route_is_behind
from nomad_path_planning.gpp.base import Grid,Request


class RegulatedPurePursuit(AckermannRollout):
    def __init__(self, lookahead=.8,lateral_accel=.35,**kwargs):
        super().__init__(**kwargs)
        if not all(math.isfinite(v) and v>0 for v in (lookahead,lateral_accel,self.speed,self.wheelbase,self.max_steer)):
            raise ValueError('RPP requires finite positive geometry, speed and regulation parameters')
        self.lookahead,self.lateral_accel=lookahead,lateral_accel

    def plan(self,start,global_path,costmap):
        failed={'success':False,'best':None,'candidates':[]}
        if not global_path or route_is_behind(start,global_path): return failed
        # Convert reference centerline to the same rear-axle reference used by
        # the bicycle model. Final point uses the last segment tangent.
        rear=[]
        for i,p in enumerate(global_path):
            a,b=(p,global_path[i+1]) if i+1<len(global_path) else (global_path[max(0,i-1)],p)
            yaw=math.atan2(b[1]-a[1],b[0]-a[0])
            rear.append((p[0]-self.reference_offset*math.cos(yaw),p[1]-self.reference_offset*math.sin(yaw)))
        yaw=start[2]
        rx,ry=start[0]-self.reference_offset*math.cos(yaw),start[1]-self.reference_offset*math.sin(yaw)
        nearest=self.nearest_path_index(rx,ry,rear)
        target=rear[-1]; remaining=self.lookahead
        for a,b in zip(rear[nearest:],rear[nearest+1:]):
            length=math.hypot(b[0]-a[0],b[1]-a[1])
            if length>=remaining and length>0:
                t=remaining/length; target=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])); break
            remaining-=length
        dx,dy=target[0]-rx,target[1]-ry
        forward=dx*math.cos(yaw)+dy*math.sin(yaw)
        lateral=-dx*math.sin(yaw)+dy*math.cos(yaw)
        if forward<=0 or dx*dx+dy*dy<1e-8: return failed
        curvature=2*lateral/(dx*dx+dy*dy)
        steer=math.atan(self.wheelbase*curvature)
        if abs(steer)>self.max_steer+1e-6: return failed
        distance=math.hypot(global_path[-1][0]-start[0],global_path[-1][1]-start[1])
        speed=min(self.speed,math.sqrt(self.lateral_accel/max(abs(curvature),1e-9)),max(.05,distance*.5))
        grid=Grid(Request(costmap,start,global_path[-1]))
        value=grid.value(*start[:2])
        if not math.isfinite(value): return failed
        speed=min(speed,self.speed/max(1.,value))
        model=AckermannRollout(wheelbase=self.wheelbase,max_steer=self.max_steer,
             speed=speed,dt=min(self.dt,costmap['resolution']*.25/max(speed,.01)),
             horizon=self.horizon,allow_unknown=self.allow_unknown,reference_offset=self.reference_offset)
        trajectory=model.simulate(start,steer,horizon=min(self.horizon,distance))
        score=model.terrain_cost(trajectory,costmap)
        if not math.isfinite(score) or not math.isfinite(grid.path_cost(trajectory)): return failed
        candidate=dict(trajectory=trajectory,steer=steer,speed=speed,score=score)
        return dict(success=True,best=candidate,candidates=[candidate])

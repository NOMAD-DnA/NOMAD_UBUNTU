"""Sensor-map-only local navigation targets. No world/route/oracle inputs."""
import heapq
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Candidate:
    point: tuple
    score: float
    distance: float
    clearance: float
    heading: float
    final: bool = False
    risk: float = 0.


class CorridorSelector:
    def __init__(self, horizon=8., inset=.7, arrival=1., seconds=.35,
                 max_expansions=20000, goal_weight=.4, length_weight=.15, narrow_weight=.8,
                 heading_weight=.5, visit_weight=.8, clearance_weight=.35,
                 risk_weight=0., risk_scale=50., bearing_weight=0., incremental=False, final_approach_radius=0.):
        if not all(math.isfinite(x) and x > 0 for x in (horizon,inset,arrival,seconds)):
            raise ValueError('corridor parameters must be positive and finite')
        self.horizon,self.inset,self.arrival,self.seconds=horizon,inset,arrival,seconds
        weights=(goal_weight,length_weight,narrow_weight,heading_weight,visit_weight,clearance_weight)
        if not all(math.isfinite(v) and v>=0 for v in weights):raise ValueError('invalid corridor weights')
        if not math.isfinite(risk_weight) or risk_weight<0 or not math.isfinite(risk_scale) or risk_scale<=0:
            raise ValueError("invalid corridor risk parameters")
        if not math.isfinite(bearing_weight) or bearing_weight<0:raise ValueError("invalid bearing weight")
        if not math.isfinite(final_approach_radius) or final_approach_radius<0:
            raise ValueError('final approach radius must be finite and nonnegative')
        self.final_approach_radius=final_approach_radius
        self.incremental=incremental
        self.bearing_weight=bearing_weight
        self.risk_weight,self.risk_scale=risk_weight,risk_scale
        self.weights=weights
        self.max_expansions=max_expansions
        self.reset()

    def reset(self):
        self.active=None
        self.active_path=None
        self.accepted_slope=None
        self.final_approach_goal=None
        self.visited=[]
        self.rejected=[]
        self.detail={}

    def reject(self, point, now):
        # A failed final approach is a stop/retry, never an exploration cooldown.
        if self.final_approach_goal is not None and tuple(point)==self.final_approach_goal:
            return
        self.rejected.append((point,now+3.))
        if self.active == point:self.active=None

    def accept(self, candidate, path=None):
        self.active=candidate.point
        self.active_path=path

    def path_slope(self, grid, path):
        values=grid.get('slope_cost')
        if values is None or not path:return None
        samples=[]
        for a,b in zip(path,path[1:]):
            count=max(1,math.ceil(math.dist(a[:2],b[:2])/(grid['resolution']*.25)))
            for j in range(count+1):
                t=j/count
                x=math.floor((a[0]+t*(b[0]-a[0])-grid['origin_x'])/grid['resolution'])
                y=math.floor((a[1]+t*(b[1]-a[1])-grid['origin_y'])/grid['resolution'])
                if 0<=x<grid['width'] and 0<=y<grid['height']:
                    v=float(values[y*grid['width']+x])
                    if math.isfinite(v):samples.append(v)
        return max(samples) if samples else None

    def invalidate_unreachable(self, grid, start, now, slope_increase=.2):
        """Invalidate target AND cached route before either retention fast path."""
        from .gpp.base import Grid, Request
        if self.active is None:return None
        view=Grid(Request(observed_grid(grid),start,self.active,tuple(grid.get('gates',()))))
        reason=None
        if not math.isfinite(view.value(*self.active)):
            reason='target_blocked_or_unknown'
        elif self.active_path:
            index=min(range(len(self.active_path)),key=lambda i:math.dist(start[:2],self.active_path[i][:2]))
            remaining=[start]+self.active_path[index:]
            if not math.isfinite(view.path_cost(remaining)):
                reason='remaining_path_blocked_or_unknown'
            else:
                slope=self.path_slope(grid,remaining)
                if slope is not None and self.accepted_slope is not None and slope-self.accepted_slope>=slope_increase:
                    reason='remaining_path_slope_increased'
        if reason:
            target=self.active
            self.reject(target,now)
            # Final mission remains locked, but its obsolete path cannot survive.
            self.active=None
            self.active_path=None
            self.accepted_slope=None
        return reason

    def retained_path(self, grid, start, goal):
        """Reuse only the accepted suffix; never construct a new steering segment.

        Current known/free cells, gates and pose alignment are rechecked. Camera
        repetition scores may decay; occupancy/unknown checks may not be skipped.
        LPP remains responsible for the actual connecting motion and swept checks.
        """
        from .gpp.base import Grid, Request
        path=self.active_path
        if self.active is None or not path or len(path)<2:return None
        final=math.dist(self.active,goal)<1e-6
        if self.final_approach_goal is not None and not final:return None
        if self.final_approach_radius>0 and math.dist(start[:2],goal)<=self.final_approach_radius and not final:return None
        if not final and math.dist(start[:2],self.active)<self.arrival:return None
        index=min(range(len(path)),key=lambda i:math.dist(start[:2],path[i][:2]))
        delta=abs(math.atan2(math.sin(path[index][2]-start[2]),math.cos(path[index][2]-start[2])))
        if math.dist(start[:2],path[index][:2])>.25 or delta>math.radians(15):return None
        remaining=path[index:]
        if len(remaining)<2:return None
        view=Grid(Request(observed_grid(grid),start,self.active,tuple(grid.get('gates',()))))
        if not math.isfinite(view.value(*self.active)):return None
        if not math.isfinite(view.path_cost([start]+remaining)):return None
        length=sum(math.dist(a[:2],b[:2]) for a,b in zip(remaining,remaining[1:]))
        return Candidate(self.active,0.,length,0.,delta,final),remaining

    def candidates(self, grid, start, goal, now):
        began=time.monotonic()
        goal=tuple(goal)
        if self.final_approach_goal is not None and self.final_approach_goal!=goal:
            self.reset()
        if self.final_approach_radius>0 and math.dist(start[:2],goal)<=self.final_approach_radius:
            self.final_approach_goal=goal
        w,h,r=grid['width'],grid['height'],grid['resolution']
        ox,oy=grid['origin_x'],grid['origin_y']
        data=np.asarray(grid['data']).reshape(h,w)
        known=(data>=0)&(data<80)
        risk=np.zeros((h,w),dtype=float)
        if self.risk_weight and "cell_weights" in grid:
            weights=np.asarray(grid["cell_weights"],dtype=float).reshape(h,w)
            if not np.all(np.isfinite(weights[known])) or np.any(weights[known]<1.):
                raise ValueError("invalid known-cell planning weights")
            risk[known]=np.clip((weights[known]-1.)/self.risk_scale,0.,4.)
        def cell(p):return (math.floor((p[0]-ox)/r),math.floor((p[1]-oy)/r))
        def world(c):return (ox+(c[0]+.5)*r,oy+(c[1]+.5)*r)
        def valid(c):return 0<=c[0]<w and 0<=c[1]<h and known[c[1],c[0]]
        s=cell(start);g=cell(goal)
        self.detail={'state':'searching','expanded':0}
        if not valid(s):
            self.detail['state']='start_not_observed_free';return []
        if self.final_approach_goal is not None:
            # Target-only proposal: the caller still checks a new kinematic path,
            # collision, unknown cells, observation support and input freshness.
            self.detail['final_approach']=True
            if not valid(g):
                self.detail['state']='final_goal_not_observed_free'
                return []
            self.detail['state']='final_goal_locked'
            return [Candidate(goal,0.,math.dist(start[:2],goal),0.,0.,True)]
        # Validate the remaining previously planned route before a full search.
        # A fresh Hybrid A* solution and observation guard are still required by
        # the caller; this only preserves the target, never republishes an old path.
        if self.incremental and self.active is not None and self.active_path and math.dist(start[:2],self.active)>=self.arrival:
            from .gpp.base import Grid,Request
            path=self.active_path
            index=min(range(len(path)),key=lambda i:math.dist(start[:2],path[i][:2]))
            delta=abs(math.atan2(math.sin(path[index][2]-start[2]),math.cos(path[index][2]-start[2])))
            remaining=[start]+path[index:]
            cx,cy=cell(self.active);radius=math.ceil((self.inset+r)/r)
            endpoint_clear=valid((cx,cy))
            if endpoint_clear and math.dist(self.active,goal)>.3:
                for yy in range(cy-radius,cy+radius+1):
                    for xx in range(cx-radius,cx+radius+1):
                        if math.hypot(xx-cx,yy-cy)*r<self.inset and not valid((xx,yy)):endpoint_clear=False
            if (endpoint_clear and math.dist(start[:2],path[index][:2])<=.5 and delta<=math.pi/6
                    and math.isfinite(Grid(Request(observed_grid(grid),start,self.active)).path_cost(remaining))):
                distance=sum(math.dist(a[:2],b[:2]) for a,b in zip(remaining,remaining[1:]))
                c=Candidate(self.active,-math.inf,distance,self.inset,delta,math.dist(self.active,goal)<.3)
                self.detail=dict(state='active_path_revalidated',expanded=0,elapsed_s=time.monotonic()-began)
                return [c]
        # Padded exterior and unknown both count as unavailable corridor width.
        clearance=cv2.distanceTransform(np.pad(known.astype(np.uint8),1),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*r
        unknown_distance=cv2.distanceTransform(np.pad((data>=0).astype(np.uint8),1),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*r
        if self.active is not None and math.dist(start[:2],self.active)<self.arrival and math.dist(self.active,goal)>.3:
            self.visited.append(self.active);self.visited=self.visited[-100:];self.active=None
        self.rejected=[(p,t) for p,t in self.rejected if t>now]
        # Dijkstra uses soft narrowness preference; hard occupancy unchanged.
        costs={s:0.};lengths={s:0.};risks={s:0.};first={s:None};queue=[(0.,s)]
        finalized=set();options=[];exhausted=False
        while queue:
            if len(finalized)>=self.max_expansions or time.monotonic()-began>self.seconds:
                exhausted=True
                if not self.incremental:
                    self.detail.update(state='selection_budget_exhausted',expanded=len(finalized));return []
                break
            cost,c=heapq.heappop(queue)
            if c in finalized:continue
            finalized.add(c)
            x,y=c;distance=lengths[c];p=world(c)
            if c!=s and (distance>=2. or c==g):
                a=first[c] or p
                heading=abs(math.atan2(math.sin(math.atan2(a[1]-start[1],a[0]-start[0])-start[2]),math.cos(math.atan2(a[1]-start[1],a[0]-start[0])-start[2])))
                clear=float(clearance[y,x])
                final=c==g
                frontier=self.inset<=unknown_distance[y,x]<=self.inset+1.2
                horizon_edge=distance>=self.horizon-1.
                # No immediate rearward targets; actual kinematics checked by GPP.
                if heading<=math.pi/2 and (final or (clear>=self.inset and (frontier or horizon_edge))):
                    if not any(math.dist(p,q)<1.5 for q,_ in self.rejected):
                        visit=sum(math.exp(-math.dist(p,q)**2/4.) for q in self.visited)
                        # Dimensionless: goal improvement/horizon, path/horizon,
                        # narrowness/path, heading/pi, bounded endpoint clearance.
                        wg,wl,wn,wh,wv,wc=self.weights
                        bearing=abs(math.atan2(math.sin(math.atan2(p[1]-start[1],p[0]-start[0])-start[2]),math.cos(math.atan2(p[1]-start[1],p[0]-start[0])-start[2])))
                        score=(self.bearing_weight*bearing/math.pi+wg*(math.dist(p,goal)-math.dist(start[:2],goal))/self.horizon
                               +wl*distance/self.horizon+wn*((cost-self.risk_weight*risks[c])/max(distance,r)-1.)
                               +self.risk_weight*risks[c]/max(distance,r)
                               +wh*heading/math.pi+wv*visit-wc*min(clear/2.,1.))
                        options.append(Candidate(tuple(goal) if final else p,score,distance,clear,heading,final,risks[c]/max(distance,r)))
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                n=(x+dx,y+dy)
                if not valid(n) or (dx and dy and (not valid((x+dx,y)) or not valid((x,y+dy)))):continue
                # Do not route through recovery gates, even in candidate ranking.
                if any(gate.blocks(world(c),world(n)) for gate in grid.get('gates',())):continue
                ds=r*math.hypot(dx,dy);length=distance+ds
                if length>self.horizon+.01:continue
                risk_edge=ds*.5*(risk[y,x]+risk[n[1],n[0]])
                nc=cost+ds*(1.+max(0.,1.-float(clearance[n[1],n[0]])/1.5)**2*3.)+self.risk_weight*risk_edge
                if nc<costs.get(n,math.inf):
                    costs[n]=nc;lengths[n]=length;risks[n]=risks[c]+risk_edge
                    first[n]=first[c] if distance>=.8 else world(n)
                    heapq.heappush(queue,(nc,n))
        # Preserve a still reachable selected target to prevent left/right flicker.
        if self.active is not None and valid(cell(self.active)) and cell(self.active) in finalized:
            c=cell(self.active)
            if clearance[c[1],c[0]]>=self.inset or math.dist(self.active,goal)<.3:
                active=Candidate(self.active,-math.inf,lengths[c],float(clearance[c[1],c[0]]),0.,math.dist(self.active,goal)<.3)
                options.insert(0,active)
        options.sort(key=lambda c:(not c.final,c.score))
        selected=[]
        for c in options:
            if all(math.dist(c.point,q.point)>1.5 for q in selected):selected.append(c)
            if len(selected)>=6:break
        state=('partial_candidates' if selected else 'selection_budget_exhausted') if exhausted else ('candidates' if selected else ('candidate_retry_cooldown' if self.rejected else 'no_observed_corridor'))
        self.detail=dict(state=state,rejected_count=len(self.rejected),expanded=len(finalized),elapsed_s=time.monotonic()-began,
                         candidates=[dict(point=c.point,score=c.score if math.isfinite(c.score) else None,clearance=c.clearance,distance=c.distance,final=c.final,risk=c.risk) for c in selected])
        return selected


def observed_grid(grid):
    """A private GPP view: unknown is forbidden; original perception untouched."""
    g=dict(grid)
    g['data']=[100 if v<0 else v for v in grid['data']]
    if 'cell_weights' in grid:
        g['cell_weights']=[math.inf if v<0 else c for v,c in zip(grid['data'],grid['cell_weights'])]
    return g


def short_path_length(path, approach_distance, horizon):
    """Reject loops/detours inconsistent with a local observation target."""
    length=sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:]))
    limit=min(1.5*horizon,1.8*approach_distance+1.)
    return length, bool(path) and math.isfinite(length) and length<=limit


def clearance_grid(grid, margin):
    """Private planning margin from already-blocked cell centres, in metres.

    Original unknown stays unknown; this is not a perception reclassification
    or a continuous vehicle-footprint guarantee. A start inside the added margin
    is deliberately not made free.
    """
    if not math.isfinite(margin) or margin<0:raise ValueError('invalid planning clearance')
    if not margin:return grid
    data=np.asarray(grid['data']).reshape(grid['height'],grid['width'])
    blocked=data>=80
    if not blocked.any():return grid
    distance=cv2.distanceTransform((~blocked).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*grid['resolution']
    added=(data>=0)&(distance<=margin+1e-7)
    copy=dict(grid);values=data.copy();values[added]=np.maximum(values[added],80)
    copy['data']=values.ravel().tolist()
    if 'cell_weights' in grid:
        weights=np.asarray(grid['cell_weights'],dtype=float).reshape(data.shape).copy()
        weights[added]=np.inf;copy['cell_weights']=weights.ravel().tolist()
    return copy

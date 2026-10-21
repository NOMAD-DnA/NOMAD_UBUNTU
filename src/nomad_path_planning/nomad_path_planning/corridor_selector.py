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
                 risk_weight=0., risk_scale=50., bearing_weight=0., incremental=False, final_approach_radius=0., continuation_distance=0., reselect_new_blocked=False, reselect_half_width=1.5, reselect_lookahead=3., observed_goal_lock=False):
        if not all(math.isfinite(x) and x > 0 for x in (horizon,inset,arrival,seconds)):
            raise ValueError('corridor parameters must be positive and finite')
        self.observed_goal_lock=observed_goal_lock
        self.horizon,self.inset,self.arrival,self.seconds=horizon,inset,arrival,seconds
        weights=(goal_weight,length_weight,narrow_weight,heading_weight,visit_weight,clearance_weight)
        if not all(math.isfinite(v) and v>=0 for v in weights):raise ValueError('invalid corridor weights')
        if not math.isfinite(risk_weight) or risk_weight<0 or not math.isfinite(risk_scale) or risk_scale<=0:
            raise ValueError("invalid corridor risk parameters")
        if not math.isfinite(bearing_weight) or bearing_weight<0:raise ValueError("invalid bearing weight")
        if not math.isfinite(final_approach_radius) or final_approach_radius<0:
            raise ValueError('final approach radius must be finite and nonnegative')
        if not math.isfinite(continuation_distance) or continuation_distance<0:
            raise ValueError("invalid continuation distance")
        if not all(math.isfinite(v) and v>0 for v in (reselect_half_width,reselect_lookahead)):
            raise ValueError("invalid reselection region")
        self.reselect_new_blocked=reselect_new_blocked
        self.reselect_half_width=reselect_half_width
        self.reselect_lookahead=reselect_lookahead
        self.continuation_distance=continuation_distance
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
        self.last_goal_probe=-math.inf
        self.previous_blocked=None
        self.previous_resolution=None

    def reject(self, point, now):
        # A failed final approach is a stop/retry, never an exploration cooldown.
        if self.final_approach_goal is not None and tuple(point)==self.final_approach_goal:
            return
        self.rejected.append((point,now+3.))
        if self.active == point:self.active=None

    def accept(self, candidate, path=None):
        # Caller accepts only after path, observation and freshness validation.
        if self.observed_goal_lock and candidate.final and path:
            self.final_approach_goal=tuple(candidate.point)
        self.active=candidate.point
        self.active_path=path

    def observed_goal_candidate(self, grid, start, goal, now):
        """Propose a known-connected goal; this does NOT lock it yet.

        A bounded 4-connected flood is conservative at diagonal corners. Actual
        vehicle kinematics, gates, support and freshness are checked by caller.
        """
        if not self.observed_goal_lock or self.final_approach_goal is not None:return None
        if now-self.last_goal_probe<2.:return None
        self.last_goal_probe=now
        w,h,r=grid['width'],grid['height'],grid['resolution']
        def cell(p):return (math.floor((p[0]-grid['origin_x'])/r),math.floor((p[1]-grid['origin_y'])/r))
        a=grid['data'];s=cell(start);g=cell(goal)
        def free(c):return 0<=c[0]<w and 0<=c[1]<h and 0<=a[c[1]*w+c[0]]<80
        if not free(s) or not free(g):return None
        queue=[s];seen={s};deadline=time.monotonic()+.03
        for x,y in queue:
            if time.monotonic()>deadline or len(seen)>20000:return None
            if (x,y)==g:return Candidate(tuple(goal),0.,math.dist(start[:2],goal),0.,0.,True)
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                c=(x+dx,y+dy)
                if c not in seen and free(c):seen.add(c);queue.append(c)
        return None

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

    def continuation_blocked(self, grid, start, point, goal):
        """Conservative dead-end veto, not a collision-free/kinematic route proof.

        Flood known free cells ahead of the target. Reachable unknown or map edge
        is inconclusive; it is never supplied to the actual planner as free.
        Exhausted compute budget is also inconclusive, not evidence of a wall.
        """
        radius=self.continuation_distance
        if radius<=0 or math.dist(point,goal)<=max(radius,self.final_approach_radius):return False
        dx,dy=point[0]-start[0],point[1]-start[1]
        norm=math.hypot(dx,dy)
        if norm<.1:return False
        dx,dy=dx/norm,dy/norm
        r=grid['resolution'];w,h=grid['width'],grid['height']
        ox,oy=grid['origin_x'],grid['origin_y'];data=grid['data']
        sx,sy=math.floor((point[0]-ox)/r),math.floor((point[1]-oy)/r)
        def value(x,y):
            return data[y*w+x] if 0<=x<w and 0<=y<h else -1
        if not 0<=value(sx,sy)<80:return False  # Existing endpoint check owns this.
        deadline=time.monotonic()+.02
        queue=[(sx,sy)];seen={(sx,sy)}
        for x,y in queue:
            if len(seen)>5000 or time.monotonic()>deadline:return False
            px,py=(x-sx)*r,(y-sy)*r
            if px*dx+py*dy>=radius*.5:return False
            for ax,ay in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nx,ny=x+ax,y+ay
                if (nx,ny) in seen:continue
                vx,vy=(nx-sx)*r,(ny-sy)*r
                if math.hypot(vx,vy)>radius or vx*dx+vy*dy<-.3:continue
                if value(nx,ny)>=80:continue
                if ax and ay and (value(x+ax,y)>=80 or value(x,y+ay)>=80):continue
                # Reachable unknown means there is insufficient evidence to veto.
                if value(nx,ny)<0:return False
                seen.add((nx,ny));queue.append((nx,ny))
        return True

    def new_direction_block(self, grid, start, goal):
        """Detect new blocked world cells, not repeated publications of a wall.

        This only removes retention priority; it never rejects a still-valid
        target or changes costs. Ordinary candidate ranking may choose it again.
        """
        if not self.reselect_new_blocked:return False
        r=grid['resolution']
        data=np.asarray(grid['data']).reshape(grid['height'],grid['width'])
        yy,xx=np.where(data>=80)
        xs=grid['origin_x']+(xx+.5)*r;ys=grid['origin_y']+(yy+.5)*r
        keys=list(zip(np.floor(xs/r+1e-7).astype(int),np.floor(ys/r+1e-7).astype(int)))
        current=set(keys);previous=self.previous_blocked
        self.previous_blocked=current
        same_resolution=self.previous_resolution==r
        self.previous_resolution=r
        if previous is None or not same_resolution or self.active is None or goal is None:return False
        if self.final_approach_goal is not None or math.dist(self.active,goal)<.3:return False
        if self.final_approach_radius>0 and math.dist(start[:2],goal)<=self.final_approach_radius:return False
        dx,dy=self.active[0]-start[0],self.active[1]-start[1]
        length=math.hypot(dx,dy)
        if length<.1:return False
        dx,dy=dx/length,dy/length
        for key,x,y in zip(keys,xs,ys):
            if key in previous:continue
            vx,vy=x-start[0],y-start[1]
            along=vx*dx+vy*dy;across=abs(vx*dy-vy*dx)
            if 0<=along<=length+self.reselect_lookahead and across<=self.reselect_half_width:
                return True
        return False

    def invalidate_unreachable(self, grid, start, now, slope_increase=.2, goal=None):
        """Invalidate target AND cached route before either retention fast path."""
        from .gpp.base import Grid, Request
        changed=self.new_direction_block(grid,start,goal)
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
        if reason is None and goal is not None and self.continuation_blocked(grid,start,self.active,goal):
            reason="target_continuation_blocked"
        if reason:
            target=self.active
            self.reject(target,now)
            # Final mission remains locked, but its obsolete path cannot survive.
            self.active=None
            self.active_path=None
            self.accepted_slope=None
        elif changed:
            # No reject()/cooldown: the old location competes at ordinary cost.
            self.active=None
            self.active_path=None
            self.accepted_slope=None
            reason='new_blocked_in_target_direction_reselect'
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
        if self.continuation_blocked(grid,start,self.active,goal):return None
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
        if self.active is not None and self.continuation_blocked(grid,start,self.active,goal):
            self.reject(self.active,now)
            self.active=None;self.active_path=None;self.accepted_slope=None
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
        continuation_rejected=0
        continuation_deadline=time.monotonic()+.12
        for c in options:
            if self.continuation_distance>0 and time.monotonic()>continuation_deadline:
                exhausted=True
                break
            if self.continuation_blocked(grid,start,c.point,goal):
                continuation_rejected+=1
                continue
            if all(math.dist(c.point,q.point)>1.5 for q in selected):selected.append(c)
            if len(selected)>=6:break
        state=('partial_candidates' if selected else 'selection_budget_exhausted') if exhausted else ('candidates' if selected else ('candidate_retry_cooldown' if self.rejected else 'no_observed_corridor'))
        self.detail=dict(state=state,continuation_rejected=continuation_rejected,rejected_count=len(self.rejected),expanded=len(finalized),elapsed_s=time.monotonic()-began,
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

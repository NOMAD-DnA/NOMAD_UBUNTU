"""Bounded forward Hybrid A*, integrating at rear axle, publishing base_link poses."""
import heapq
import math
import time
from .base import Grid, Result, Budget, BudgetExceeded


class CostToGoal:
    """Lazy reverse grid search for cost-aware SE(2) search guidance.

    This discretized estimate is guidance, not a continuous optimality bound.
    All motion edges still pass the original collision and gate checks.
    """
    def __init__(self, grid, goal, budget):
        self.grid, self.budget = grid, budget
        self.width, self.height = grid.g['width'], grid.g['height']
        self.weights = [math.inf if v >= 80 else grid.unknown_penalty if v < 0
                        else 1+4*v/79 for v in grid.g['data']]
        cell = grid.cell(*goal)
        self.queue = [(0., cell)]
        self.best, self.closed = {cell: 0.}, {}

    def __call__(self, pose):
        target = self.grid.cell(*pose[:2])
        while target not in self.closed and self.queue:
            cost, cell = heapq.heappop(self.queue)
            if cell in self.closed:
                continue
            # Share the deadline with the kinematic search. Grid expansions
            # do not consume the SE(2) expansion count.
            if time.monotonic() > self.budget.deadline:
                raise BudgetExceeded('planning budget exhausted')
            self.closed[cell] = cost
            x, y = cell
            weight = self.weights[y*self.width+x]
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nx, ny = x+dx, y+dy
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                other = self.weights[ny*self.width+nx]
                if not math.isfinite(other):
                    continue
                if dx and dy and not (math.isfinite(self.weights[y*self.width+nx])
                                      and math.isfinite(self.weights[ny*self.width+x])):
                    continue
                candidate = cost+(weight+other)*.5*self.grid.r*(math.sqrt(2) if dx and dy else 1.)
                nxt = (nx, ny)
                if candidate < self.best.get(nxt, math.inf):
                    self.best[nxt] = candidate
                    heapq.heappush(self.queue, (candidate, nxt))
        return self.closed.get(target, math.inf)


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def advance(pose, curvature, distance, offset):
    x,y,yaw = pose
    x,y = x-offset*math.cos(yaw), y-offset*math.sin(yaw)
    end = yaw+curvature*distance
    if abs(curvature)<1e-10:
        x,y = x+distance*math.cos(yaw),y+distance*math.sin(yaw)
    else:
        x,y = x+(math.sin(end)-math.sin(yaw))/curvature,y+(math.cos(yaw)-math.cos(end))/curvature
    return x+offset*math.cos(end), y+offset*math.sin(end),wrap(end)


class HybridAStar:
    def __init__(self, wheelbase=.72,max_steer=.4,reference_offset=.36,
                 unknown_penalty=4.5,seconds=0.,expansions=50000):
        self.curvature = math.tan(max_steer)/wheelbase
        self.offset = reference_offset
        self.unknown_penalty = unknown_penalty
        self.seconds,self.expansions = seconds,expansions

    def successors(self, pose, grid):
        length = max(.6,grid.r*2)
        steps = max(4,math.ceil(length/min(.08,grid.r*.4)))
        for ratio in (-1.,-.5,0.,.5,1.):
            yield [advance(pose,ratio*self.curvature,length*i/steps,self.offset)
                   for i in range(1,steps+1)]

    def plan(self, request, corridor=None):
        grid = Grid(request,self.unknown_penalty)
        budget = Budget(self.seconds,self.expansions)
        spatial = max(.12,grid.r*.7)
        def key(p):
            return (round(p[0]/spatial),round(p[1]/spatial),round(wrap(p[2])*72/(2*math.pi))%72)
        def distance(p):
            return math.hypot(p[0]-request.goal[0],p[1]-request.goal[1])
        guidance = CostToGoal(grid, request.goal, budget)
        def heuristic(p):
            return max(distance(p), guidance(p))
        first = request.start
        # Immutable records avoid corrupting reconstructed chains when a bin is improved.
        records = [(first,None,[])]
        best = {key(first):0.}
        queue = [(heuristic(first),0.,0)]
        while queue:
            _,cost,idx = heapq.heappop(queue)
            pose,parent,edge = records[idx]
            if cost != best.get(key(pose)):
                continue
            budget.tick()
            if distance(pose) <= .29:
                segments=[]
                while records[idx][1] is not None:
                    segments.append(records[idx][2]); idx=records[idx][1]
                path=[first]+[p for segment in reversed(segments) for p in segment]
                return Result(path,type(self).__name__,budget.count)
            for segment in self.successors(pose,grid):
                if corridor is not None and any(grid.cell(*p[:2]) not in corridor for p in segment):
                    continue
                edge_cost = grid.path_cost([pose]+segment)
                if not math.isfinite(edge_cost):
                    continue
                endpoint=segment[-1]; nk=key(endpoint)
                candidate=cost+edge_cost
                if candidate+1e-8 < best.get(nk,math.inf):
                    best[nk]=candidate
                    records.append((endpoint,idx,segment))
                    heapq.heappush(queue,(candidate+heuristic(endpoint),candidate,len(records)-1))
        return Result(detail='no forward kinematic path',expansions=budget.count)

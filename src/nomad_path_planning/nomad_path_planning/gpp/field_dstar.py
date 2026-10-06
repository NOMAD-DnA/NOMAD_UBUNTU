"""Interpolated D* Lite value field on adjacent grid triangles.

Uses linear edge interpolation with a uniform conservative triangle cost. This is
a Field-D* variant, not the original paper's two-cell analytic cost formula.
Map increases/decreases repair the same g/rhs cache. Continuous extraction is
sampled and validated; an Ackermann reference adapter is applied separately.
"""
import math
from .dstar_lite import DStarLite
from .base import Grid,Result,Budget,poses


class FieldGraph(DStarLite):
    ring=((-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1))

    def heuristic(self,a,b):
        return math.hypot(a[0]-b[0],a[1]-b[1])

    def triangle_value(self,u,a,b):
        if any(self.is_blocked(p) for p in (u,a,b)):
            return math.inf
        if any(not math.isfinite(self.edge_cost(u,p)) for p in (a,b)):
            return math.inf
        ga,gb=self.g[self.index(a)],self.g[self.index(b)]
        if not all(map(math.isfinite,(ga,gb))):
            return math.inf
        cost=max(self.terrain_penalty(p) for p in (u,a,b))
        def value(t):
            x,y=a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])
            return cost*math.hypot(x-u[0],y-u[1])+(1-t)*ga+t*gb
        lo,hi=0.,1.
        for _ in range(16):
            left=lo+(hi-lo)/3; right=hi-(hi-lo)/3
            if value(left)<value(right): hi=right
            else: lo=left
        return min(value(0),value(1),value((lo+hi)/2))

    def update_vertex(self,u):
        if u!=self.goal:
            values=[self.edge_cost(u,n)+self.g[self.index(n)] for n in self.neighbors(u)]
            ring=[(u[0]+x,u[1]+y) for x,y in self.ring]
            values += [self.triangle_value(u,a,b) for a,b in zip(ring,ring[1:]+ring[:1])]
            self.rhs[self.index(u)]=min(values,default=math.inf)
        self.remove(u)
        if self.g[self.index(u)]!=self.rhs[self.index(u)]:
            self.push(u)

    def pop(self):
        self.budget.tick()
        return super().pop()


class FieldDStar:
    def __init__(self,unknown_penalty=4.5,seconds=1.,expansions=50000):
        self.unknown_penalty=unknown_penalty
        self.seconds,self.expansions=seconds,expansions
        self.graph=None; self.signature=None

    def plan(self,request):
        grid=Grid(request,self.unknown_penalty); data=request.grid
        start,goal=grid.cell(*request.start[:2]),grid.cell(*request.goal[:2])
        signature=(data['width'],data['height'],grid.r,data['origin_x'],data['origin_y'],goal,
                   tuple((g.x,g.y,g.yaw,g.half_width) for g in request.gates))
        if self.graph is None or signature!=self.signature:
            self.graph=FieldGraph(data['width'],data['height'],data['data'],start,goal,
                                  unknown_penalty=self.unknown_penalty)
            self.signature=signature
        graph=self.graph
        graph.budget=Budget(self.seconds,self.expansions)
        graph.edge_filter=lambda a,b: not any(g.blocks(grid.world(a),grid.world(b)) for g in request.gates)
        graph.move_start(start)
        graph.update_costmap(data['data'])
        if not graph.compute_shortest_path():
            return Result(detail='no field path')
        cells=graph.extract_path()
        if not cells:
            return Result(detail='field extraction failed')
        # Continuous shortcut extraction, accepted only if it does not increase
        # the integrated map cost. Value propagation above is interpolated D*,
        # not A* relabelled as Field D*.
        points=[request.start[:2]]+[grid.world(c) for c in cells]+[request.goal[:2]]
        output=[points[0]]; i=0
        while i<len(points)-1:
            graph.budget.tick()
            target=i+1; walked=0.
            for j in range(i+1,min(len(points),i+16)):
                walked+=grid.segment(points[j-1],points[j])
                direct=grid.segment(points[i],points[j])
                if math.isfinite(direct) and direct<=walked+1e-8:
                    target=j
            output.append(points[target]); i=target
        if not math.isfinite(grid.path_cost(output)):
            return Result(detail='field path validation failed')
        return Result(poses(output),'interpolated Field D* variant',graph.budget.count)

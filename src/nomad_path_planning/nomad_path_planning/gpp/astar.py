"""A*/Weighted A*: same costs and directional gates as the legacy D* planner."""
import heapq
import math
from .base import Grid, Result, Budget, poses
from .dstar_lite import DStarLite


class AStar:
    def __init__(self, weight=1., unknown_penalty=4.5, seconds=1., expansions=50000):
        if not math.isfinite(weight) or weight < 1:
            raise ValueError('weighted_astar_weight must be finite and >= 1')
        self.weight, self.unknown_penalty = weight, unknown_penalty
        self.seconds, self.expansions = seconds, expansions

    def plan(self, request):
        grid = Grid(request,self.unknown_penalty)
        start,goal = grid.cell(*request.start[:2]),grid.cell(*request.goal[:2])
        g = request.grid
        graph = DStarLite(g['width'],g['height'],g['data'],start,goal,
                          unknown_penalty=self.unknown_penalty)
        graph.edge_filter = lambda a,b: not any(g.blocks(grid.world(a),grid.world(b)) for g in request.gates)
        budget = Budget(self.seconds,self.expansions)
        costs,parents = {start:0.},{}
        queue = [(self.weight*graph.heuristic(start,goal),0.,start)]
        while queue:
            _,cost,cell = heapq.heappop(queue)
            if cost != costs.get(cell):
                continue
            budget.tick()
            if cell == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(parents[path[-1]])
                points = [grid.world(p) for p in reversed(path)]
                points = [request.start[:2]]+points+[request.goal[:2]]
                if not math.isfinite(grid.path_cost(points)):
                    return Result(detail='endpoint connector blocked')
                return Result(poses(points),'weighted A*' if self.weight>1 else 'A*',budget.count)
            for nxt in graph.neighbors(cell):
                candidate = cost+graph.edge_cost(cell,nxt)
                if candidate < costs.get(nxt,math.inf):
                    costs[nxt],parents[nxt] = candidate,cell
                    heapq.heappush(queue,(candidate+self.weight*graph.heuristic(nxt,goal),candidate,nxt))
        return Result(detail='no path',expansions=budget.count)

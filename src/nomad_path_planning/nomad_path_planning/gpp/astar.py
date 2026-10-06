"""A*/Weighted A*: same costs and directional gates as the legacy D* planner."""
import heapq
import math
from .base import Grid, Result, Budget, poses
from .dstar_lite import DStarLite
from .hybrid_astar import CostToGoal


class AStar:
    def __init__(self, weight=1., unknown_penalty=4.5, seconds=0., expansions=50000):
        if not math.isfinite(weight) or weight < 1:
            raise ValueError('weighted_astar_weight must be finite and >= 1')
        self.weight, self.unknown_penalty = weight, unknown_penalty
        self.seconds, self.expansions = seconds, expansions

    def plan(self, request):
        budget = Budget(self.seconds,self.expansions)
        grid = Grid(request,self.unknown_penalty)
        start,goal = grid.cell(*request.start[:2]),grid.cell(*request.goal[:2])
        g = request.grid
        graph = DStarLite(g['width'],g['height'],g['data'],start,goal,
                          unknown_penalty=self.unknown_penalty)
        if request.gates:
            graph.edge_filter = lambda a,b: not any(g.blocks(grid.world(a),grid.world(b)) for g in request.gates)
        guidance = CostToGoal(grid, request.goal, budget)
        def heuristic(cell):
            # Reverse guidance is in metres; graph costs are in cell units.
            # Ignoring directional gates here is a relaxation. Forward edges
            # and the final path still enforce every gate and collision check.
            return guidance(grid.world(cell)) / grid.r
        initial = heuristic(start)
        if not math.isfinite(initial):
            return Result(detail='no path', expansions=budget.count)
        costs,parents = {start:0.},{}
        queue = [(self.weight*initial,0.,start)]
        while queue:
            _,negative_cost,cell = heapq.heappop(queue)
            cost = -negative_cost
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
                    # Prefer progress toward the goal on equal-cost contours.
                    heapq.heappush(queue,(candidate+self.weight*heuristic(nxt),-candidate,nxt))
        return Result(detail='no path',expansions=budget.count)

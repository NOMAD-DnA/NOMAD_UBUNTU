"""Factory and explicit strategy names. No silent algorithm substitution."""
from .astar import AStar
from .hybrid_astar import HybridAStar
from .state_lattice import StateLattice
from .field_dstar import FieldDStar
from .base import Grid,Result,Budget,poses
from .dstar_lite import DStarLite
import math

NAMES=('dstar_lite','astar','weighted_astar','hybrid_astar','state_lattice','field_dstar')


class DStarStrategy:
    """Adapter for non-legacy combinations. The default node retains its fallback."""
    def __init__(self,unknown_penalty=4.5,seconds=0.,expansions=50000):
        self.unknown_penalty=unknown_penalty
        self.seconds,self.expansions=seconds,expansions
        self.graph=None; self.signature=None

    def plan(self,request):
        grid=Grid(request,self.unknown_penalty); d=request.grid
        start,goal=grid.cell(*request.start[:2]),grid.cell(*request.goal[:2])
        signature=(d['width'],d['height'],grid.r,d['origin_x'],d['origin_y'],goal,request.gates)
        if self.graph is None or signature!=self.signature:
            self.graph=DStarLite(d['width'],d['height'],d['data'],start,goal,unknown_penalty=self.unknown_penalty)
            self.signature=signature
        graph=self.graph
        graph.edge_filter=lambda a,b: not any(g.blocks(grid.world(a),grid.world(b)) for g in request.gates)
        graph.move_start(start); graph.update_costmap(d['data'])
        budget=Budget(self.seconds,self.expansions)
        original=graph.pop
        def bounded_pop():
            budget.tick(); return original()
        graph.pop=bounded_pop
        try:
            success=graph.compute_shortest_path()
        finally:
            graph.pop=original
        cells=graph.extract_path() if success else []
        if not cells: return Result(detail='no D* path')
        points=[request.start[:2]]+[grid.world(c) for c in cells]+[request.goal[:2]]
        return Result(poses(points),'D* Lite',budget.count) if math.isfinite(grid.path_cost(points)) else Result(detail='blocked endpoint')


def create(name, *, wheelbase=.72,max_steer=.4,reference_offset=.36,
           unknown_penalty=4.5,seconds=0.,expansions=50000,weight=1.5):
    if name not in NAMES:
        raise ValueError(f'Unknown GPP {name!r}; choose {", ".join(NAMES)}')
    values=(wheelbase,max_steer,reference_offset,unknown_penalty,seconds,weight)
    if not all(math.isfinite(v) for v in values) or not (wheelbase>0 and 0<max_steer<math.pi/2 and reference_offset>=0 and unknown_penalty>=1 and seconds>=0 and expansions>0 and weight>=1):
        raise ValueError('Invalid GPP geometry/budget/weight parameters')
    common=dict(unknown_penalty=unknown_penalty,seconds=seconds,expansions=expansions)
    if name in ('astar','weighted_astar'):
        return AStar(weight=weight if name=='weighted_astar' else 1.,**common)
    if name=='field_dstar': return FieldDStar(**common)
    if name=='dstar_lite': return DStarStrategy(**common)
    vehicle=dict(wheelbase=wheelbase,max_steer=max_steer,reference_offset=reference_offset)
    return (HybridAStar if name=='hybrid_astar' else StateLattice)(**common,**vehicle)

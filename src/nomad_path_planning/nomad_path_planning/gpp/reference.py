"""Adapter: constrain a geometric route to an Ackermann corridor for RPP.

Never relabel the adapter as the selected GPP; report both stages. If no feasible
route exists within the corridor, return failure instead of a clipped curvature.
"""
import math
from .base import Grid,Result
from .hybrid_astar import HybridAStar


class ReferenceAdapter:
    def __init__(self, **vehicle):
        self.search=HybridAStar(**vehicle)

    def adapt(self,request,result):
        if not result.path: return result
        grid=Grid(request)
        corridor=set(); radius=math.ceil(1.2/grid.r)
        for a,b in zip(result.path,result.path[1:]):
            count=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/grid.r))
            for i in range(count+1):
                cx,cy=grid.cell(a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count)
                for dx in range(-radius,radius+1):
                    for dy in range(-radius,radius+1):
                        if dx*dx+dy*dy<=radius*radius: corridor.add((cx+dx,cy+dy))
        adapted=self.search.plan(request,corridor)
        adapted.detail=result.detail+' + Ackermann corridor adapter: '+adapted.detail
        adapted.expansions+=result.expansions
        return adapted

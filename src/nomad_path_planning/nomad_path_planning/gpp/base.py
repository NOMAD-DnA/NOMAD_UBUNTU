"""Shared world-coordinate contract. Costmaps are already inflated by perception."""
from dataclasses import dataclass, field
from typing import Protocol
import math
import time


@dataclass
class Request:
    grid: dict
    start: tuple
    goal: tuple
    gates: tuple = ()


@dataclass
class Result:
    path: list = field(default_factory=list)  # base_link (x, y, yaw), forward only
    detail: str = ''
    expansions: int = 0


class GlobalPlanner(Protocol):
    def plan(self, request: Request) -> Result: ...


class BudgetExceeded(RuntimeError):
    pass


class Budget:
    def __init__(self, seconds=1.0, expansions=50000):
        self.deadline = time.monotonic() + seconds
        self.limit = expansions
        self.count = 0

    def tick(self):
        self.count += 1
        if self.count > self.limit or time.monotonic() > self.deadline:
            raise BudgetExceeded('planning budget exhausted')


class Grid:
    def __init__(self, request, unknown_penalty=4.5):
        self.request = request
        self.g = request.grid
        self.r = self.g['resolution']
        self.unknown_penalty = unknown_penalty

    def cell(self, x, y):
        return (math.floor((x-self.g['origin_x'])/self.r),
                math.floor((y-self.g['origin_y'])/self.r))

    def world(self, cell):
        return (self.g['origin_x']+(cell[0]+.5)*self.r,
                self.g['origin_y']+(cell[1]+.5)*self.r)

    def value(self, x, y):
        cx, cy = self.cell(x, y)
        if not (0 <= cx < self.g['width'] and 0 <= cy < self.g['height']):
            return math.inf
        v = self.g['data'][cy*self.g['width']+cx]
        return math.inf if v >= 80 else self.unknown_penalty if v < 0 else 1+4*v/79

    def segment(self, a, b):
        if any(g.blocks(a, b) for g in self.request.gates):
            return math.inf
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        count = max(1, math.ceil(length/(self.r*.25)))
        values = [self.value(a[0]+(b[0]-a[0])*i/count,
                             a[1]+(b[1]-a[1])*i/count) for i in range(count+1)]
        # Reject corner cutting, including diagonals past touching inflated cells.
        previous = self.cell(*a[:2])
        for i in range(1, count+1):
            current = self.cell(a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
            if current[0] != previous[0] and current[1] != previous[1]:
                for cell in ((current[0],previous[1]),(previous[0],current[1])):
                    if not math.isfinite(self.value(*self.world(cell))):
                        return math.inf
            previous = current
        return length*sum(values)/len(values) if all(map(math.isfinite,values)) else math.inf

    def path_cost(self, points):
        if not points or not math.isfinite(self.value(*points[0][:2])):
            return math.inf
        return sum(self.segment(a,b) for a,b in zip(points,points[1:]))


def poses(points):
    result = []
    for i, point in enumerate(points):
        a,b = (point,points[i+1]) if i+1<len(points) else (points[max(0,i-1)],point)
        result.append((point[0],point[1],math.atan2(b[1]-a[1],b[0]-a[0])))
    return result

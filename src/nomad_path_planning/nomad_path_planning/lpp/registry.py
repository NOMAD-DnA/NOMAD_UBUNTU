"""Local planner factory preserving the legacy rollout contract."""
from typing import Protocol
from .rollout import AckermannRollout
from .rpp import RegulatedPurePursuit

NAMES=('rollout','rpp')


class LocalPlanner(Protocol):
    speed: float
    def plan(self,start,global_path,costmap): ...


def create(name, *, lookahead=.8,lateral_accel=.35,**kwargs):
    if name=='rollout': return AckermannRollout(**kwargs)
    if name=='rpp': return RegulatedPurePursuit(lookahead=lookahead,lateral_accel=lateral_accel,**kwargs)
    raise ValueError(f'Unknown LPP {name!r}; choose {", ".join(NAMES)}')

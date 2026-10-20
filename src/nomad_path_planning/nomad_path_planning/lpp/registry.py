"""Local planner factory preserving the legacy rollout contract."""
from typing import Protocol
from .rollout import AckermannRollout
from .rpp import RegulatedPurePursuit

NAMES=('rollout','rpp')


class LocalPlanner(Protocol):
    speed: float
    # Shared node supplies base_link (x, y, yaw). RPP requires vehicle yaw;
    # rollout uses XY and also accepts legacy XY-only recovery paths.
    def plan(self,start,global_path,costmap): ...


def create(name, *, lookahead=.8,lateral_accel=.35,adaptive_lookahead=False,**kwargs):
    if name=='rollout': return AckermannRollout(**kwargs)
    if name=='rpp': return RegulatedPurePursuit(lookahead=lookahead,lateral_accel=lateral_accel,adaptive_lookahead=adaptive_lookahead,**kwargs)
    raise ValueError(f'Unknown LPP {name!r}; choose {", ".join(NAMES)}')

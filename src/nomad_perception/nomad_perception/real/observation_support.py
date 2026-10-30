"""Distinct camera captures, using NOMAD dev's support strength and decay.

The strength window is anchored to the last capture. Waiting cannot improve
support or erase its evidence during the hold period. Times remain integer ns.
"""
import math
from .capture_timing import positive_ns

EVIDENCE_WINDOW_NS=3_000_000_000
TIME_TOLERANCE_NS=1_000  # Same 1us tolerance as the dev float-time contract.


def observe_camera(cell,capture_ns):
    """Count a label image once per cell across both sensors and all points."""
    capture_ns=positive_ns(capture_ns)
    history=cell.setdefault('camera_capture_ns',[])
    if history and capture_ns<=history[-1]:return
    history.append(capture_ns)
    cutoff=capture_ns-EVIDENCE_WINDOW_NS
    while history and history[0]<cutoff:history.pop(0)
    cell['camera_observed_ns']=capture_ns


def support_metrics(captures_ns,snapshot_ns,*,hold_s=10.,expire_s=30.):
    """Return score, distinct capture count and span at the latest observation.

    This is the dev policy: min(N/3,1) * min(span/0.5s,1), then hold/decay.
    Missing history produces zero support, independently of slope/terrain cost.
    """
    snapshot_ns=positive_ns(snapshot_ns)
    if not all(math.isfinite(v) for v in (hold_s,expire_s)) or not 0<=hold_s<expire_s:
        raise ValueError('Invalid camera support hold/expiry')
    times=sorted({positive_ns(t) for t in captures_ns if positive_ns(t)<=snapshot_ns+TIME_TOLERANCE_NS})
    if not times:return 0,0,0
    latest=times[-1]
    times=[t for t in times if latest-t<=EVIDENCE_WINDOW_NS+TIME_TOLERANCE_NS]
    count=len(times);span=latest-times[0];age=max(0,snapshot_ns-latest)/1e9
    strength=min(count/3.,1.)*min(span/500_000_000.,1.)
    freshness=1. if age<=hold_s else max(0.,(expire_s-age)/(expire_s-hold_s))
    return round(100*strength*freshness),count,span

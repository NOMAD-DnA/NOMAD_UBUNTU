"""Bounded GPP reference lease for a cooperating LPP; never authorizes motion.

A blocked reference prefix may be kept as guidance, but the local trajectory and
reference suffix must pass current-map checks. Execution guards remain separate.
"""
from dataclasses import dataclass
import math
from .gpp.base import Grid, Request
from .observed_guard import footprint_cells

TRACKING, AVOIDING, BLOCKED = 0, 1, 2


@dataclass(frozen=True)
class Progress:
    stamp: float
    reference: tuple
    state: int
    request_replan: bool
    rejoin_index: int
    motion_stamp: float
    path: tuple
    speed: float
    steering: float
    frame: str


class Cooperation:
    def __init__(self, status_timeout=.5, wall_timeout=1., max_duration=30.,
                 max_wall_duration=120., no_progress_timeout=5.,
                 progress_distance=.15, max_deviation=3., max_speed=.5,
                 wheelbase=.72, max_steer=.4, reference_offset=.36):
        for name, value in locals().copy().items():
            if name == 'self': continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'cooperation.{name} must be positive and finite')
            setattr(self, name, value)
        self.reset()

    def reset(self):
        self.message = None
        self.received = -math.inf
        self.active = False
        self.started = self.started_wall = self.last_progress = None
        self.last_now = None
        self.reason = 'legacy'

    def receive(self, message, wall):
        # Replayed/old messages do not renew the receipt lease.
        if self.message is not None and message.stamp <= self.message.stamp:
            return
        self.message, self.received = message, wall

    def evaluate(self, *, reference_id, reference, start, grid, frame, now, wall,
                 legacy_failed=False):
        """Return legacy / hold / replan. A failure cannot renew the same lease."""
        def reject(reason):
            self.active = False
            self.reason = reason
            self.message = None
            return 'replan'
        m = self.message
        was_active = self.active
        if self.last_now is not None and now < self.last_now:
            self.reset()
            self.reason = 'clock_reset'
            return 'replan' if was_active else 'legacy'
        self.last_now = now
        if m is None:
            return reject('missing_status') if was_active else 'legacy'
        if m.reference != reference_id:
            return reject('reference_changed') if was_active else 'legacy'
        if (m.frame != frame or not -.05 <= now-m.stamp <= self.status_timeout
                or not 0 <= wall-self.received <= self.wall_timeout):
            return reject('status_expired_or_invalid')
        if m.request_replan or m.state == BLOCKED or legacy_failed:
            return reject('lpp_requested_replan')
        if m.state == TRACKING:
            return reject('avoidance_finished_reconnect') if was_active else 'legacy'
        if m.state != AVOIDING:
            return reject('invalid_state')
        if len(reference) < 2 or not 0 <= m.rejoin_index < len(reference):
            return reject('invalid_rejoin_index')
        nearest = min(range(len(reference)), key=lambda i: math.dist(start[:2], reference[i][:2]))
        if m.rejoin_index < nearest:
            return reject('rejoin_behind_progress')
        if min(math.dist(start[:2], p[:2]) for p in reference) > self.max_deviation:
            return reject('deviation_limit')
        # Unknown is not a traversable corridor for granting an avoidance lease.
        observed = dict(grid)
        observed['data'] = [100 if v < 0 else v for v in grid['data']]
        observed.pop('cell_weights', None)
        view = Grid(Request(observed, start, reference[-1][:2], tuple(grid.get('gates', ()))))
        if not math.isfinite(view.path_cost(reference[m.rejoin_index:])):
            return reject('rejoin_suffix_blocked_or_unknown')
        if not self.valid_motion(m, start, grid, view, now):
            return reject('local_motion_invalid')
        if not self.active:
            self.started, self.started_wall = now, wall
            self.last_progress = now
            self.rejoin_index = m.rejoin_index
            self.best_distance = math.dist(start[:2], reference[m.rejoin_index][:2])
            self.active = True
        elif m.rejoin_index != self.rejoin_index:
            return reject('rejoin_changed_request_replan')
        distance = math.dist(start[:2], reference[self.rejoin_index][:2])
        if distance <= self.best_distance-self.progress_distance:
            self.best_distance, self.last_progress = distance, now
        if now-self.started >= self.max_duration or wall-self.started_wall >= self.max_wall_duration:
            return reject('avoidance_duration_limit')
        if now-self.last_progress >= self.no_progress_timeout:
            return reject('avoidance_no_progress')
        self.reason = 'validated_local_avoidance'
        return 'hold'

    def valid_motion(self, m, start, grid, view, now):
        if (not -.05 <= now-m.motion_stamp <= self.status_timeout or len(m.path) < 2
                or len(m.path) > 500 or not all(math.isfinite(v) for p in m.path for v in p)
                or not math.isfinite(m.speed) or not 0 < m.speed <= self.max_speed
                or not math.isfinite(m.steering) or abs(m.steering) > self.max_steer
                or math.dist(m.path[0][:2], start[:2]) > .3
                or abs(angle(m.path[0][2]-start[2])) > .35):
            return False
        if not math.isfinite(view.path_cost([start]+list(m.path))):
            return False
        r, w, h = grid['resolution'], grid['width'], grid['height']
        data = grid['data']
        initial = footprint_cells(grid, start)
        exempt = {(x,y) for x,y in initial if 0 <= x < w and 0 <= y < h and data[y*w+x] < 0}
        length = 0.
        for a,b in zip(m.path,m.path[1:]):
            rear_a = (a[0]-self.reference_offset*math.cos(a[2]), a[1]-self.reference_offset*math.sin(a[2]))
            rear_b = (b[0]-self.reference_offset*math.cos(b[2]), b[1]-self.reference_offset*math.sin(b[2]))
            ds = math.dist(rear_a, rear_b); dyaw = angle(b[2]-a[2])
            if ds < 1e-6 or ds > .25:
                return False
            length += ds
            # Rear-axle chord curvature and tangent, forward-only Ackermann.
            if 2*abs(math.sin(dyaw/2))/ds > math.tan(self.max_steer)/self.wheelbase+1e-3:
                return False
            bearing = math.atan2(rear_b[1]-rear_a[1], rear_b[0]-rear_a[0])
            if abs(angle(bearing-(a[2]+dyaw/2))) > .08:
                return False
            n = max(1, math.ceil(ds/min(.02,r*.1)))
            for j in range(n+1):
                t = j/n; pose = (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),a[2]+t*dyaw)
                for x,y in footprint_cells(grid,pose):
                    if not (0 <= x < w and 0 <= y < h): return False
                    if data[y*w+x] < 0 and (x,y) not in exempt: return False
        return length >= .1


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))

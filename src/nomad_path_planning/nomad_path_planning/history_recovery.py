"""Frontier-free recovery geometry. History is a reference, never free-space proof."""
from collections import deque
from dataclasses import dataclass
import math

from nomad_path_planning.rollout import AckermannRollout, normalize_angle


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def free(grid, x, y):
    cx = math.floor((x-grid['origin_x'])/grid['resolution'])
    cy = math.floor((y-grid['origin_y'])/grid['resolution'])
    return (0 <= cx < grid['width'] and 0 <= cy < grid['height']
            and 0 <= grid['data'][cy*grid['width']+cx] < 80)


def segment_free(grid, a, b):
    n = max(1, math.ceil(distance(a, b)/(grid['resolution']*.4)))
    return all(free(grid, a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)
               for i in range(n+1))


@dataclass(frozen=True)
class EntryGate:
    x: float
    y: float
    yaw: float
    half_width: float = 2.0

    def blocks(self, a, b):
        nx, ny = math.cos(self.yaw), math.sin(self.yaw)
        da, db = (a[0]-self.x)*nx+(a[1]-self.y)*ny, (b[0]-self.x)*nx+(b[1]-self.y)*ny
        if not da <= 0 < db:
            return False
        t = -da/(db-da)
        x, y = a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
        return abs(-(x-self.x)*ny+(y-self.y)*nx) <= self.half_width


def allowed_path(path, gates):
    return all(not gate.blocks(a, b) for a, b in zip(path, path[1:]) for gate in gates)


class ProgressWindow:
    def __init__(self, seconds=4.0, radius=.3):
        self.seconds, self.radius = seconds, radius
        self.samples = deque()

    def clear(self):
        self.samples.clear()

    def stalled(self, now, pose):
        if self.samples and now < self.samples[-1][0]:
            self.clear()
        self.samples.append((now, pose))
        while len(self.samples) > 1 and self.samples[1][0] <= now-self.seconds:
            self.samples.popleft()
        if now-self.samples[0][0] < self.seconds:
            return False
        xs, ys = [p[0] for _, p in self.samples], [p[1] for _, p in self.samples]
        # Bounding box detects no progress, including small forward/reverse oscillation.
        return math.hypot(max(xs)-min(xs), max(ys)-min(ys)) <= self.radius*2


@dataclass
class Breadcrumb:
    pose: tuple
    arclength: float
    escape_space: float


class Trail:
    def __init__(self, spacing=.15, max_length=60.0):
        self.spacing, self.max_length = spacing, max_length
        self.points = []

    def record(self, pose, grid):
        if self.points and distance(self.points[-1].pose, pose) < self.spacing:
            return
        if self.points:
            prev = self.points[-1].pose
            # Never connect a reverse leg or a localization discontinuity as forward history.
            along = (pose[0]-prev[0])*math.cos(prev[2])+(pose[1]-prev[1])*math.sin(prev[2])
            if distance(prev, pose) > 1.0 or along < -.03 or abs(normalize_angle(pose[2]-prev[2])) > .6:
                self.points.clear()
        arc = self.points[-1].arclength+distance(self.points[-1].pose, pose) if self.points else 0.
        clearance = 0.
        # Residual lateral free space after the existing footprint inflation.
        for side in (-1, 1):
            for step in range(1, 16):
                d = step*.1
                if not free(grid, pose[0]-side*d*math.sin(pose[2]), pose[1]+side*d*math.cos(pose[2])):
                    break
                clearance = max(clearance, d)
        self.points.append(Breadcrumb(pose, arc, clearance))
        while len(self.points) > 2 and arc-self.points[0].arclength > self.max_length:
            self.points.pop(0)

    def select(self, current, min_retreat=2.0, max_retreat=8.0, escape_clearance=1.0):
        if len(self.points) < 2 or distance(current, self.points[-1].pose) > .8:
            return None
        end_arc = self.points[-1].arclength+distance(current, self.points[-1].pose)
        candidates = [i for i, p in enumerate(self.points)
                      if min_retreat <= end_arc-p.arclength <= max_retreat]
        if not candidates:
            return None
        wide = [i for i in candidates if self.points[i].escape_space >= escape_clearance]
        index = max(wide) if wide else min(candidates)
        ref = [current] + [p.pose for p in reversed(self.points[index:])]
        # Preserve vehicle yaw; reversing order must NOT rotate the vehicle 180 degrees.
        cleaned = [ref[0]]
        for p in ref[1:]:
            if distance(cleaned[-1], p) > .01:
                cleaned.append(p)
        return index, cleaned


    def retreat(self, current, min_retreat=2.0, max_retreat=60.0):
        # Choose the oldest connected point in range. Intermediate breadcrumbs
        # are tracking references, never stop/replan checkpoints.
        return self.select(current, min_retreat, max_retreat, math.inf)


class TrajectoryFollower:
    """Track a recorded pose sequence with signed speed and a monotonic cursor."""
    def __init__(self, reference, wheelbase=.72, max_steer=.4, offset=.36,
                 speed=-.2, horizon=.6, tolerance=.18):
        self.last_steer = 0.0
        self.reference = reference
        self.cursor = 0
        self.tolerance = tolerance
        self.model = AckermannRollout(wheelbase=wheelbase, max_steer=max_steer,
            reference_offset=offset, speed=speed, horizon=horizon, dt=.1,
            steer_samples=17, allow_unknown=False)
        self.arc = [0.]
        for a, b in zip(reference, reference[1:]):
            self.arc.append(self.arc[-1]+distance(a, b))

    def advance(self, pose):
        # Limited search prevents jumps at crossings/parallel nearby trail segments.
        stop = min(len(self.reference), self.cursor+9)
        self.cursor = min(range(self.cursor, stop), key=lambda i: distance(pose, self.reference[i]))
        return (self.arc[-1]-self.arc[self.cursor] < .35
                and distance(pose, self.reference[-1]) <= self.tolerance
                and abs(normalize_angle(pose[2]-self.reference[-1][2])) < .35)

    def plan(self, pose, grid):
        remaining = self.arc[-1]-self.arc[self.cursor]
        self.model.horizon = max(.03, min(.6, remaining))
        target_idx = self.cursor
        while target_idx+1 < len(self.reference) and self.arc[target_idx]-self.arc[self.cursor] < self.model.horizon:
            target_idx += 1
        target = self.reference[target_idx]
        ref = self.reference[self.cursor:min(len(self.reference), target_idx+5)]
        best = None
        self.last_steer = 0.0
        for steer in self.model.steering_candidates():
            trajectory = self.model.simulate(pose, steer)
            if not all(segment_free(grid, a, b) for a, b in zip(trajectory, trajectory[1:])):
                continue
            cross = sum(min(distance(p, r) for r in ref) for p in trajectory)/len(trajectory)
            if max(min(distance(p, r) for r in ref) for p in trajectory) > .55:
                continue
            end = trajectory[-1]
            score = 5*distance(end, target)+2*abs(normalize_angle(end[2]-target[2]))+3*cross+.05*abs(steer)
            if best is None or score < best[0]:
                best = score, trajectory
                self.last_steer = steer
        return None if best is None else best[1]


class ReverseFollower(TrajectoryFollower):
    def __init__(self, reference, wheelbase=.72, max_steer=.4, offset=.36,
                 speed=.2, horizon=.6, tolerance=.18):
        super().__init__(reference, wheelbase, max_steer, offset, -speed, horizon, tolerance)

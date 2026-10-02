"""Bounded forward Ackermann search for a known-free exit to the original route.

A free straight rollout inside the dead end is not an exit. The full maneuver
must leave the failed trail, advance along GPP, and align with its new branch.
"""
import math
from nomad_path_planning.history_recovery import distance, segment_free, allowed_path
from nomad_path_planning.rollout import AckermannRollout, normalize_angle


class Polyline:
    def __init__(self, points):
        self.segments = []
        arc = 0.
        for a, b in zip(points, points[1:]):
            length = distance(a, b)
            if length < 1e-6:
                continue
            self.segments.append((a, b, length, arc, math.atan2(b[1]-a[1], b[0]-a[0])))
            arc += length
        self.length = arc

    def project(self, point):
        best = (math.inf, 0., 0.)
        for a, b, length, arc, heading in self.segments:
            t = max(0., min(1., ((point[0]-a[0])*(b[0]-a[0])
                                +(point[1]-a[1])*(b[1]-a[1]))/(length*length)))
            error = math.hypot(point[0]-a[0]-t*(b[0]-a[0]), point[1]-a[1]-t*(b[1]-a[1]))
            if error < best[0]:
                best = error, arc+t*length, heading
        return best


class EscapeProbe:
    def __init__(self, wheelbase=.72, max_steer=.4, offset=.36, speed=.3,
                 horizon=3.0, separation=1.0):
        self.model = AckermannRollout(wheelbase=wheelbase, max_steer=max_steer,
            reference_offset=offset, speed=speed, horizon=.5, dt=.1, steer_samples=7,
            allow_unknown=False)
        self.horizon, self.separation = horizon, separation

    def plan(self, pose, route, grid, failed_trail, gates=(), start_tolerance=.6):
        if len(route) < 2 or not allowed_path(route, gates):
            return None
        # Only a nearby, forward-going route can be taken without more backing.
        route_line = Polyline(route)
        error, start_arc, _ = route_line.project(pose)
        if error > start_tolerance or route_line.length-start_arc < 2.0:
            return None
        ahead = next((b for _, b, length, arc, _ in route_line.segments if arc+length >= start_arc+.8), route[-1])
        if ((ahead[0]-pose[0])*math.cos(pose[2])+(ahead[1]-pose[1])*math.sin(pose[2])) < -.1:
            return None
        # Crop the projection region; distant loops must not count as local progress.
        points = [a for a, _, length, arc, _ in route_line.segments
                  if start_arc-.3 <= arc+length and arc <= start_arc+self.horizon+2.]
        last = next((b for _, b, length, arc, _ in route_line.segments
                     if arc+length >= start_arc+self.horizon+2.), route[-1])
        line = Polyline(points+[last])
        _, initial, _ = line.project(pose)
        history = Polyline(failed_trail)
        # Beam search allows steering to change during the turn. Every subsegment
        # uses the latest already-inflated local grid, including unknown rejection.
        beam = [(0., [pose])]
        for _ in range(math.ceil(self.horizon/self.model.horizon)):
            expanded = []
            for _, prefix in beam:
                for steer in self.model.steering_candidates():
                    leg = self.model.simulate(prefix[-1], steer)
                    if not allowed_path(leg, gates) or not all(segment_free(grid,a,b) for a,b in zip(leg,leg[1:])):
                        continue
                    error, progress, heading = line.project(leg[-1])
                    if error > 1.2 or progress < initial-.2:
                        continue
                    heading_error = abs(normalize_angle(leg[-1][2]-heading))
                    score = 4*error+heading_error-1.5*(progress-initial)+.05*abs(steer)
                    expanded.append((score, prefix+leg[1:]))
            if not expanded:
                return None
            # Keep distinct poses, avoiding many almost identical beam branches.
            beam, cells = [], set()
            for item in sorted(expanded, key=lambda v:v[0]):
                end = item[1][-1]
                key = (round(end[0]/.15), round(end[1]/.15), round(end[2]/.15))
                if key not in cells:
                    beam.append(item)
                    cells.add(key)
                if len(beam) >= 10:
                    break
        for _, trajectory in beam:
            end = trajectory[-1]
            error, progress, heading = line.project(end)
            if (error <= .6 and progress-initial >= 2.0
                    and abs(normalize_angle(end[2]-heading)) <= .6
                    and history.project(end)[0] >= self.separation):
                return trajectory
        return None


class ExitAdjustment:
    """A short reverse turn is useful only if its endpoint has a verified exit."""
    def __init__(self, probe, reverse_speed=.2, length=.6):
        self.probe = probe
        model = probe.model
        self.reverse = AckermannRollout(wheelbase=model.wheelbase,
            max_steer=model.max_steer, reference_offset=model.reference_offset,
            speed=-reverse_speed, horizon=length, dt=.1, steer_samples=7,
            allow_unknown=False)

    def plan(self, pose, route, grid, failed_trail, gates=()):
        choices = []
        for steer in self.reverse.steering_candidates():
            reverse = self.reverse.simulate(pose, steer)
            if (not allowed_path(reverse, gates)
                    or not all(segment_free(grid, a, b) for a, b in zip(reverse, reverse[1:]))):
                continue
            forward = self.probe.plan(reverse[-1], route, grid, failed_trail, gates,
                                      start_tolerance=.8)
            if forward is not None:
                # Prefer little steering when several complete maneuvers are feasible.
                choices.append((abs(steer), reverse, forward))
        if not choices:
            return None
        _, reverse, forward = min(choices, key=lambda item: item[0])
        return reverse, forward

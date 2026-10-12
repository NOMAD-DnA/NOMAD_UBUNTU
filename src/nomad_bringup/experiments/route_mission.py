"""Ordered mission goals from a design route, not a replacement planner path."""
import csv
import math


def loop_waypoints(path, laps=2, spacing=4.):
    if laps < 1 or spacing <= 0:
        raise ValueError('laps and spacing must be positive')
    with open(path) as stream:
        points = [(float(r['x_m']), float(r['y_m'])) for r in csv.DictReader(stream)]
    if len(points) < 3 or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('Invalid route')
    if math.dist(points[-1], points[0]) > 1.:
        raise ValueError('Route must be a closed loop')
    points[-1] = points[0]
    goals = []
    accumulated = 0.
    for previous, point in zip(points, points[1:]):
        accumulated += math.dist(previous, point)
        if accumulated >= spacing:
            goals.append(list(point))
            accumulated = 0.
    if goals and math.dist(goals[-1], points[0]) < spacing / 2:
        goals.pop()
    goals.append(list(points[0]))
    return goals * laps, len(goals)

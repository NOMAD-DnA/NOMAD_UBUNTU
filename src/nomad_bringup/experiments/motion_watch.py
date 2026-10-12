"""Stationary timeout in simulator time, independent of planner status strings."""
import math


class MotionWatch:
    def __init__(self, now, position, timeout=60., radius=.5):
        self.since = now
        self.anchor = tuple(position[:2])
        self.timeout = timeout
        self.radius = radius

    def update(self, now, position):
        if math.dist(self.anchor, position[:2]) >= self.radius:
            self.anchor = tuple(position[:2])
            self.since = now
        return now - self.since >= self.timeout

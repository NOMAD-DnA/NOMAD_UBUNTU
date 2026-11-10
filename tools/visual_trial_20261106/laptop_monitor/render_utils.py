"""Laptop-only render caches. Stored measurements and control inputs are intact."""
from __future__ import annotations


def history_signature(history):
    if not history:
        return (id(history), 0)
    keys = ("t", "x", "y", "z", "qx", "qy", "qz", "qw", "frame_id")
    return (id(history), len(history), tuple(history[0].get(k) for k in keys),
            tuple(history[-1].get(k) for k in keys))


class HistorySampler:
    """Cache one source/frame and enforce an exact drawing-point cap."""
    def __init__(self, limit=4000):
        self.limit = max(2, int(limit))
        self.key = None
        self.points = []

    def sample(self, history, frame=None):
        key = (history_signature(history), frame)
        if key == self.key:
            return self.points
        points = ([p for p in history if p.get("frame_id") == frame]
                  if frame is not None else list(history))
        count = len(points)
        if count > self.limit:
            points = [points[i*(count-1)//(self.limit-1)] for i in range(self.limit)]
        self.key, self.points = key, points
        return points


def fit_axes_3d(ax, points, *, force=False, initialized=False):
    """Expand padded bounds only when data approaches an edge; never breathe."""
    if not points:
        return initialized
    mins = [min(p[k] for p in points) for k in ("x", "y", "z")]
    maxs = [max(p[k] for p in points) for k in ("x", "y", "z")]
    limits = (ax.get_xlim(), ax.get_ylim(), ax.get_zlim())
    if initialized and not force and all(
            low + .07*(high-low) <= lo and hi <= high - .07*(high-low)
            for lo, hi, (low, high) in zip(mins, maxs, limits)):
        return True
    half = max(.5, max(hi-lo for lo, hi in zip(mins, maxs))*.65)
    for low, high, setter in zip(mins, maxs, (ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        center = (low+high)/2
        setter(center-half, center+half)
    return True


def set_text(label, text):
    if label.text() != text:
        label.setText(text)


def set_style(widget, style):
    if widget.styleSheet() != style:
        widget.setStyleSheet(style)


def stable_metadata(label, rows=2):
    """Keep changing age/error text from resizing adjacent video or canvases."""
    label.setWordWrap(True)
    label.setFixedHeight(label.fontMetrics().lineSpacing()*rows + 4)
    policy = label.sizePolicy()
    policy.setHorizontalPolicy(policy.Ignored)
    policy.setVerticalPolicy(policy.Fixed)
    label.setSizePolicy(policy)

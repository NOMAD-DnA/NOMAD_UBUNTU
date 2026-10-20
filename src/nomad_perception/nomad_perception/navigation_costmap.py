"""Observed terrain -> global/local costs; no ROS publishers or commands."""

import math
from collections import deque
import cv2
import numpy as np
from .terrain_layers import Layers


class TemporalBlockEvidence:
    """Independent capture votes; no repeated timestamp can confirm a block."""

    def __init__(self, window=3.0, min_hits=3, min_span=0.5, share=0.75):
        self.window = window
        self.min_hits = min_hits
        self.min_span = min_span
        self.share = share
        self.history = {}
        self.confirmed = set()

    def update(self, key, blocked, stamp):
        history = self.history.setdefault(key, deque())
        if history and stamp <= history[-1][0]:
            return key in self.confirmed
        history.append((stamp, bool(blocked)))
        while history and history[0][0] < stamp - self.window:
            history.popleft()
        hits = [t for t, b in history if b]
        if (
            len(hits) >= self.min_hits
            and hits[-1] - hits[0] >= self.min_span
            and len(hits) / len(history) >= self.share
        ):
            self.confirmed.add(key)
        # Clear only repeated direct nonblocking evidence, not unobserved time.
        misses = [t for t, b in history if not b]
        if (
            len(misses) >= 4
            and misses[-1] - misses[0] >= 1.0
            and not any(b for t, b in history if t >= stamp - 1.0)
        ):
            self.confirmed.discard(key)
        return key in self.confirmed


class NavigationLayers(Layers):
    def __init__(self, *args, **kwargs):
        self.slope_limit = kwargs.pop("slope_limit", 10.0)
        self.obstacle_cost_radius = kwargs.pop("obstacle_cost_radius", 1.2)
        self.passable_semantic_ids = tuple(kwargs.pop("passable_semantic_ids", (3,)))
        if (
            not math.isfinite(self.obstacle_cost_radius)
            or self.obstacle_cost_radius <= 0
        ):
            raise ValueError("obstacle_cost_radius must be finite and positive")
        self.bootstrap_override = kwargs.pop("bootstrap_override", False)
        super().__init__(*args, **kwargs)
        # Continuous obstacle costs replace the former all-blocked dilation.
        self.views.grid.kernel = np.ones((1, 1), dtype=np.uint8)
        self.seen = {}
        self.solid = {}
        self.body_z = None
        self.block_evidence = TemporalBlockEvidence()

    def bootstrap_free(self, x, y, stamp, radius=1.5, yaw=None):
        """User-selected initial clear-area assumption; unknown cells only."""
        if getattr(self, "bootstrap_origin", None) is not None:
            return
        if not math.isfinite(radius) or radius < 0:
            raise ValueError("bootstrap_radius must be finite and nonnegative")
        if radius == 0:
            return  # Disabled: do not invent seen timestamps or free cells.
        g = self.views.grid
        g.ensure_bounds(x, y, radius)
        yy, xx = np.mgrid[: g.height, : g.width]
        inside = (g.origin_x + (xx + 0.5) * g.resolution - x) ** 2 + (
            g.origin_y + (yy + 0.5) * g.resolution - y
        ) ** 2 <= radius**2
        if yaw is not None:
            forward = (g.origin_x + (xx + 0.5) * g.resolution - x) * math.cos(yaw) + (
                g.origin_y + (yy + 0.5) * g.resolution - y
            ) * math.sin(yaw)
            inside &= forward >= -0.25
        self.bootstrap_yaw = yaw
        unknown = ~np.isfinite(g.seen)
        mask = inside & unknown
        g.log_odds[mask] = -3.0
        g.seen[mask] = stamp
        self.bootstrap_origin = (float(x), float(y), float(stamp), float(radius))
        self.bootstrap_active = True

    def ingest(self, xyz, classes, stamp):
        estimates = super().ingest(xyz, classes, stamp)
        keys = np.floor(xyz[:, :2] / self.views.grid.resolution).astype(np.int64)
        observed = np.unique(keys, axis=0)
        for k in observed:
            self.seen[tuple(k)] = stamp
        # A cached plane alone is not a new independent ground observation.
        ground_keys = {tuple(k) for k in keys[np.isin(classes, self.ground_ids)]}
        candidates = set()
        height_points = 0
        vetoed = 0
        # Only solid returns within vehicle collision height, not overhead canopy.
        if self.body_z is not None:
            reference = np.full(len(xyz), self.body_z, dtype=np.float64)
            for i, k in enumerate(keys):
                e = estimates.get(tuple(k))
                if e is not None and e.height_valid and e.height is not None:
                    reference[i] = e.height
            # A semantic label alone must not turn ground-level misclassification
            # into a collision. Require a measurable raised solid return.
            mask = (
                np.isin(classes, [1, 6, 7])
                & (xyz[:, 2] >= reference + 0.2)
                & (xyz[:, 2] <= reference + 0.8)
            )
            height_points = int(mask.sum())
            unique, counts = np.unique(keys[mask], axis=0, return_counts=True)
            for k, n in zip(unique, counts):
                if n >= 3:
                    candidates.add(tuple(k))
        for key in observed:
            k = tuple(key)
            e = estimates.get(k)
            steep = (
                e is not None
                and e.height_valid
                and e.height is not None
                and np.isfinite(e.height)
                and e.slope_valid
                and e.slope_deg is not None
                and e.slope_deg >= self.slope_limit
            )
            fresh_slope = (
                k in ground_keys and e is not None
                and not getattr(e, "slope_reused", False)
            )
            votes = self.votes.get(k)
            passable = (
                votes is not None
                and votes.sum() >= self.min_points
                and votes[list(self.passable_semantic_ids)].sum() / votes.sum() >= 0.6
            )
            if passable and (k in candidates or steep):
                vetoed += 1
            # This experiment treats confident grass as traversable vegetation.
            # A normal fit to foliage is not ground-slope proof at a grass cell.
            if steep and not fresh_slope and k not in candidates and not passable:
                # No new slope evidence: neither confirm nor clear this cell.
                continue
            confirmed = self.block_evidence.update(
                k, (k in candidates or steep) and not passable, stamp
            )
            if confirmed:
                self.solid[k] = stamp
            elif k in self.solid:
                del self.solid[k]
        self.obstacle_diagnostics = dict(projected_points=len(xyz),
            tree_points=int(np.count_nonzero(classes == 1)),
            rock_points=int(np.count_nonzero(classes == 7)),
            collision_height_points=height_points, candidate_cells=len(candidates),
            grass_veto_cells=vetoed, confirmed_cells=len(self.solid))
        return estimates


def scan_free_memory(layers, now):
    """Keep actual negative-ray observations, aligned after bounded grid growth."""
    g = layers.views.grid
    shape = (g.height, g.width)
    previous = getattr(layers, "scan_free_seen", None)
    metadata = (g.origin_x, g.origin_y, g.resolution)
    if previous is None:
        history = np.full(shape, -np.inf)
    elif previous.shape == shape and layers.scan_free_metadata == metadata:
        history = previous
    else:
        history = np.full(shape, -np.inf)
        oldx, oldy, oldr = layers.scan_free_metadata
        if oldr == g.resolution:
            dx = round((oldx - g.origin_x) / g.resolution)
            dy = round((oldy - g.origin_y) / g.resolution)
            sx = max(0, -dx)
            sy = max(0, -dy)
            ex = min(previous.shape[1], g.width - dx)
            ey = min(previous.shape[0], g.height - dy)
            if ex > sx and ey > sy:
                history[sy + dy : ey + dy, sx + dx : ex + dx] = previous[sy:ey, sx:ex]
    actual = (g.log_odds < 0) & np.isfinite(g.seen) & (g.seen <= now + 1e-6)
    initial = getattr(layers, "bootstrap_origin", None)
    if initial is not None:
        actual &= np.abs(g.seen - initial[2]) > 1e-6
    history[actual] = np.maximum(history[actual], g.seen[actual])
    layers.scan_free_seen = history
    layers.scan_free_metadata = metadata
    return np.isfinite(history), now - history


def navigation_arrays(layers, estimates, arrays, now, slope_limit=10.0, local_age=30.0):
    """Ordinary costs 0..79; confirmed collision 100; insufficient evidence -1."""
    g = layers.views.grid
    h, s, c = arrays
    raw = np.full(c.shape, -1, dtype=np.int8)
    seen = np.full(c.shape, -np.inf)
    # Actual class histograms, not relabelled GOOSE IDs. Conservative soft costs.
    semantic = np.array([40, 70, 35, 15, 30, 25, 70, 70, 5, 40], dtype=float)
    for k, v in layers.votes.items():
        x, y = g.cell((k[0] + 0.5) * g.resolution, (k[1] + 0.5) * g.resolution)
        if 0 <= x < g.width and 0 <= y < g.height and v.sum() >= layers.min_points:
            # Slope missing is unknown rather than falsely declaring impassable.
            if np.isfinite(s[y, x]):
                S = float(v @ semantic / v.sum())
                G = 79 * min(s[y, x] / slope_limit, 1.0) ** 2
                raw[y, x] = min(79, round(0.3 * S + 0.7 * G))
            seen[y, x] = layers.seen.get(k, -np.inf)
    # Retain previously observed cell costs in a persistent sparse cache.
    cache = getattr(layers, "navigation_cache", {})
    for y, x in zip(*np.where(raw >= 0)):
        k = (
            math.floor((g.origin_x + (x + 0.5) * g.resolution) / g.resolution),
            math.floor((g.origin_y + (y + 0.5) * g.resolution) / g.resolution),
        )
        cache[k] = (int(raw[y, x]), seen[y, x])
    layers.navigation_cache = cache
    for k, (cost, stamp) in cache.items():
        x, y = g.cell((k[0] + 0.5) * g.resolution, (k[1] + 0.5) * g.resolution)
        if 0 <= x < g.width and 0 <= y < g.height:
            raw[y, x] = cost
            seen[y, x] = max(seen[y, x], stamp)
    # Above-limit slopes remain ordinary pending independent-capture confirmation.
    # Confirmed geometry/solid decisions enter through the shared temporal state.
    for k, stamp in layers.solid.items():
        x, y = g.cell((k[0] + 0.5) * g.resolution, (k[1] + 0.5) * g.resolution)
        if 0 <= x < g.width and 0 <= y < g.height:
            raw[y, x] = 100
            seen[y, x] = max(seen[y, x], stamp)
    blocked = cv2.dilate((raw == 100).astype(np.uint8), g.kernel) != 0
    scanfree, age = scan_free_memory(layers, now)
    globalmap = raw.copy()
    globalmap[(globalmap < 0) & scanfree] = 5
    globalmap[blocked] = 100
    fresh = (
        (raw >= 0) & (raw < 80) & (now - seen >= -1e-6) & (now - seen <= local_age)
    ) | (scanfree & (age <= local_age))
    safe = (
        cv2.erode(
            fresh.astype(np.uint8),
            g.kernel,
            borderType=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        != 0
    )
    localmap = np.full(raw.shape, -1, dtype=np.int8)
    localmap[safe] = np.where(raw[safe] >= 0, raw[safe], 5)
    localmap[blocked] = 100
    if np.any(raw == 100):
        distance = (
            cv2.distanceTransform(
                (raw != 100).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
            )
            * g.resolution
        )
        proximity = np.rint(
            100 * np.clip(1 - (distance / layers.obstacle_cost_radius) ** 2, 0.0, 1.0)
        ).astype(np.int8)
        globalmap = np.where(
            globalmap >= 0, np.maximum(globalmap, proximity), globalmap
        ).astype(np.int8)
        localmap = np.where(
            localmap >= 0, np.maximum(localmap, proximity), localmap
        ).astype(np.int8)
    initial = getattr(layers, "bootstrap_origin", None)
    if initial is not None:
        x, y, _, radius = initial
        yy, xx = np.mgrid[: g.height, : g.width]
        seed = (g.origin_x + (xx + 0.5) * g.resolution - x) ** 2 + (
            g.origin_y + (yy + 0.5) * g.resolution - y
        ) ** 2 <= radius**2
        yaw = getattr(layers, "bootstrap_yaw", None)
        if yaw is not None:
            forward = (g.origin_x + (xx + 0.5) * g.resolution - x) * math.cos(yaw) + (
                g.origin_y + (yy + 0.5) * g.resolution - y
            ) * math.sin(yaw)
            seed &= forward >= -0.25
        globalmap[seed & (globalmap < 0) & ~blocked] = 5
        if getattr(layers, "bootstrap_active", False):
            localmap[seed & (localmap < 0) & ~blocked] = 5
            # Explicit simulation start-area assumption takes output priority.
            # Keep measured evidence intact; outside/after this area normal rules apply.
            if layers.bootstrap_override:
                globalmap[seed] = 5
                localmap[seed] = 5
    return globalmap, localmap

"""Manual mapping cost conversion without altering the shared geometry core."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GeometryCostPolicy:
    """Convert a retained slope estimate to the displayed and archived cost.

    Existing unknown and blocked decisions are preserved. The legacy option
    retains the shared core's result exactly; angle_power_v1 recomputes only
    ordinary costs from the unchanged slope and an optional eight-neighbor
    confidence correction. Raw support confidence remains in the shared core.
    """

    name: str = "angle_power_v1"
    limit_deg: float = 20.0
    power: float = 2.0
    confidence_gain: float = 0.25
    neighbor_confidence_enabled: bool = True
    confidence_low_threshold: float = 0.3
    confidence_neighbor_min: float = 0.7

    def __post_init__(self):
        if self.name not in ("legacy", "angle_power_v1"):
            raise ValueError("cost policy must be legacy or angle_power_v1")
        if not isinstance(self.neighbor_confidence_enabled, bool):
            raise ValueError("neighbor_confidence_enabled must be bool")
        for field in ("limit_deg", "power", "confidence_gain",
                      "confidence_low_threshold", "confidence_neighbor_min"):
            value = getattr(self, field)
            try:
                finite = math.isfinite(value)
            except TypeError:
                finite = False
            if not finite:
                raise ValueError(f"{field} must be finite")
        if self.limit_deg <= 0:
            raise ValueError("limit_deg must be positive")
        if self.power < 1:
            raise ValueError("power must be at least 1")
        if not 0 <= self.confidence_gain <= 1:
            raise ValueError("confidence_gain must be within [0, 1]")
        if not 0 <= self.confidence_low_threshold < self.confidence_neighbor_min <= 1:
            raise ValueError("confidence thresholds must satisfy 0 <= low < neighbor_min <= 1")

    def correct_confidence(self, cells, slopes, q):
        """Raise isolated low confidence using all eight original neighbors.

        Cell coordinates are signed world-grid indices, not flattened image
        offsets. Missing neighbors and invalid slopes cannot supply evidence.
        Every decision reads the original q array, so correction cannot spread.
        The average is a display-cost heuristic, not new measurement support.
        """
        cells = np.asarray(cells)
        slopes = np.asarray(slopes, dtype=np.float64)
        q = np.asarray(q, dtype=np.float64)
        if (q.ndim != 1 or slopes.shape != q.shape or cells.shape != (len(q), 2)
                or cells.dtype.kind not in "iu"):
            raise ValueError("cells must be an integer Nx2 array; slopes and q must be length N")
        result, changed = q.copy(), np.zeros(q.shape, dtype=bool)
        if not self.neighbor_confidence_enabled or self.name == "legacy" or not len(q):
            return result, changed
        valid = np.isfinite(slopes) & (slopes >= 0) & np.isfinite(q) & (q >= 0) & (q <= 1)
        candidates = np.flatnonzero(valid & (q < self.confidence_low_threshold))
        if not len(candidates):
            return result, changed
        lookup = {tuple(map(int, cell)): index for index, cell in enumerate(cells)}
        if len(lookup) != len(cells):
            raise ValueError("cell coordinates must be unique")
        high = valid & (q >= self.confidence_neighbor_min)
        offsets = ((-1, -1), (0, -1), (1, -1), (-1, 0),
                   (1, 0), (-1, 1), (0, 1), (1, 1))
        for index in candidates:
            ix, iy = map(int, cells[index])
            neighbors = [lookup.get((ix + dx, iy + dy)) for dx, dy in offsets]
            if all(other is not None and high[other] for other in neighbors):
                result[index] = float(np.mean(q[neighbors]))
                changed[index] = True
        return result, changed

    def apply(self, cost, slopes, q):
        """Return a same-shape, same-dtype cost array; never mutate inputs."""
        cost = np.asarray(cost)
        slopes = np.asarray(slopes, dtype=np.float64)
        q = np.asarray(q, dtype=np.float64)
        if cost.shape != slopes.shape or cost.shape != q.shape:
            raise ValueError("cost, slopes and q must have the same shape")
        if cost.dtype.kind != "i":
            raise ValueError("cost must use a signed integer dtype")
        if np.any((cost < -1) | (cost > 100)):
            raise ValueError("cost values must be within [-1, 100]")
        result = cost.copy()
        if self.name == "legacy":
            return result
        ordinary = (cost >= 0) & (cost < 100)
        usable = ordinary & np.isfinite(slopes) & (slopes >= 0) & (slopes < self.limit_deg)
        result[ordinary & ~usable] = 100
        confidence = np.clip(np.nan_to_num(q[usable], nan=0.0, posinf=1.0, neginf=0.0), 0.0, 1.0)
        scaled = 99.0 * np.power(slopes[usable] / self.limit_deg, self.power)
        adjusted = scaled * (1.0 + self.confidence_gain * (1.0 - confidence))
        result[usable] = np.minimum(99.0, np.floor(adjusted + 0.5)).astype(cost.dtype)
        return result

    def layers(self, estimates, grid):
        """Retain the shared grid placement and slope image, convert its costs."""
        from nomad_perception.stage1_geometry_layers import geometry_layers

        cost, slopes = geometry_layers(estimates, grid)
        confidence = np.zeros(cost.shape, dtype=np.float64)
        cells, positions, raw_q = [], [], []
        for (ix, iy), item in estimates.items():
            cx, cy = grid.cell((ix + 0.5) * grid.resolution, (iy + 0.5) * grid.resolution)
            if 0 <= cx < grid.width and 0 <= cy < grid.height:
                cells.append((ix, iy))
                positions.append((cy, cx))
                raw_q.append(item.q_g)
        if cells:
            rows, columns = np.asarray(positions).T
            effective, _ = self.correct_confidence(
                np.asarray(cells), slopes[rows, columns], raw_q)
            confidence[rows, columns] = effective
        return self.apply(cost, slopes, confidence), slopes

    def sparse_confidence(self, state):
        """Return cost confidence and its correction mask; preserve raw q_g."""
        if "cells" not in state:
            q = np.asarray(state["q_g"], dtype=np.float64)
            return q.copy(), np.zeros(q.shape, dtype=bool)
        return self.correct_confidence(state["cells"], state["slope_deg"], state["q_g"])

    def sparse_costs(self, state):
        """Return sparse costs matching the policy used by the dense grid."""
        confidence, _ = self.sparse_confidence(state)
        return self.apply(state["output_cost"], state["slope_deg"], confidence)

    def metadata(self):
        return {
            "name": self.name,
            "limit_deg": self.limit_deg,
            "power": self.power,
            "confidence_gain": self.confidence_gain,
            "rounding": "half_up",
            "neighbor_confidence": {
                "enabled": self.neighbor_confidence_enabled and self.name != "legacy",
                "target_below": self.confidence_low_threshold,
                "neighbor_min": self.confidence_neighbor_min,
                "required_neighbors": 8,
                "average": "arithmetic_mean",
                "source": "original_confidence_single_pass",
            },
        }

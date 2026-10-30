"""OccupancyGrid cell conversion shared by GUI and tests."""

from dataclasses import dataclass
from math import floor, isfinite


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def valid(self, data_length: int) -> bool:
        return (
            self.width > 0 and self.height > 0
            and isfinite(self.resolution) and self.resolution > 0
            and isfinite(self.origin_x) and isfinite(self.origin_y)
            and data_length == self.width * self.height
        )

    def cell_from_fraction(self, fx: float, fy: float) -> tuple[int, int] | None:
        """Top-left display fractions to ROS bottom-left, row-major cell."""
        if not (0 <= fx < 1 and 0 <= fy < 1):
            return None
        return floor(fx * self.width), self.height - 1 - floor(fy * self.height)

    def center(self, col: int, row: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def fraction_from_world(self, x: float, y: float) -> tuple[float, float]:
        return (
            (x - self.origin_x) / (self.width * self.resolution),
            1 - (y - self.origin_y) / (self.height * self.resolution),
        )

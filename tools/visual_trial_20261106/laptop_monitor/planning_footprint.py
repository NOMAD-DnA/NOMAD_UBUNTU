"""Display geometry matching Pi4 result_validation.hpp (2026-11-09).

No occupancy or permission is inferred from preview image colours. These cells
are the conservative current-pose footprint on the *displayed* grid, not the
planner's local trajectory collision report. Margin is a braking parameter and
is deliberately not used to inflate this rectangle.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Footprint:
    half_length: float
    half_width: float
    center_offset: float
    reference: str = 'OAK IMU'

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.half_length, self.half_width, self.center_offset))
                or min(self.half_length, self.half_width) <= 0
                or abs(self.center_offset) >= self.half_length):
            raise ValueError('Invalid display footprint geometry')

    @property
    def front(self):
        return self.half_length + self.center_offset

    @property
    def rear(self):
        return self.half_length - self.center_offset

    def corners(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        # First edge is the front bumper. The pose is NOT the rectangle centre.
        return [(x+c*dx-s*dy, y+s*dx+c*dy) for dx, dy in (
            (self.front, self.half_width), (self.front, -self.half_width),
            (-self.rear, -self.half_width), (-self.rear, self.half_width))]

    def cells(self, x, y, yaw, grid, origin_yaw):
        """Exact conservative selection used by the Pi4 footprint_cells lambda.

        Includes outside-grid indices; drawing them does not mark them free.
        Returns None if invalid/unreasonably fine display metadata exceeds the
        bounded rendering budget, rather than freezing the UI or approximating.
        """
        resolution = grid['resolution']
        origin = grid['origin']
        c, s = math.cos(yaw), math.sin(yaw)
        cx, cy = x+self.center_offset*c, y+self.center_offset*s
        mc, ms = math.cos(origin_yaw), math.sin(origin_yaw)
        dx, dy = cx-origin['x'], cy-origin['y']
        px, py = mc*dx+ms*dy, -ms*dx+mc*dy
        rc, rs = math.cos(yaw-origin_yaw), math.sin(yaw-origin_yaw)
        ex = abs(rc)*self.half_length + abs(rs)*self.half_width
        ey = abs(rs)*self.half_length + abs(rc)*self.half_width
        extent = .5*resolution*(abs(rc)+abs(rs))
        bounds = ((px-ex)/resolution, (px+ex)/resolution,
                  (py-ey)/resolution, (py+ey)/resolution)
        if not all(math.isfinite(v) and -2147483647 < v < 2147483646 for v in bounds):
            return None
        x0, x1, y0, y1 = (math.floor(v) for v in bounds)
        if (x1-x0+1)*(y1-y0+1) > 4096:
            return None
        result = []
        for iy in range(y0, y1+1):
            for ix in range(x0, x1+1):
                gx, gy = (ix+.5)*resolution, (iy+.5)*resolution
                wx = origin['x']+mc*gx-ms*gy
                wy = origin['y']+ms*gx+mc*gy
                dx, dy = wx-cx, wy-cy
                if (abs(c*dx+s*dy) <= self.half_length+extent
                        and abs(-s*dx+c*dy) <= self.half_width+extent):
                    result.append((ix, iy))
        return result


def cell_polygons(cells, grid, origin_yaw):
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    ox, oy, r = grid['origin']['x'], grid['origin']['y'], grid['resolution']
    return [[(ox+c*gx*r-s*gy*r, oy+s*gx*r+c*gy*r)
             for gx, gy in ((ix, iy), (ix+1, iy), (ix+1, iy+1), (ix, iy+1))]
            for ix, iy in cells]


def load_footprint():
    data = json.loads((Path(__file__).parent/'config'/'planning_footprint.json').read_text())
    return Footprint(*(float(data[k]) for k in ('half_length', 'half_width', 'center_offset')),
                     reference=data['reference'])

"""Laser inverse sensor model. No world geometry or preloaded map is used."""
import math
from itertools import product

import cv2
import numpy as np


def ray_cells(x0, y0, x1, y1):
    """Integer Bresenham ray, including both endpoints."""
    dx, dy = abs(x1-x0), -abs(y1-y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2*error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


class HitEvidence:
    """Bounded XY bins retaining the full height range of every solid return.

    Box corners conservatively enclose all samples; checking every corner
    against a ground plane cannot hide a tall point through averaging.
    """
    def __init__(self, bin_size):
        self.bin_size = bin_size
        self.boxes = {}

    def add(self, point):
        key = (math.floor(point[0]/self.bin_size), math.floor(point[1]/self.bin_size))
        p = np.asarray(point)
        if key in self.boxes:
            lo, hi = self.boxes[key]
            self.boxes[key] = (np.minimum(lo, p), np.maximum(hi, p))
        else:
            self.boxes[key] = (p.copy(), p.copy())

    def __len__(self):
        return 8*len(self.boxes)

    def __iter__(self):
        for lo, hi in self.boxes.values():
            yield from product(*zip(lo, hi))


class ObservedGrid:
    def __init__(self, resolution=0.2, size=40.0, inflation=0.4, max_size=120.0,
                 soft_inflation_radius=0.0):
        if (not math.isfinite(soft_inflation_radius) or soft_inflation_radius < 0
                or (soft_inflation_radius > 0 and soft_inflation_radius <= inflation)):
            raise ValueError('soft_inflation_radius must be 0 (disabled) or greater than inflation')
        self.inflation = inflation
        self.soft_inflation_radius = soft_inflation_radius
        self.resolution = resolution
        self.origin_x = self.origin_y = -size/2
        n = math.ceil(size/resolution)
        self.log_odds = np.zeros((n, n), dtype=np.float32)
        self.seen = np.full((n, n), -np.inf, dtype=np.float64)
        self.terrain = np.full((n, n), -1, dtype=np.int8)
        self.terrain_seen = np.full((n, n), -np.inf, dtype=np.float64)
        self.terrain_passable = np.zeros((n, n), dtype=bool)
        self.max_cells = math.ceil(max_size/resolution)
        # None means missing height evidence: only free rays may clear it.
        self.hit_points = {}
        # Cover a full cell, not just its center.
        radius = inflation + resolution*math.sqrt(2)/2
        nrad = math.ceil(radius/resolution)
        yy, xx = np.mgrid[-nrad:nrad+1, -nrad:nrad+1]
        self.kernel = ((xx*resolution)**2+(yy*resolution)**2 <= radius**2).astype(np.uint8)

    @property
    def width(self):
        return self.log_odds.shape[1]

    @property
    def height(self):
        return self.log_odds.shape[0]

    def cell(self, x, y):
        return (math.floor((x-self.origin_x)/self.resolution),
                math.floor((y-self.origin_y)/self.resolution))

    def ensure_bounds(self, x, y, margin=2.0):
        """Extend with unknown cells; preserve prior world coordinates."""
        lo_x, lo_y = self.cell(x-margin, y-margin)
        hi_x, hi_y = self.cell(x+margin, y+margin)
        block = max(1, round(5.0/self.resolution))
        def grow(n):
            return math.ceil(max(0, n)/block)*block
        left, bottom = grow(-lo_x), grow(-lo_y)
        right, top = grow(hi_x-self.width+1), grow(hi_y-self.height+1)
        if self.width+left+right > self.max_cells or self.height+bottom+top > self.max_cells:
            return False
        if left or right or bottom or top:
            padding = ((bottom, top), (left, right))
            self.log_odds = np.pad(self.log_odds, padding)
            self.seen = np.pad(self.seen, padding, constant_values=-np.inf)
            self.terrain = np.pad(self.terrain, padding, constant_values=-1)
            self.terrain_seen = np.pad(self.terrain_seen, padding, constant_values=-np.inf)
            self.terrain_passable = np.pad(self.terrain_passable, padding)
            self.origin_x -= left*self.resolution
            self.origin_y -= bottom*self.resolution
            self.hit_points = {(cy+bottom, cx+left): points
                               for (cy, cx), points in self.hit_points.items()}
        return True

    def integrate(self, pose, ranges, angle_min, angle_increment, range_min,
                  range_max, stamp, usable_range=8.0):
        x, y, yaw = pose
        if not self.ensure_bounds(x, y, usable_range+1.0):
            return False
        x0, y0 = self.cell(x, y)
        free, occupied = set(), set()
        limit = min(usable_range, range_max)
        for i, value in enumerate(ranges):
            # NaNs / negative infinity / below-minimum returns are not free rays.
            if math.isnan(value) or value < range_min:
                continue
            hit = math.isfinite(value) and value < limit and value < range_max
            distance = min(value, limit)
            a = yaw+angle_min+i*angle_increment
            end = self.cell(x+distance*math.cos(a), y+distance*math.sin(a))
            cells = list(ray_cells(x0, y0, *end))
            for cx, cy in cells[:-1] if hit else cells:
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    free.add((cy, cx))
            cx, cy = end
            if hit and 0 <= cx < self.width and 0 <= cy < self.height:
                occupied.add((cy, cx))
        for cell in occupied:
            self.hit_points[cell] = None  # Legacy XY scans have no height evidence.
        # A hit wins over another ray traversing the same discretized cell.
        free -= occupied
        for cells, delta in ((free, -0.85), (occupied, 2.0)):
            if cells:
                indices = tuple(np.array(list(cells)).T)
                updated = np.clip(self.log_odds[indices]+delta, -3, 3)
                self.log_odds[indices] = np.maximum(updated, 1.0) if delta > 0 else updated
                self.seen[indices] = stamp
        self._forget_free_hits(free)
        return bool(free or occupied)

    def integrate_rays(self, origin, rays, stamp):
        """Accumulate world rays (x, y, hit[, z]), retaining solid hit evidence."""
        if not rays:
            return False
        reach = max(math.hypot(ray[0]-origin[0], ray[1]-origin[1]) for ray in rays)
        if not self.ensure_bounds(*origin, margin=reach+1.0):
            return False
        start = self.cell(*origin)
        free, occupied = set(), set()
        for ray in rays:
            x, y, hit = ray[:3]
            cells = list(ray_cells(*start, *self.cell(x, y)))
            free.update((cy, cx) for cx, cy in (cells[:-1] if hit else cells)
                        if 0 <= cx < self.width and 0 <= cy < self.height)
            cx, cy = cells[-1]
            if hit and 0 <= cx < self.width and 0 <= cy < self.height:
                occupied.add((cy, cx))
                key = (cy, cx)
                if len(ray) != 4 or not math.isfinite(ray[3]):
                    self.hit_points[key] = None
                elif key not in self.hit_points or self.hit_points[key] is not None:
                    points = self.hit_points.setdefault(key, HitEvidence(self.resolution/10))
                    points.add((x, y, ray[3]))
        free -= occupied
        for cells, delta in ((free, -0.85), (occupied, 2.0)):
            if cells:
                indices = tuple(np.array(list(cells)).T)
                updated = np.clip(self.log_odds[indices]+delta, -3, 3)
                self.log_odds[indices] = np.maximum(updated, 1.0) if delta > 0 else updated
                self.seen[indices] = stamp
        self._forget_free_hits(free)
        return bool(free or occupied)

    def _forget_free_hits(self, cells):
        for cell in cells:
            if self.log_odds[cell] < 0:
                self.hit_points.pop(cell, None)

    def reclassify_ground(self, matcher, stamp):
        """Clear old hits only when every saved 3D return is observed ground.

        Never open unknown cells, extrapolate height, or clear newer scans with
        older depth evidence. Mixed solid/ground cells remain occupied.
        """
        candidates = [(cell, points) for cell, points in self.hit_points.items()
                      if points and self.seen[cell] <= stamp and self.log_odds[cell] >= 0]
        if not candidates:
            return 0
        points = np.array([point for _, group in candidates for point in group])
        matches = matcher(points, stamp)
        offset, cleared = 0, 0
        for cell, group in candidates:
            count = len(group)
            if np.all(matches[offset:offset+count]):
                self.log_odds[cell] = -3.
                self.seen[cell] = stamp
                del self.hit_points[cell]
                cleared += 1
            offset += count
        return cleared

    def integrate_terrain(self, points, labels, stamp, dirt_cost=0, field_cost=65,
                          neutral_cost=35, min_samples=3):
        """Accumulate colour evidence only; never mark cells observed/free.

        Require multiple depth samples per cell; grass wins at mixed edges.
        New observations replace old preferences so changing appearance clears.
        """
        if len(points) == 0:
            return 0
        points, labels = np.asarray(points), np.asarray(labels)
        valid = np.all(np.isfinite(points), axis=1)
        points, labels = points[valid], labels[valid]
        cells = np.floor((points[:, :2] - [self.origin_x, self.origin_y]) / self.resolution).astype(int)
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < self.width)
                 & (cells[:, 1] >= 0) & (cells[:, 1] < self.height))
        cells, labels = cells[valid], labels[valid]
        indices = cells[:, 1]*self.width+cells[:, 0]
        unique, inverse = np.unique(indices, return_inverse=True)
        total = np.bincount(inverse, minlength=len(unique))
        dirt = np.bincount(inverse, weights=(labels == 1), minlength=len(unique))
        field = np.bincount(inverse, weights=(labels == 2), minlength=len(unique))
        costs = np.full(len(unique), neutral_cost, dtype=np.int8)
        costs[dirt >= 0.7*total] = dirt_cost
        costs[field >= 0.2*total] = field_cost
        keep = (total >= min_samples) & (stamp >= self.terrain_seen.ravel()[unique])
        self.terrain.ravel()[unique[keep]] = costs[keep]
        self.terrain_seen.ravel()[unique[keep]] = stamp
        self.terrain_passable.ravel()[unique[keep]] = False
        return int(np.count_nonzero(keep))

    def integrate_surface_costs(self, centres, costs, stamp, *, passable=False):
        """Store geometry and optional confirmed traversability separately from cost."""
        if len(centres) == 0:
            return 0
        cells = np.floor((centres-[self.origin_x, self.origin_y])/self.resolution).astype(int)
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < self.width)
                 & (cells[:, 1] >= 0) & (cells[:, 1] < self.height))
        cells, costs = cells[valid], costs[valid]
        x, y = cells.T
        fresh = stamp >= self.terrain_seen[y, x]
        self.terrain[y[fresh], x[fresh]] = costs[fresh]
        self.terrain_seen[y[fresh], x[fresh]] = stamp
        self.terrain_passable[y[fresh], x[fresh]] = passable & (costs[fresh] >= 0) & (costs[fresh] < 80)
        return int(np.count_nonzero(fresh))

    def terrain_map(self, now, max_age):
        return np.where((now-self.terrain_seen >= 0) & (now-self.terrain_seen <= max_age),
                        self.terrain, -1).astype(np.int8)

    def add_proximity_costs(self, obstacles, *layers):
        """Add a soft preference without changing blocked or unknown masks.

        Sources are UNINFLATED obstacle cells, never the already blocked ring.
        Nearest-obstacle distance implements max, not sum, for overlapping rings.
        Keep the existing hard dilation/observed-space erosion authoritative.
        """
        if self.soft_inflation_radius == 0 or not np.any(obstacles):
            return layers
        distance = cv2.distanceTransform((~np.asarray(obstacles, dtype=bool)).astype(np.uint8),
                                         cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        # Account for the source cell extent once; hard masks are untouched.
        clearance = distance*self.resolution - self.resolution*math.sqrt(2)/2
        fraction = np.clip((self.soft_inflation_radius-clearance)
                           / (self.soft_inflation_radius-self.inflation), 0., 1.)
        costs = np.rint(79*fraction**2).astype(np.int8)
        return tuple(np.where((layer >= 0) & (layer < 80),
                              np.maximum(layer, costs), layer).astype(np.int8)
                     for layer in layers)

    def local_observation_valid(self, now, stamps, max_age, robot_xy=None,
                                local_memory_seconds=0.0, local_memory_radius=3.0):
        """Reuse actual observations near the vehicle without refreshing their age.

        A moving spatial window selects existing world-fixed evidence. It never
        paints the robot footprint free or extends rays into unseen space.
        """
        age = now-stamps
        valid = (age >= 0) & (age <= max_age)
        if (robot_xy is None or local_memory_seconds <= max_age
                or not all(math.isfinite(v) for v in robot_xy)):
            return valid
        yy, xx = np.ogrid[:self.height, :self.width]
        dx = self.origin_x+(xx+.5)*self.resolution-robot_xy[0]
        dy = self.origin_y+(yy+.5)*self.resolution-robot_xy[1]
        near = dx*dx+dy*dy <= local_memory_radius**2
        return valid | (near & (age >= 0) & (age <= local_memory_seconds))

    def navigation_maps_without_lidar_obstacles(self, now, memory_seconds, local_max_age,
                                               **local_memory):
        """Keep observed free space, but use depth alone for obstacle blocking.

        LiDAR free rays retain mapping evidence up to (not including) a hit.
        An unclassified hit stays unknown, rather than becoming a free cell or
        an inflated obstacle. Depth-confirmed ground may open that hit cell.
        Neutral colour alone cannot establish free space. Explicit simulator
        grass evidence can establish traversability with a nonzero cost.
        """
        terrain = self.terrain_map(now, memory_seconds)
        blocked = cv2.dilate((terrain >= 80).astype(np.uint8), self.kernel) != 0
        scan_free = (self.log_odds < 0) & (now-self.seen >= 0)
        ground_free = (terrain == 0) | (self.terrain_passable & (terrain >= 0) & (terrain < 80))
        global_map = np.full(terrain.shape, -1, dtype=np.int8)
        global_map[scan_free | ground_free] = 0
        global_map[blocked] = 100
        scan_valid = self.local_observation_valid(now, self.seen, local_max_age, **local_memory)
        terrain_valid = self.local_observation_valid(now, self.terrain_seen, local_max_age,
                                                     **local_memory)
        fresh = ((scan_free & scan_valid) | (ground_free & terrain_valid)).astype(np.uint8)
        safe = cv2.erode(fresh, self.kernel, borderType=cv2.BORDER_CONSTANT,
                         borderValue=0) != 0
        # The simulator explicitly labels an observed grass endpoint as
        # traversable. Do not revoke that label just because surrounding
        # cells lack observations. Only these age-valid semantic cells
        # bypass unknown-space erosion; real obstacle inflation still wins.
        fresh_grass = (self.terrain_passable & (terrain >= 0) & (terrain < 80)
                       & terrain_valid)
        safe |= fresh_grass
        local_map = np.full(terrain.shape, -1, dtype=np.int8)
        local_map[safe] = 0
        local_map[blocked] = 100
        # Retain surface preference independently of passability and erosion.
        for layer in (global_map, local_map):
            known = (layer >= 0) & (layer < 80) & (terrain >= 0) & (terrain < 80)
            layer[known] = np.maximum(layer[known], terrain[known])
        return self.add_proximity_costs(terrain >= 80, global_map, local_map)

    def maps(self, now, local_max_age=2.0, **local_memory):
        raw = np.full(self.log_odds.shape, -1, dtype=np.int8)
        observed = np.isfinite(self.seen)
        raw[observed & (self.log_odds < 0)] = 0
        # Immediately mark each current hit occupied; persistent clearing uses rays.
        raw[observed & (self.log_odds >= 0)] = 100
        blocked = cv2.dilate((raw == 100).astype(np.uint8), self.kernel) != 0
        global_map = raw.copy()
        global_map[blocked] = 100
        valid = self.local_observation_valid(now, self.seen, local_max_age, **local_memory)
        fresh_free = ((raw == 0) & valid).astype(np.uint8)
        # The entire clearance disc must have age-valid free-space evidence.
        safe_free = cv2.erode(fresh_free, self.kernel, borderType=cv2.BORDER_CONSTANT,
                             borderValue=0) != 0
        local_map = np.full(raw.shape, -1, dtype=np.int8)
        local_map[safe_free] = 0
        local_map[blocked] = 100
        return raw, global_map, local_map

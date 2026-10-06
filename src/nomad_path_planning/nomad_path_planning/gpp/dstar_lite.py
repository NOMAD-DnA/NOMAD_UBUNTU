import heapq
import math


INF = float('inf')


class DStarLite:
    def __init__(
        self,
        width,
        height,
        data,
        start,
        goal,
        obstacle_threshold=80,
        unknown_penalty=2.5,
    ):
        self.edge_filter = None
        self.width = width
        self.height = height
        self.data = list(data)

        self.start = start
        self.last = start
        self.goal = goal

        self.obstacle_threshold = obstacle_threshold
        self.unknown_penalty = unknown_penalty
        self.penalties = [self.cost_for_value(value) for value in self.data]
        self.blocked = [value >= obstacle_threshold for value in self.data]
        self.neighbor_cache = {}

        self.km = 0.0

        n = width * height

        self.g = [INF] * n
        self.rhs = [INF] * n

        self.open_heap = []
        self.open_keys = {}
        self.serial = 0

        self.last_compute_expansions = 0

        self.rhs[self.index(goal)] = 0.0

        self.push(goal)

    def index(self, cell):
        x, y = cell
        return y * self.width + x

    def cell_from_index(self, index):
        x = index % self.width
        y = index // self.width
        return x, y

    def in_bounds(self, cell):
        x, y = cell

        return (
            0 <= x < self.width
            and 0 <= y < self.height
        )

    def raw_cost(self, cell):
        return self.data[self.index(cell)]

    def is_blocked(self, cell):
        if not self.in_bounds(cell):
            return True

        return self.blocked[self.index(cell)]

    def cost_for_value(self, value):
        if value < 0:
            return self.unknown_penalty
        if value >= self.obstacle_threshold:
            return INF
        return 1.0 + 4.0 * value / 79.0

    def terrain_penalty(self, cell):
        return self.penalties[self.index(cell)]

    def heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal

        return (
            math.sqrt(2.0) * diagonal
            + straight
        )

    def neighbors(self, cell):
        cached = self.neighbor_cache.get(cell)
        if cached is not None:
            return cached
        x, y = cell

        result = []

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):

                if dx == 0 and dy == 0:
                    continue

                n = (x + dx, y + dy)

                if self.in_bounds(n):
                    result.append(n)

        self.neighbor_cache[cell] = result
        return result

    def edge_cost(self, a, b):
        if self.edge_filter is not None and not self.edge_filter(a, b):
            return INF
        if not self.in_bounds(a) or not self.in_bounds(b):
            return INF
        ai, bi = self.index(a), self.index(b)
        if self.blocked[ai] or self.blocked[bi]:
            return INF

        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        # 대각선 이동 시 벽 모서리를 뚫고 가지 못하게 함
        if dx == 1 and dy == 1:
            side1 = (a[0], b[1])
            side2 = (b[0], a[1])

            if self.blocked[self.index(side1)] or self.blocked[self.index(side2)]:
                return INF

            distance = math.sqrt(2.0)

        else:
            distance = 1.0

        return distance * 0.5 * (self.penalties[ai] + self.penalties[bi])

    def calculate_key(self, cell):
        idx = self.index(cell)

        best = min(
            self.g[idx],
            self.rhs[idx]
        )

        return (
            best
            + self.heuristic(self.start, cell)
            + self.km,
            best,
        )

    def push(self, cell):
        key = self.calculate_key(cell)

        self.serial += 1

        self.open_keys[cell] = key

        heapq.heappush(
            self.open_heap,
            (
                key[0],
                key[1],
                self.serial,
                cell,
            )
        )

    def remove(self, cell):
        self.open_keys.pop(cell, None)

    def top_key(self):
        while self.open_heap:

            k1, k2, _, cell = self.open_heap[0]

            stored = self.open_keys.get(cell)

            if stored is None or stored != (k1, k2):
                heapq.heappop(self.open_heap)
                continue

            return k1, k2

        return INF, INF

    def pop(self):
        while self.open_heap:

            k1, k2, _, cell = heapq.heappop(
                self.open_heap
            )

            stored = self.open_keys.get(cell)

            if stored is None:
                continue

            if stored != (k1, k2):
                continue

            del self.open_keys[cell]

            return cell, (k1, k2)

        return None, (INF, INF)

    def update_vertex(self, u):
        if u != self.goal:

            best_rhs = INF

            for s in self.neighbors(u):

                c = self.edge_cost(u, s)

                if math.isinf(c):
                    continue

                candidate = (
                    c
                    + self.g[self.index(s)]
                )

                if candidate < best_rhs:
                    best_rhs = candidate

            self.rhs[self.index(u)] = best_rhs

        self.remove(u)

        idx = self.index(u)

        if self.g[idx] != self.rhs[idx]:
            self.push(u)

    def compute_shortest_path(self):
        self.last_compute_expansions = 0

        max_iterations = (
            self.width
            * self.height
            * 100
        )

        iterations = 0

        while (
            self.top_key()
            < self.calculate_key(self.start)
            or self.rhs[self.index(self.start)]
            != self.g[self.index(self.start)]
        ):
            iterations += 1

            if iterations > max_iterations:
                raise RuntimeError(
                    'D* Lite exceeded iteration limit'
                )

            u, old_key = self.pop()

            if u is None:
                break

            self.last_compute_expansions += 1

            new_key = self.calculate_key(u)

            if old_key < new_key:
                self.push(u)

                continue

            idx = self.index(u)

            if self.g[idx] > self.rhs[idx]:

                self.g[idx] = self.rhs[idx]

                for p in self.neighbors(u):
                    self.update_vertex(p)

            else:

                self.g[idx] = INF

                self.update_vertex(u)

                for p in self.neighbors(u):
                    self.update_vertex(p)

        return not math.isinf(
            self.g[self.index(self.start)]
        )

    def move_start(self, new_start):
        if new_start == self.start:
            return

        self.km += self.heuristic(
            self.last,
            new_start
        )

        self.start = new_start
        self.last = new_start

    def update_costmap(self, new_data):
        new_data = list(new_data)

        if len(new_data) != len(self.data):
            raise ValueError('Costmap size changed')

        changed_indices = []

        for i, (old, new) in enumerate(
            zip(self.data, new_data)
        ):
            if old != new:
                changed_indices.append(i)

        if not changed_indices:
            return []

        self.data = new_data

        for index in changed_indices:
            value = new_data[index]
            self.penalties[index] = self.cost_for_value(value)
            self.blocked[index] = value >= self.obstacle_threshold

        affected = set()

        for index in changed_indices:

            cell = self.cell_from_index(index)

            affected.add(cell)

            for n in self.neighbors(cell):
                affected.add(n)

        for cell in affected:
            self.update_vertex(cell)

        return [
            self.cell_from_index(i)
            for i in changed_indices
        ]

    def extract_path(self):
        if math.isinf(
            self.g[self.index(self.start)]
        ):
            return []

        path = [self.start]

        current = self.start

        visited = {current}

        max_steps = (
            self.width
            * self.height
        )

        for _ in range(max_steps):

            if current == self.goal:
                return path

            best = None
            best_value = INF

            for n in self.neighbors(current):

                c = self.edge_cost(
                    current,
                    n
                )

                if math.isinf(c):
                    continue

                value = (
                    c
                    + self.g[self.index(n)]
                )

                if (
                    value < best_value
                    and n not in visited
                ):
                    best_value = value
                    best = n

            if best is None:
                return []

            current = best
            visited.add(current)
            path.append(current)

        return []

    def path_cost(self, path):
        if len(path) < 2:
            return 0.0

        total = 0.0

        for a, b in zip(
            path[:-1],
            path[1:]
        ):
            total += self.edge_cost(a, b)

        return total


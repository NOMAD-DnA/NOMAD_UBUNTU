import math

from nomad_path_planning.route_guard import route_is_behind


INF = float('inf')


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class AckermannRollout:
    def __init__(
        self,
        wheelbase=0.72,
        max_steer=0.4,
        steer_samples=9,
        speed=0.5,
        dt=0.1,
        horizon=1.5,
        obstacle_threshold=80,
        unknown_penalty=2.5,
        allow_unknown=False,
        reference_offset=0.36,
    ):
        self.reference_offset = reference_offset
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.steer_samples = steer_samples

        self.speed = speed
        self.dt = dt
        self.horizon = horizon

        self.obstacle_threshold = obstacle_threshold
        self.unknown_penalty = unknown_penalty
        self.allow_unknown = allow_unknown

    def steering_candidates(self):
        if self.steer_samples <= 1:
            return [0.0]

        result = []

        for i in range(self.steer_samples):
            ratio = i / (self.steer_samples - 1)

            steer = (
                -self.max_steer
                + 2.0 * self.max_steer * ratio
            )

            result.append(steer)

        return result

    def simulate(self, start, steer, horizon=None):
        x, y, yaw = start

        trajectory = [
            (x, y, yaw)
        ]

        # Integrate at the rear axle; expose the base_link center to planners.
        x -= self.reference_offset * math.cos(yaw)
        y -= self.reference_offset * math.sin(yaw)
        step_distance = abs(self.speed) * self.dt

        if step_distance <= 0.0:
            return trajectory

        steps = max(
            1,
            int(math.ceil(
                (self.horizon if horizon is None else horizon) / step_distance
            ))
        )

        for _ in range(steps):
            x += (
                self.speed
                * math.cos(yaw)
                * self.dt
            )

            y += (
                self.speed
                * math.sin(yaw)
                * self.dt
            )

            yaw += (
                self.speed
                / self.wheelbase
                * math.tan(steer)
                * self.dt
            )

            yaw = normalize_angle(yaw)

            trajectory.append(
                (x + self.reference_offset * math.cos(yaw),
                 y + self.reference_offset * math.sin(yaw), yaw)
            )

        return trajectory

    def world_to_cell(
        self,
        x,
        y,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
    ):
        cx = int(
            math.floor(
                (x - origin_x)
                / resolution
            )
        )

        cy = int(
            math.floor(
                (y - origin_y)
                / resolution
            )
        )

        if (
            cx < 0
            or cy < 0
            or cx >= width
            or cy >= height
        ):
            return None

        return cx, cy

    def cell_value(
        self,
        x,
        y,
        costmap,
    ):
        cell = self.world_to_cell(
            x=x,
            y=y,
            width=costmap['width'],
            height=costmap['height'],
            resolution=costmap['resolution'],
            origin_x=costmap['origin_x'],
            origin_y=costmap['origin_y'],
        )

        if cell is None:
            return None

        cx, cy = cell

        index = (
            cy * costmap['width']
            + cx
        )

        return costmap['data'][index]

    def terrain_cost(
        self,
        trajectory,
        costmap,
    ):
        total = 0.0

        for x, y, _ in trajectory:
            value = self.cell_value(
                x,
                y,
                costmap,
            )

            # 지도 밖은 충돌로 처리
            if value is None:
                return INF

            # 장애물
            if value >= self.obstacle_threshold:
                return INF

            # Unknown
            if value < 0:
                if not self.allow_unknown:
                    return INF
                total += self.unknown_penalty

            else:
                total += (
                    1.0
                    + 2.0
                    * value
                    / 79.0
                )

        return total / len(trajectory)

    def nearest_path_index(
        self,
        x,
        y,
        global_path,
    ):
        best_index = 0
        best_distance = INF

        for i, point in enumerate(global_path):
            px, py = point

            d = math.hypot(
                px - x,
                py - y
            )

            if d < best_distance:
                best_distance = d
                best_index = i

        return best_index

    def lookahead_target(
        self,
        start,
        global_path,
        lookahead_distance=1.5,
    ):
        x, y, _ = start

        nearest = self.nearest_path_index(
            x,
            y,
            global_path,
        )

        accumulated = 0.0

        for i in range(
            nearest,
            len(global_path) - 1
        ):
            x1, y1 = global_path[i]
            x2, y2 = global_path[i + 1]

            accumulated += math.hypot(
                x2 - x1,
                y2 - y1
            )

            if accumulated >= lookahead_distance:
                yaw = math.atan2(
                    y2 - y1,
                    x2 - x1
                )

                return (
                    x2,
                    y2,
                    yaw,
                )

        if len(global_path) >= 2:
            x1, y1 = global_path[-2]
            x2, y2 = global_path[-1]

            yaw = math.atan2(
                y2 - y1,
                x2 - x1
            )

            return x2, y2, yaw

        px, py = global_path[-1]

        return px, py, start[2]

    def score(
        self,
        trajectory,
        steer,
        global_path,
        costmap,
    ):
        # Scoring cannot turn a forward-only candidate into reverse travel.
        if self.speed > 0 and route_is_behind(trajectory[0], global_path):
            return INF
        terrain = self.terrain_cost(
            trajectory,
            costmap,
        )

        if math.isinf(terrain):
            return INF

        start = trajectory[0]

        target_x, target_y, target_yaw = (
            self.lookahead_target(
                start,
                global_path,
                lookahead_distance=self.horizon,
            )
        )

        end_x, end_y, end_yaw = (
            trajectory[-1]
        )

        target_distance = math.hypot(
            target_x - end_x,
            target_y - end_y
        )

        heading_error = abs(
            normalize_angle(
                target_yaw - end_yaw
            )
        )

        steer_penalty = (
            abs(steer)
            / self.max_steer
            if self.max_steer > 0
            else 0.0
        )

        # 1차 테스트용 경험적 score.
        # 작을수록 좋은 trajectory.
        score = (
            4.0 * target_distance
            + 1.5 * heading_error
            + 0.6 * terrain
            + 0.3 * steer_penalty
        )

        return score

    def plan(
        self,
        start,
        global_path,
        costmap,
    ):
        if not global_path:
            return None

        candidates = []

        # A nearby final goal does not require clearance beyond that goal.
        # Keep the full horizon for reverse recovery and ordinary distant goals.
        horizon = self.horizon
        if self.speed > 0:
            horizon = min(horizon, math.hypot(global_path[-1][0]-start[0],
                                             global_path[-1][1]-start[1]))

        for steer in self.steering_candidates():
            trajectory = self.simulate(
                start,
                steer,
                horizon=horizon,
            )

            score = self.score(
                trajectory,
                steer,
                global_path,
                costmap,
            )

            candidates.append({
                'steer': steer,
                'trajectory': trajectory,
                'score': score,
            })

        valid = [
            c
            for c in candidates
            if not math.isinf(c['score'])
        ]

        if not valid:
            return {
                'success': False,
                'best': None,
                'candidates': candidates,
            }

        best = min(
            valid,
            key=lambda c: c['score']
        )

        return {
            'success': True,
            'best': best,
            'candidates': candidates,
        }

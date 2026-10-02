"""D* Lite with bounded retry, LPP feedback and validated A* recovery."""
import heapq
import math
import time

import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from nomad_path_planning.history_recovery import EntryGate
from nomad_path_planning.dstar_lite import DStarLite
from nomad_path_planning.dstar_lite_node import DStarLiteNode


def astar_path(planner):
    """Independent fallback using exactly the same occupied cells / edge rules."""
    start, goal = planner.start, planner.goal
    if planner.is_blocked(start) or planner.is_blocked(goal):
        return []
    costs, parents = {start: 0.}, {}
    queue = [(planner.heuristic(start, goal), 0., start)]
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost != costs.get(cell):
            continue
        if cell == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parents[path[-1]])
            return path[::-1]
        for nxt in planner.neighbors(cell):
            new_cost = cost + planner.edge_cost(cell, nxt)
            if new_cost < costs.get(nxt, math.inf):
                costs[nxt], parents[nxt] = new_cost, cell
                heapq.heappush(queue, (new_cost+planner.heuristic(nxt, goal), new_cost, nxt))
    return []


class RecoveryGPP(DStarLiteNode):
    def __init__(self):
        self.last_attempt = 0.
        self.retry_pending = True
        self.failed_since = None
        self.feedback_replanned = False
        self.last_report = None
        self.last_map_data = None
        super().__init__()
        self.gates = []
        self.gate_width = self.declare_parameter('recovery.gate_half_width', 2.0).value
        self.create_subscription(PoseArray, '/nomad/recovery_gates', self.on_gates,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.unknown_penalty = self.declare_parameter('unknown_penalty', 2.5).value
        self.create_subscription(Bool, '/nomad/lpp_failed', self.on_lpp, 10)
        self.create_timer(0.2, self.try_plan)

    def goal_callback(self, msg):
        old = self.goal_pose
        self.goal_pose = msg
        if old is None or (old.pose.position.x, old.pose.position.y) != (
                msg.pose.position.x, msg.pose.position.y):
            # The retry throttle for the previous goal must not delay a new one.
            self.last_attempt = -math.inf
            self.failed_since = None
            self.feedback_replanned = False
        self.try_plan()

    def on_gates(self, msg):
        if msg.header.frame_id != self.planning_frame:
            return
        self.gates = [EntryGate(p.position.x, p.position.y,
            math.atan2(2*p.orientation.w*p.orientation.z, 1-2*p.orientation.z**2), self.gate_width)
            for p in msg.poses]
        # Directional edge changes require invalidating D* state, not obstacle painting.
        self.planner = None
        self.last_attempt = -math.inf
        self.try_plan()

    def on_lpp(self, msg):
        if msg.data:
            if self.failed_since is None:
                self.failed_since = time.monotonic()
                self.feedback_replanned = False
        else:
            self.failed_since = None
            self.feedback_replanned = False

    def feedback_due(self, now):
        return (self.failed_since is not None and not self.feedback_replanned
                and now-self.failed_since >= 1.0)

    def report(self, message):
        self.publish_status(message)
        if message != self.last_report:
            self.get_logger().info(message)
            self.last_report = message

    def stop_and_retry(self, reason):
        self.retry_pending = True
        self.publish_empty_path()
        self.report(reason)

    def try_plan(self):
        if self.costmap is None or self.current_pose is None or self.goal_pose is None:
            return
        now = time.monotonic()
        if now-self.last_attempt < (1.0 if self.retry_pending else 0.2):
            return
        info = self.costmap.info
        def cell(pose):
            p = pose.pose.position
            return (math.floor((p.x-info.origin.position.x)/info.resolution),
                    math.floor((p.y-info.origin.position.y)/info.resolution))
        start, goal = cell(self.current_pose), cell(self.goal_pose)
        self.last_attempt = now
        for name, point in [('START', start), ('GOAL', goal)]:
            x, y = point
            if not (0 <= x < info.width and 0 <= y < info.height):
                self.stop_and_retry(f'GPP_WAIT: {name}_OUTSIDE_MAP')
                return
            if self.costmap.data[y*info.width+x] >= 80:
                self.stop_and_retry(f'GPP_WAIT: {name}_BLOCKED; waiting for clearance')
                return
        signature = self.map_signature()
        feedback = self.feedback_due(now)
        rebuild = (self.planner is None or signature != self.last_map_signature
                   or goal != self.planner.goal or self.retry_pending or feedback)
        changed, moved = [], False
        if rebuild:
            self.planner = DStarLite(info.width, info.height, self.costmap.data, start, goal,
                                     unknown_penalty=self.unknown_penalty)
            gates = tuple(getattr(self, 'gates', ()))
            if gates:
                resolution, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
                def permitted(a, b):
                    aw = (ox+(a[0]+.5)*resolution, oy+(a[1]+.5)*resolution)
                    bw = (ox+(b[0]+.5)*resolution, oy+(b[1]+.5)*resolution)
                    return not any(g.blocks(aw, bw) for g in gates)
                self.planner.edge_filter = permitted
            self.last_map_signature = signature
            if feedback:
                self.feedback_replanned = True
        else:
            moved = start != self.planner.start
            self.planner.move_start(start)
            # Pose/timer callbacks often see the exact same received map.
            # ROS replaces data on new messages; avoid comparing 160k cells
            # repeatedly when only the pose changed.
            if self.costmap.data is not getattr(self, 'last_map_data', None):
                changed = self.planner.update_costmap(self.costmap.data)
        self.last_map_data = self.costmap.data
        if not (rebuild or moved or changed):
            return
        try:
            success = self.planner.compute_shortest_path()
            path = self.planner.extract_path() if success else []
        except RuntimeError as error:
            self.get_logger().warn(str(error))
            path = []
        valid = bool(path) and path[-1] == goal and all(
            math.isfinite(self.planner.edge_cost(a, b)) for a, b in zip(path, path[1:]))
        fallback = not valid
        if fallback:
            # A failed incremental state must not leave the stopped car stuck forever.
            path = astar_path(self.planner)
        if not path:
            self.stop_and_retry('GPP_WAIT: NO_PATH; retrying once per second')
            return
        self.retry_pending = False
        self.publish_path(path)
        mode = 'ASTAR_RECOVERY' if fallback else ('LPP_REPLAN' if feedback else 'DSTAR_OK')
        self.report(f'{mode}: {len(path)} cells')
        if fallback:
            # Reinitialize D* on the next update; never reuse an inconsistent cache.
            self.planner = None


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryGPP()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

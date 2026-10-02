from nomad_path_planning.contracts import valid_grid
from nomad_interfaces.msg import PlannedMotion
"""History-based dead-end recovery, with one atomic priority Path channel.

recovery_path: [] releases control; one pose holds STOP; >=2 poses is a
collision-checked signed recovery rollout (reverse or exit). A stale channel remains stopped, never silently
falls back to an older normal rollout. No frontier extraction or world truth.
"""
import math
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool, String
from nomad_path_planning.history_recovery import Trail, ProgressWindow, ReverseFollower, TrajectoryFollower, EntryGate, distance, segment_free, allowed_path

from nomad_path_planning.escape_probe import EscapeProbe, ExitAdjustment
from nomad_path_planning.route_guard import route_is_behind


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


class RecoverySupervisor(Node):
    def __init__(self):
        super().__init__('nomad_recovery_supervisor')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = p('planning_frame', 'odom')
        self.behind_seconds = float(p('recovery.behind_seconds', .3))
        self.behind_since = None
        self.min_retreat = float(p('recovery.min_retreat', 2.0))
        self.max_retreat = float(p('recovery.max_retreat', 60.0))
        self.gate_width = float(p('recovery.gate_half_width', 2.0))
        self.gate_ttl = float(p('recovery.gate_ttl', 120.0))
        self.max_attempts = int(p('recovery.max_attempts', 4))
        self.recovery_timeout = float(p('recovery.timeout', 360.0))
        self.speed = float(p('recovery.speed', .2))
        self.wheelbase = float(p('wheelbase', .72))
        self.max_steer = float(p('max_steer', .4))
        self.offset = float(p('reference_offset', .36))
        self.exit_speed = float(p('recovery.exit_speed', .3))
        self.exit_horizon = float(p('recovery.exit_horizon', 3.0))
        self.exit_separation = float(p('recovery.exit_separation', 1.0))
        self.adjust_length = float(p('recovery.adjust_length', .6))
        self.adjust_limit = int(p('recovery.adjust_attempts', 3))
        self.exit_blocked_seconds = float(p('recovery.exit_blocked_seconds', .3))
        window = float(p('recovery.stuck_seconds', 4.0))
        radius = float(p('recovery.stuck_radius', .3))
        if (not all(math.isfinite(v) and v > 0 for v in
                    [self.behind_seconds, self.min_retreat, self.max_retreat, self.gate_width, self.gate_ttl,
                     self.recovery_timeout, self.speed, self.wheelbase, self.max_steer,
                     self.exit_speed, self.exit_horizon, self.exit_separation, window, radius,
                     self.adjust_length, self.exit_blocked_seconds])
                or self.max_retreat < self.min_retreat or self.max_attempts < 1
                or self.adjust_limit < 1):
            raise ValueError('Invalid recovery parameters')
        self.probe = EscapeProbe(self.wheelbase, self.max_steer, self.offset,
                                 self.exit_speed, self.exit_horizon, self.exit_separation)
        self.adjuster = ExitAdjustment(self.probe, self.speed, self.adjust_length)
        self.adjust_count = 0
        self.adjust_follower = self.resume_follower = None
        self.exit_blocked_since = None
        self.last_probe = -math.inf
        self.exit_follower = None
        self.progress = ProgressWindow(window, radius)
        self.trail = Trail()
        self.pose = self.grid = self.goal = None
        self.pose_msg = None
        self.pose_at = self.map_at = self.health_at = -math.inf
        self.health = False
        self.mode = 'FOLLOW'
        self.gates = []
        self.attempts = 0
        self.follower = None
        self.previous_time = None
        self.last_status = None
        self.last_pose = None
        self.resume_after = 0.
        self.global_path = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/nomad/recovery_path', 10)
        self.motion_pub = self.create_publisher(PlannedMotion, '/nomad/planning/recovery_motion', 10)
        self.reference_pub = self.create_publisher(Path, '/nomad/recovery_reference', latched)
        self.gate_pub = self.create_publisher(PoseArray, '/nomad/recovery_gates', latched)
        self.status_pub = self.create_publisher(String, '/nomad/recovery_status', 10)
        self.create_subscription(PoseStamped, '/nomad/current_pose', self.on_pose, 10)
        self.create_subscription(OccupancyGrid, '/nomad/local_costmap', self.on_map, latched)
        self.create_subscription(PoseStamped, '/nomad/goal', self.on_goal, latched)
        self.create_subscription(Bool, '/nomad/sensors_ready', self.on_health, 10)
        self.create_subscription(Path, '/nomad/global_path', self.on_global, latched)
        self.create_timer(.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_pose(self, msg):
        if msg.header.frame_id != self.frame:
            return
        p = msg.pose.position
        q = msg.pose.orientation
        values = [p.x, p.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[2:]) < .5:
            return
        pose = (p.x, p.y, yaw(q))
        if self.last_pose and distance(pose, self.last_pose) > 1.0:
            self.mode = 'FAULT'
            self.trail.points.clear()
        self.pose, self.last_pose = pose, pose
        self.pose_msg = msg
        self.pose_at = stamp(msg)

    def on_map(self, msg):
        if not valid_grid(msg,self.frame):
            self.grid = None
            return
        i = msg.info
        self.grid = dict(width=i.width, height=i.height, resolution=i.resolution,
                         origin_x=i.origin.position.x, origin_y=i.origin.position.y, data=list(msg.data))
        self.map_at = stamp(msg)

    def on_health(self, msg):
        self.health, self.health_at = msg.data, self.now()

    def on_global(self, msg):
        self.global_path = msg

    def on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in goal):
            return
        if goal == self.goal:
            return
        self.goal = goal
        self.behind_since = None
        self.mode, self.follower, self.attempts = 'FOLLOW', None, 0
        self.exit_follower = None
        self.last_probe = -math.inf
        self.adjust_count = 0
        self.adjust_follower = self.resume_follower = None
        self.exit_blocked_since = None
        self.trail.points.clear()
        self.progress.clear()
        self.gates.clear()
        self.publish_gates()
        self.publish_path([])
        self.reference_pub.publish(self.path_message([]))
        self.resume_after = self.now()+.3

    def path_message(self, poses):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, heading in poses:
            p = PoseStamped(header=deepcopy(msg.header))
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            msg.poses.append(p)
        return msg

    def publish_path(self, poses):
        path = self.path_message(poses)
        self.path_pub.publish(path)
        motion = PlannedMotion(path=path, active=self.mode != 'FOLLOW')
        if len(poses) >= 2:
            follower = (self.adjust_follower if self.mode == 'ADJUST_REVERSE' else
                        self.exit_follower if self.mode == 'EXIT' else self.follower)
            motion.target_speed = float(follower.model.speed)
            motion.target_steering_angle = float(follower.last_steer)
        self.motion_pub.publish(motion)

    def hold(self, reason):
        if self.pose is not None:
            self.publish_path([self.pose])
        self.report(reason)

    def report(self, message):
        self.status_pub.publish(String(data=message))
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def publish_gates(self):
        msg = PoseArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for gate, _ in self.gates:
            p = Pose()
            p.position.x, p.position.y = gate.x, gate.y
            p.orientation.z, p.orientation.w = math.sin(gate.yaw/2), math.cos(gate.yaw/2)
            msg.poses.append(p)
        self.gate_pub.publish(msg)

    def tick(self):
        now = self.now()
        if self.previous_time is not None and now < self.previous_time:
            self.mode = 'FAULT'
            self.trail.points.clear()
        self.previous_time = now
        if self.mode == 'FAULT':
            self.hold('RECOVERY_FAULT: pose/clock reset; restart planning')
            return
        if self.pose is None or self.grid is None or self.goal is None:
            return
        healthy = (self.health and all(-.05 <= now-t <= .6 for t in [self.health_at, self.pose_at, self.map_at]))
        if not healthy:
            self.behind_since = None
            self.exit_blocked_since = None
            self.progress.clear()
            if self.mode != 'FOLLOW':
                self.hold('RECOVERY_PAUSED: fresh sensors required')
            return
        remaining_gates = [(g, t) for g, t in self.gates if t > now]
        if self.mode == 'FOLLOW' and len(remaining_gates) != len(self.gates):
            self.gates = remaining_gates
            self.publish_gates()
        if self.mode == 'STOP':
            self.hold('RECOVERY_STOP: exhausted history/attempts; new goal or restart required')
            return
        if self.mode in ('REVERSE', 'SWITCH', 'EXIT', 'ADJUST_STOP', 'ADJUST_REVERSE'):
            if now-self.started > self.recovery_timeout:
                self.mode = 'STOP'
                self.hold('RECOVERY_STOP: maneuver timeout')
                return
            if self.mode == 'ADJUST_STOP':
                self.hold('RECOVERY_ADJUST_STOP: brake before a short reverse turn')
                if now-self.switch_at >= .3:
                    self.mode = 'ADJUST_REVERSE'
                return
            if self.mode == 'ADJUST_REVERSE':
                if self.adjust_follower.advance(self.pose):
                    # Re-plan from the actual reached pose, not the predicted endpoint.
                    exit_path = self.find_adjusted_exit(now)
                    if exit_path is None:
                        self.resume_retreat(now)
                    else:
                        self.exit_follower = TrajectoryFollower(exit_path, self.wheelbase,
                            self.max_steer, self.offset, speed=self.exit_speed)
                        self.mode, self.switch_at = 'SWITCH', now
                        self.hold('RECOVERY_SWITCH: adjusted heading has a verified exit')
                    return
                trajectory = self.adjust_follower.plan(self.pose, self.grid)
                if trajectory is None or not allowed_path(trajectory, [g for g,_ in self.gates]):
                    self.resume_retreat(now)
                else:
                    self.publish_path(trajectory)
                    self.report('RECOVERY_ADJUST_REVERSE: short reverse turn before retrying exit')
                return
            if self.mode == 'SWITCH':
                self.hold('RECOVERY_SWITCH: stop before taking the verified exit')
                if now-self.switch_at >= .3:
                    # Revalidate the entire exit after braking, not just its first step.
                    ref = self.exit_follower.reference
                    if (allowed_path(ref, [g for g,_ in self.gates])
                            and all(segment_free(self.grid,a,b) for a,b in zip(ref,ref[1:]))):
                        self.mode = 'EXIT'
                        self.exit_blocked_since = None
                        RecoverySupervisor.trim_trail(self)
                        self.reference_pub.publish(self.path_message(ref))
                    else:
                        self.resume_retreat(now)
                return
            if self.mode == 'EXIT':
                self.trail.record(self.pose, self.grid)
                if self.exit_follower.advance(self.pose):
                    self.mode = 'FOLLOW'
                    self.progress.clear()
                    self.gates = [(g, now+self.gate_ttl) for g, _ in self.gates]
                    self.publish_gates()
                    self.publish_path([])
                    self.report('RECOVERY_COMPLETE: entered a different drivable branch')
                    return
                trajectory = self.exit_follower.plan(self.pose, self.grid)
                if trajectory is None or not allowed_path(trajectory, [g for g,_ in self.gates]):
                    self.hold('RECOVERY_EXIT_BLOCKED: verified exit no longer clear')
                    if getattr(self, 'exit_blocked_since', None) is None:
                        self.exit_blocked_since = now
                    if now-self.exit_blocked_since+1e-6 >= self.exit_blocked_seconds:
                        self.retry_exit(now)
                else:
                    self.exit_blocked_since = None
                    self.publish_path(trajectory)
                    self.report('RECOVERY_EXIT: following the verified forward maneuver')
                return
            reached_end = self.follower.advance(self.pose)
            # Probe while backing. A grid route alone never ends recovery.
            if now-self.last_probe >= .5:
                self.last_probe = now
                exit_path = self.find_exit(now)
                if exit_path is not None:
                    self.exit_follower = TrajectoryFollower(exit_path, self.wheelbase,
                        self.max_steer, self.offset, speed=self.exit_speed)
                    self.mode, self.switch_at = 'SWITCH', now
                    self.hold('RECOVERY_SWITCH: a drivable alternative branch is verified')
                    return
            if reached_end:
                self.hold('RECOVERY_HISTORY_END: no earlier history; waiting for a drivable exit')
                return
            trajectory = self.follower.plan(self.pose, self.grid)
            if trajectory is None:
                self.hold('RECOVERY_BLOCKED: rear unknown/obstacle or trail deviation')
            else:
                self.publish_path(trajectory)
                self.report('RECOVERY_REVERSE: backing until a drivable exit is found')
            return
        # FOLLOW: no goal progress detector while already arrived or while inputs settle.
        if distance(self.pose, self.goal) < .35 or now < self.resume_after:
            self.behind_since = None
            self.progress.clear()
            return
        self.trail.record(self.pose, self.grid)
        # Initial planning may take longer than the stuck window. Only arm the
        # detector after a connected, usable retreat history exists; otherwise
        # normal GPP/LPP control must remain free to start when planning finishes.
        if self.trail.retreat(self.pose, self.min_retreat, self.max_retreat) is None:
            self.behind_since = None
            self.progress.clear()
            self.report('RECOVERY_WAIT_HISTORY: normal planner retains control')
            return
        msg = getattr(self, 'global_path', None)
        rearward = False
        if (msg is not None and msg.poses and msg.header.frame_id == self.frame
                and -.05 <= now-stamp(msg) <= 3.):
            route = [(p.pose.position.x,p.pose.position.y) for p in msg.poses]
            rearward = distance(route[-1], self.goal) <= .35 and route_is_behind(self.pose, route)
        if rearward:
            self.progress.clear()
            if getattr(self, 'behind_since', None) is None:
                self.behind_since = now
            self.report('RECOVERY_PATH_BEHIND: confirming rearward route while stopped')
            if now-self.behind_since+1e-6 >= self.behind_seconds:
                self.begin(now)
            return
        self.behind_since = None
        self.report('RECOVERY_MONITORING: retreat history available')
        if self.progress.stalled(now, self.pose):
            self.begin(now)

    def find_exit(self, now):
        msg = self.global_path
        if (msg is None or len(msg.poses) < 2 or msg.header.frame_id != self.frame
                or stamp(msg)+1e-6 < self.started or not -.05 <= now-stamp(msg) <= 3.):
            return None
        route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if (distance(route[-1], self.goal) > .35 or distance(route[0], self.pose) > .6
                or not all(math.isfinite(v) for p in route for v in p)):
            return None
        return self.probe.plan(self.pose, route, self.grid, self.follower.reference,
                               [g for g, _ in self.gates])

    def adjustment_route(self, now):
        msg = self.global_path
        if (msg is None or len(msg.poses) < 2 or msg.header.frame_id != self.frame
                or stamp(msg)+1e-6 < self.started or stamp(msg) > now+.05):
            return None
        route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if (not all(math.isfinite(v) for p in route for v in p)
                or distance(route[-1], self.goal) > .35):
            return None
        # GPP is event-driven: a stopped vehicle may have an old but unchanged
        # route. It is guidance only here; both maneuver legs use the fresh local
        # grid and current gates. The probe checks proximity and forward progress.
        return route

    def find_adjusted_exit(self, now):
        route = self.adjustment_route(now)
        if route is None:
            return None
        return self.probe.plan(self.pose, route, self.grid, self.follower.reference,
                               [g for g,_ in self.gates], start_tolerance=.8)

    def retry_exit(self, now):
        route = self.adjustment_route(now)
        selected = self.trail.retreat(self.pose, self.min_retreat, self.max_retreat)
        choice = None
        if route is not None and selected is not None and self.adjust_count < self.adjust_limit:
            choice = self.adjuster.plan(self.pose, route, self.grid,
                                       self.follower.reference, [g for g,_ in self.gates])
        if choice is None:
            # The branch itself may be blocked: retreat over the actual driven
            # exit history and mark this failed approach, instead of retrying it forever.
            self.begin(now)
            return
        reverse, _ = choice
        self.resume_follower = ReverseFollower(selected[1], self.wheelbase,
            self.max_steer, self.offset, self.speed)
        self.adjust_follower = ReverseFollower(reverse, self.wheelbase,
            self.max_steer, self.offset, self.speed, tolerance=.015)
        self.adjust_count += 1
        self.mode, self.switch_at = 'ADJUST_STOP', now
        self.reference_pub.publish(self.path_message(reverse))
        self.hold('RECOVERY_ADJUST_STOP: reverse and subsequent forward exit verified')

    def resume_retreat(self, now):
        # This reference includes the actual forward exit leg before adjustment.
        # Advance the cursor so the short reverse is not undone by a forward jump.
        if getattr(self, 'resume_follower', None) is not None:
            self.follower = self.resume_follower
            self.follower.advance(self.pose)
        self.mode = 'REVERSE'
        self.exit_follower = self.adjust_follower = self.resume_follower = None
        self.last_probe = now
        self.exit_blocked_since = None
        self.reference_pub.publish(self.path_message(self.follower.reference))
        self.hold('RECOVERY_RETRY_RETREAT: continue backing to another usable exit')

    def trim_trail(self):
        if not self.trail.points:
            return
        nearest = min(range(len(self.trail.points)),
                      key=lambda i: distance(self.pose, self.trail.points[i].pose))
        self.trail.points = self.trail.points[:nearest+1]
        # Keep the last recorded point behind the reached pose; otherwise a
        # small reverse correction could erase all connected forward history.
        while len(self.trail.points) > 1:
            p = self.trail.points[-1].pose
            along = (self.pose[0]-p[0])*math.cos(p[2])+(self.pose[1]-p[1])*math.sin(p[2])
            if along >= -.03:
                break
            self.trail.points.pop()

    def begin(self, now):
        self.progress.clear()
        selected = self.trail.retreat(self.pose, self.min_retreat, self.max_retreat)
        if selected is None and self.mode == 'FOLLOW':
            # Never latch a recovery STOP merely because normal driving has not
            # accumulated enough history. An active recovery still keeps control on failure.
            self.report('RECOVERY_WAIT_HISTORY: normal planner retains control')
            return
        if selected is None or self.attempts >= self.max_attempts:
            self.mode = 'STOP'
            self.hold('RECOVERY_STOP: no connected retreat history or attempt limit')
            return
        self.target_index, reference = selected
        self.follower = ReverseFollower(reference, self.wheelbase, self.max_steer, self.offset, self.speed)
        self.mode, self.started = 'REVERSE', now
        self.attempts += 1
        self.last_probe = -math.inf
        self.exit_follower = None
        self.adjust_count = 0
        self.adjust_follower = self.resume_follower = None
        self.exit_blocked_since = None
        # Remember the failed stem once, about 1m behind the blocked position.
        # Crossing outward is allowed; only re-entry toward the failure is banned.
        arc = 0.
        anchor = reference[-1]
        for a, b in zip(reference, reference[1:]):
            arc += distance(a, b)
            if arc >= 1.:
                anchor = b
                break
        self.gates.append((EntryGate(anchor[0], anchor[1], anchor[2], self.gate_width), now+self.gate_ttl))
        self.publish_gates()
        self.reference_pub.publish(self.path_message(reference))
        self.hold('RECOVERY_START: stop before reversing')


def main(args=None):
    rclpy.init(args=args)
    node = RecoverySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.pose is not None:
            node.hold('RECOVERY_STOP: supervisor shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

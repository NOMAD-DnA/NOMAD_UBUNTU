from nomad_path_planning.contracts import valid_grid
from nomad_interfaces.msg import PlannedMotion
import math
from nomad_path_planning.route_guard import route_is_behind
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped, PoseArray
from nomad_path_planning.history_recovery import EntryGate, allowed_path

from std_msgs.msg import Bool
from std_msgs.msg import String

from nomad_path_planning.rollout import AckermannRollout


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('ackermann_rollout_lpp')

        self.planning_frame = self.declare_parameter('planning_frame', 'odom').value
        self.costmap_msg = None
        self.pose_msg = None
        self.global_path_msg = None
        self.allow_unknown = self.declare_parameter('allow_unknown', False).value
        self.forward_speed = self.declare_parameter('forward_speed', 0.4).value
        self.wheelbase = self.declare_parameter('wheelbase', 0.72).value
        self.max_steer = self.declare_parameter('max_steer', 0.4).value
        self.reference_offset = self.declare_parameter('reference_offset', 0.36).value
        self.require_sensor_health = self.declare_parameter('require_sensor_health', True).value
        self.sensor_health_timeout = self.declare_parameter('sensor_health_timeout', 1.0).value
        self.sensor_ready = not self.require_sensor_health
        self.sensor_ready_at = -math.inf

        self.planner = AckermannRollout(
            wheelbase=self.wheelbase,
            max_steer=self.max_steer,
            steer_samples=9,
            speed=self.forward_speed,
            dt=0.1,
            horizon=1.5,
            allow_unknown=self.allow_unknown,
            reference_offset=self.reference_offset,
        )

        self.create_subscription(
            OccupancyGrid,
            '/nomad/costmap',
            self.costmap_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.create_subscription(
            PoseStamped,
            '/nomad/current_pose',
            self.pose_callback,
            10,
        )

        self.create_subscription(
            Path,
            '/nomad/global_path',
            self.global_path_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        if self.require_sensor_health:
            self.create_subscription(Bool, '/nomad/sensors_ready', self.on_sensor_health, 10)

        self.local_path_pub = self.create_publisher(
            Path,
            '/nomad/local_path',
            10,
        )

        self.motion_pub = self.create_publisher(PlannedMotion, '/nomad/planning/local_motion', 10)

        self.failed_pub = self.create_publisher(
            Bool,
            '/nomad/lpp_failed',
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            '/nomad/lpp_status',
            10,
        )

        self.managed_recovery = self.declare_parameter('managed_recovery', False).value
        self.gates = []
        self.gate_width = self.declare_parameter('recovery.gate_half_width', 2.0).value
        self.create_subscription(PoseArray, '/nomad/recovery_gates', self.on_gates,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_status = None
        self.reverse_origin = None
        self.reverse_last = None
        self.reverse_distance = 0.0
        self.recovery_count = 0
        self.forward_origin = None
        self.last_goal = None
        self.create_subscription(PoseStamped, '/nomad/goal', self.on_goal,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_log_time = 0.0
        # Planning latency is wall-clock; trajectory and freshness remain sim-time.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.try_plan, clock=self.wall_clock)

        self.get_logger().info(
            'Ackermann Rollout LPP started'
        )

    def on_gates(self, msg):
        if msg.header.frame_id == self.planning_frame:
            self.gates = [EntryGate(p.position.x, p.position.y,
                self.quaternion_to_yaw(p.orientation), self.gate_width) for p in msg.poses]

    def costmap_callback(self, msg):
        if not valid_grid(msg,self.planning_frame):
            self.costmap_msg = None
            self.publish_empty_path()
            return
        self.costmap_msg = msg

    def on_sensor_health(self, msg):
        self.sensor_ready = msg.data
        self.sensor_ready_at = self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        if goal != self.last_goal:
            self.last_goal = goal
            self.reverse_origin = None
            self.reverse_last = None
            self.reverse_distance = 0.0
            self.recovery_count = 0
            self.forward_origin = None

    def pose_callback(self, msg):
        self.pose_msg = msg

    def global_path_callback(self, msg):
        self.global_path_msg = msg

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def make_costmap(self):
        info = self.costmap_msg.info

        return {
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin_x': info.origin.position.x,
            'origin_y': info.origin.position.y,
            'data': list(self.costmap_msg.data),
        }

    def make_global_path(self):
        result = []

        for pose in self.global_path_msg.poses:
            result.append(
                (
                    pose.pose.position.x,
                    pose.pose.position.y,
                )
            )

        return result

    def publish_failed(self, failed):
        msg = Bool()
        msg.data = failed

        self.failed_pub.publish(msg)

    def publish_status(self, text):
        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

        status_kind = text.split(':', 1)[0]
        now = time.monotonic()
        if status_kind != self.last_status or now - self.last_log_time >= 2.0:
            self.get_logger().info(text)
            self.last_status = status_kind
            self.last_log_time = now

    def publish_local_path(self, trajectory, steer=0.0, speed=0.0):
        msg = Path()

        msg.header.frame_id = self.planning_frame
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for x, y, yaw in trajectory:
            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = (
                math.sin(yaw / 2.0)
            )

            pose.pose.orientation.w = (
                math.cos(yaw / 2.0)
            )

            msg.poses.append(pose)

        self.local_path_pub.publish(msg)
        motion = PlannedMotion(path=msg, active=True,
                               target_speed=float(speed), target_steering_angle=float(steer))
        self.motion_pub.publish(motion)

    def publish_empty_path(self):
        msg = Path()

        msg.header.frame_id = self.planning_frame
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.local_path_pub.publish(msg)
        self.motion_pub.publish(PlannedMotion(path=msg, active=True))

    def try_plan(self):
        if self.require_sensor_health:
            now = self.get_clock().now().nanoseconds * 1e-9
            if not self.sensor_ready or now-self.sensor_ready_at > self.sensor_health_timeout:
                self.publish_empty_path()
                self.publish_failed(False)
                self.publish_status('LPP_PAUSED: required navigation sensors unavailable')
                return
        if (
            self.costmap_msg is None
            or self.pose_msg is None
            or self.global_path_msg is None
        ):
            return

        if not self.global_path_msg.poses:
            self.publish_empty_path()
            self.publish_failed(True)
            return

        pose = self.pose_msg.pose

        start = (
            pose.position.x,
            pose.position.y,
            self.quaternion_to_yaw(
                pose.orientation
            ),
        )

        global_path = (
            self.make_global_path()
        )

        costmap = self.make_costmap()

        if self.forward_origin is not None and math.hypot(
            start[0]-self.forward_origin[0], start[1]-self.forward_origin[1]) > 1.0:
            self.recovery_count = 0
            self.forward_origin = None

        if self.reverse_origin is not None:
            self.reverse_distance += math.hypot(start[0]-self.reverse_last[0], start[1]-self.reverse_last[1])
            self.reverse_last = start
            if self.reverse_distance < 0.57:
                self.reverse_recovery(start, global_path, costmap, 0.6-self.reverse_distance)
                return
            self.reverse_origin = None
            self.forward_origin = start

        result = self.planner.plan(
            start=start,
            global_path=global_path,
            costmap=costmap,
        )

        if result and result['success'] and getattr(self, 'gates', []):
            valid = [candidate for candidate in result['candidates']
                     if math.isfinite(candidate['score']) and allowed_path(candidate['trajectory'], self.gates)]
            result['success'] = bool(valid)
            if valid:
                result['best'] = min(valid, key=lambda candidate: candidate['score'])

        if (
            result is None
            or not result['success']
        ):
            if getattr(self, 'managed_recovery', False):
                self.publish_failed(True)
                self.publish_empty_path()
                self.publish_status('LPP_PATH_BEHIND: forward motion rejected; waiting for recovery'
                                    if route_is_behind(start, global_path) else
                                    'LPP_FAILED: waiting for history recovery supervisor')
                return
            if self.recovery_count < 3:
                self.reverse_origin = start
                self.reverse_last = start
                self.reverse_distance = 0.0
                self.recovery_count += 1
                self.reverse_recovery(start, global_path, costmap, 0.6)
                return
            self.publish_failed(True)
            self.publish_empty_path()

            self.publish_status(
                'LPP_FAILED: no collision-free trajectory'
            )

            return

        best = result['best']

        self.publish_failed(False)

        self.publish_local_path(
            best['trajectory'], best['steer'], self.planner.speed
        )

        valid_count = sum(
            1
            for candidate in result['candidates']
            if not math.isinf(
                candidate['score']
            )
        )

        steer_deg = math.degrees(
            best['steer']
        )

        self.publish_status(
            (
                f'LPP_OK: '
                f'steer={steer_deg:.1f} deg, '
                f'score={best["score"]:.2f}, '
                f'valid={valid_count}/'
                f'{len(result["candidates"])}'
            )
        )

    def reverse_recovery(self, start, global_path, costmap, remaining):
        reverse = AckermannRollout(wheelbase=self.wheelbase, max_steer=self.max_steer,
            steer_samples=9, speed=-min(0.3, self.forward_speed), dt=0.1,
            horizon=max(0.03, remaining), allow_unknown=self.allow_unknown,
            reference_offset=self.reference_offset)
        result = reverse.plan(start, global_path, costmap)
        if not result['success']:
            self.publish_failed(True)
            self.publish_empty_path()
            valid_count = sum(
                1 for candidate in result['candidates']
                if math.isfinite(candidate['score'])
            )
            self.publish_status(
                'LPP_FAILED: forward and reverse blocked; '
                f'reverse_valid={valid_count}/{len(result["candidates"])}; '
                f'allow_unknown={self.allow_unknown}; waiting for clearance'
            )
            return
        # Prefer a reverse endpoint from which a forward rollout is feasible.
        candidates = [c for c in result['candidates'] if math.isfinite(c['score'])]

        def rank(candidate):
            forward = self.planner.plan(candidate['trajectory'][-1], global_path, costmap)
            return (not forward['success'], abs(candidate['steer']), candidate['score'])

        best = min(candidates, key=rank)
        self.publish_failed(False)
        self.publish_local_path(best['trajectory'], best['steer'], reverse.speed)
        self.publish_status(f'LPP_RECOVERY: reversing {remaining:.2f} m, attempt {self.recovery_count}/3')


def main(args=None):
    rclpy.init(args=args)

    node = LocalPlannerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

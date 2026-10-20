"""Planning owns motion arbitration and publishes one explicit vehicle command."""
from nomad_path_planning.contracts import valid_grid, valid_pose
from nomad_path_planning.history_recovery import segment_free

import math
import time
from nomad_path_planning.observed_guard import observed_distance, speed_limit

from geometry_msgs.msg import PoseStamped
from nomad_interfaces.msg import DriveCommand, VehicleState, ModuleStatus, PlannedMotion
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from nomad_path_planning.route_guard import path_is_behind, path_moves_forward


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class PlanningCommandNode(Node):
    def __init__(self):
        super().__init__('nomad_planning_command')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.max_steer = self.declare_parameter('max_steer', .4).value
        self.vehicle_state = None
        self.vehicle_wall = -math.inf
        self.local_motion = self.recovery_motion = None
        self.allow_unknown = self.declare_parameter('allow_unknown', False).value
        self.max_speed = self.declare_parameter('max_speed', 0.4).value
        self.observed_guard = self.declare_parameter('observed_guard.enabled', True).value
        self.guard_half_length = self.declare_parameter('observed_guard.half_length', .52).value
        self.guard_half_width = self.declare_parameter('observed_guard.half_width', .40).value
        self.guard_deceleration = self.declare_parameter('observed_guard.deceleration', .4).value
        self.guard_latency = self.declare_parameter('observed_guard.latency', .3).value
        self.guard_margin = self.declare_parameter('observed_guard.margin', .1).value
        self.guard_wheelbase = self.declare_parameter('wheelbase', .72).value
        self.guard_offset = self.declare_parameter('reference_offset', .36).value
        if (not all(math.isfinite(v) for v in (self.guard_half_length,self.guard_half_width,self.guard_deceleration,
                self.guard_latency,self.guard_margin,self.guard_wheelbase,self.guard_offset))
                or min(self.guard_half_length,self.guard_half_width,self.guard_deceleration,self.guard_wheelbase)<=0
                or min(self.guard_latency,self.guard_margin,self.guard_offset)<0):
            raise ValueError('Invalid observed-space guard parameters')

        self.health_label = self.declare_parameter('health_label', 'odom/scan/TF').value
        self.sync_ok = False
        self.sync_received_at = 0.0
        self.costmap = None
        self.pose = None
        self.goal = None
        self.navigation_target = None
        self.goal_mode = self.declare_parameter('navigation.goal_mode','direct').value
        self.recovery_active = False
        self.recovery_path = None
        self.recovery_received_at = -math.inf
        self.goal_epoch = -math.inf
        self.planning_frame = self.declare_parameter('planning_frame', 'odom').value
        self.global_path = None
        self.pending_global = None
        self.local_path = None
        self.pose_received_at = 0.0
        self.local_received_at = 0.0
        self.last_status = None
        self.status_pub = self.create_publisher(String, '/nomad/planning_status', 10)
        self.cmd_pub = self.create_publisher(DriveCommand, '/nomad/planning/drive_command', 1)
        self.module_pub = self.create_publisher(ModuleStatus, '/nomad/planning/status', 10)
        self.create_subscription(VehicleState, '/nomad/control/vehicle_state', self.on_vehicle, 10)
        self.create_subscription(PoseStamped, '/nomad/current_pose', self.on_pose, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, '/nomad/goal', self.on_goal, latched)
        if self.goal_mode=='observed_corridor':
            self.create_subscription(PoseStamped,'/nomad/navigation_target',self.on_navigation_target,latched)
        self.create_subscription(Path, '/nomad/global_path', self.on_global_path, latched)
        self.create_subscription(PlannedMotion, '/nomad/planning/local_motion', self.on_local_motion, 10)
        self.create_subscription(PlannedMotion, '/nomad/planning/recovery_motion', self.on_recovery_motion, 10)
        self.create_subscription(Bool, '/nomad/sensors_ready', self.on_sync, 10)
        self.create_subscription(OccupancyGrid, '/nomad/costmap', self.on_costmap, latched)
        # NOMAD's command watchdog expires in WALL time, including slow Gazebo.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_sim_time = None
        self.last_clock_change = time.monotonic()
        self.create_timer(0.1, self.control, clock=self.wall_clock)

    def command_message(self, speed=0., steer=0., stop=False):
        msg = DriveCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.target_speed, msg.target_steering_angle = float(speed), float(steer)
        msg.valid_for.nanosec = 300_000_000
        msg.stop_requested = stop
        return msg

    def stop(self):
        self.cmd_pub.publish(self.command_message(stop=True))

    def on_vehicle(self, msg):
        self.vehicle_state, self.vehicle_wall = msg, time.monotonic()
        if not msg.ready or msg.fault:
            self.stop()

    def on_local_motion(self, msg):
        if self.recovery_active:
            return
        self.local_motion = msg
        self.on_local_path(msg.path)

    def on_recovery_motion(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = msg.path.header.stamp.sec+msg.path.header.stamp.nanosec*1e-9
        if msg.path.header.frame_id != self.planning_frame or stamp < self.goal_epoch or not -.05 <= now-stamp <= .5:
            return
        self.recovery_motion = msg
        self.on_recovery_path(msg.path)

    def on_sync(self, msg):
        self.sync_ok = msg.data
        self.sync_received_at = self.get_clock().now().nanoseconds * 1e-9

    def on_costmap(self, msg):
        self.costmap = msg
        if self.recovery_path is not None and not self.path_clear(self.recovery_path):
            self.recovery_path = None
            self.stop()
        # Reject an old rollout as soon as a newly placed obstacle blocks it.
        if self.local_path is not None and not self.path_clear(self.local_path):
            self.local_path = None
            self.stop()

    def path_clear(self, path):
        if self.costmap is None:
            return False
        info = self.costmap.info
        stamp = self.costmap.header.stamp.sec+self.costmap.header.stamp.nanosec*1e-9
        now = self.get_clock().now().nanoseconds*1e-9
        if (not valid_grid(self.costmap,self.planning_frame) or not -.05<=now-stamp<=2.
                or path.header.frame_id!=self.planning_frame or len(path.poses)<2
                or not all(valid_pose(p.pose) for p in path.poses)):
            return False
        grid = dict(width=info.width,height=info.height,resolution=info.resolution,
                    origin_x=info.origin.position.x,origin_y=info.origin.position.y,data=self.costmap.data)
        points = [(p.pose.position.x,p.pose.position.y) for p in path.poses]
        return all(segment_free(grid,a,b) for a,b in zip(points,points[1:]))

    def on_pose(self, msg):
        self.pose = msg
        self.pose_received_at = self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, msg):
        if self.goal is not None and math.hypot(
            msg.pose.position.x-self.goal.pose.position.x,
            msg.pose.position.y-self.goal.pose.position.y) < 1e-6:
            return
        self.goal = msg
        self.navigation_target = None
        self.goal_epoch = self.get_clock().now().nanoseconds*1e-9
        self.recovery_active = False
        self.recovery_path = None
        # A fast GPP can deliver the new path before this goal callback.
        if self.global_path is not None and self.global_path.poses:
            endpoint = self.global_path.poses[-1].pose.position
            if math.hypot(endpoint.x-msg.pose.position.x, endpoint.y-msg.pose.position.y) < 0.35:
                return
        self.global_path = None
        self.local_path = None
        self.stop()
        self.get_logger().info('Goal changed; waiting for a new GPP and LPP path')
        if self.pending_global is not None:
            self.on_global_path(self.pending_global)

    def on_navigation_target(self,msg):
        if (self.goal is None or msg.header.frame_id!=self.planning_frame or not valid_pose(msg.pose)
                or (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)<self.goal_epoch):
            return
        old=self.navigation_target
        self.navigation_target=msg
        if old is None or math.hypot(old.pose.position.x-msg.pose.position.x,old.pose.position.y-msg.pose.position.y)>1e-6:
            self.global_path=self.local_path=None
            if not self.recovery_active:self.stop()
        if self.pending_global is not None:self.on_global_path(self.pending_global)

    def on_global_path(self, msg):
        if msg.header.frame_id!=self.planning_frame or not all(valid_pose(p.pose) for p in msg.poses):
            self.global_path = self.local_path = None
            self.stop()
            return
        self.pending_global = msg
        if self.goal is None or not msg.poses:
            self.global_path = None
            self.local_path = None
            return
        endpoint = msg.poses[-1].pose.position
        associated=getattr(self,'navigation_target',None) if getattr(self,'goal_mode','direct')=='observed_corridor' else self.goal
        if associated is None:return
        if msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9 < self.goal_epoch:return
        target = associated.pose.position
        if math.hypot(endpoint.x - target.x, endpoint.y - target.y) > 0.35:
            return  # Path for the previous goal.
        self.global_path = msg
        # React in the GPP callback, before the next LPP/control timer. Do not
        # interrupt the supervisor's already-validated reverse/exit maneuver.
        if (not self.recovery_active and path_is_behind(self.pose, msg)
                and (self.local_path is None or path_moves_forward(self.local_path))):
            self.local_path = None
            self.stop()
            self.report('STOP: global route points behind; waiting for recovery')
        if self.local_path is not None and not self.path_clear(self.local_path):
            self.local_path = None

    def on_recovery_path(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id != self.planning_frame or stamp < self.goal_epoch or not -.05 <= now-stamp <= .5:
            return
        if self.recovery_active != self.recovery_motion.active:
            self.stop()
        self.recovery_active = self.recovery_motion.active
        self.recovery_received_at = now
        self.recovery_path = None
        self.local_path = None  # Never resume a cached normal rollout after release.
        if self.recovery_active and len(msg.poses) >= 2 and self.path_clear(msg):
            first, second = msg.poses[0].pose, msg.poses[1].pose
            heading = yaw_of(first.orientation)
            along = ((second.position.x-first.position.x)*math.cos(heading)
                     +(second.position.y-first.position.y)*math.sin(heading))
            if abs(along) > 1e-6:
                self.recovery_path = msg
        if self.recovery_active and self.recovery_path is None:
            self.stop()

    def on_local_path(self, msg):
        if self.recovery_active:
            return

        if path_moves_forward(msg) and path_is_behind(self.pose, self.global_path):
            # An old forward LPP message may arrive after the newer GPP message.
            self.local_path = None
            self.stop()
            return
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id != self.planning_frame or stamp < self.goal_epoch or not -.05 <= now-stamp <= .5:
            self.local_path = None
            return
        if self.global_path is not None and len(msg.poses) >= 2 and self.path_clear(msg):
            self.local_path = msg
            self.local_received_at = self.get_clock().now().nanoseconds * 1e-9
        else:
            self.local_path = None

    def report(self, message):
        self.status_pub.publish(String(data=message))
        status = ModuleStatus()
        status.header.frame_id = self.planning_frame
        status.header.stamp = self.get_clock().now().to_msg()
        status.valid_for.nanosec = 600_000_000
        status.state = ModuleStatus.READY if message in ('DRIVING','DRIVING: observation-limited','REVERSING TO RECOVER','ARRIVED') else ModuleStatus.STOPPED
        status.detail = message
        self.module_pub.publish(status)
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def control(self):
        command = self.command_message(stop=True)
        # Run on wall time; message timestamps and leases use ROS simulation time.
        now = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        if self.last_sim_time != now:
            self.last_clock_change = wall
            self.last_sim_time = now
        recovering = getattr(self, 'recovery_active', False)
        path = self.recovery_path if recovering else getattr(self, 'local_path', None)
        received = self.recovery_received_at if recovering else getattr(self, 'local_received_at', 0.)
        reason = None
        vehicle = self.vehicle_state
        if (vehicle is None or not vehicle.ready or vehicle.fault
                or not vehicle.speed_valid or not vehicle.steering_valid
                or vehicle.header.frame_id != self.base_frame
                or not all(math.isfinite(v) for v in (vehicle.speed,vehicle.steering_angle))
                or not -.05 <= now-(vehicle.header.stamp.sec+vehicle.header.stamp.nanosec*1e-9) <= .6
                or wall-self.vehicle_wall > .6):
            reason = 'STOP: control feedback unavailable'
        elif wall-self.last_clock_change > 0.5:
            reason = 'STOP: simulation clock paused'
        elif not self.sync_ok or now-self.sync_received_at > 1.0:
            reason = f'STOP: {self.health_label} unavailable'
        elif self.goal is None:
            reason = 'WAIT: choose a goal'
        elif self.pose is None or now-self.pose_received_at > 1.0:
            reason = 'STOP: odometry unavailable'
        elif not recovering and self.global_path is None:
            reason = 'STOP: waiting for GPP recovery'
        elif (path is None or now-received > 0.5
              or not -.05 <= now-(path.header.stamp.sec+path.header.stamp.nanosec*1e-9) <= .5
              or not self.path_clear(path)):
            reason = 'STOP: recovery hold/stale' if recovering else 'STOP: waiting for a collision-free LPP path'
        if reason:
            self.report(reason)
            self.cmd_pub.publish(command)
            return

        position = self.pose.pose.position
        target = self.goal.pose.position
        distance = math.hypot(target.x - position.x, target.y - position.y)
        if not recovering and distance < 0.35:
            self.report('ARRIVED')
            self.cmd_pub.publish(command)
            return

        first = path.poses[0].pose
        second = path.poses[1].pose
        if math.hypot(first.position.x - position.x, first.position.y - position.y) > 0.45:
            self.report('STOP: local path is behind the vehicle')
            self.cmd_pub.publish(command)
            return

        motion = self.recovery_motion if recovering else self.local_motion
        if motion is None or not all(math.isfinite(v) for v in
                                     (motion.target_speed,motion.target_steering_angle)):
            self.stop()
            self.report('STOP: invalid planned motion')
            return
        sampled_speed = motion.target_speed
        heading = yaw_of(first.orientation)
        along = (second.position.x-first.position.x)*math.cos(heading)+(second.position.y-first.position.y)*math.sin(heading)
        if along*sampled_speed <= 0:
            self.stop()
            self.report('STOP: command and checked trajectory disagree')
            return
        if (abs(sampled_speed) < 1e-6 or abs(motion.target_steering_angle) > self.max_steer+1e-6
                or abs(sampled_speed) > self.max_speed+1e-6):
            self.stop()
            self.report('STOP: planned command outside limits')
            return
        if not recovering and sampled_speed > 0 and path_is_behind(self.pose,self.global_path):
            self.local_path = None
            self.stop()
            self.report('STOP: global route points behind; waiting for recovery')
            return
        if sampled_speed*vehicle.speed < 0 and abs(vehicle.speed) > .03:
            self.stop()
            self.report('STOP: waiting for vehicle to stop before reversing direction')
            return
        speed = math.copysign(min(abs(sampled_speed),
            self.max_speed if recovering else max(.10,distance*.5)),sampled_speed)
        limited = False
        if getattr(self, 'observed_guard', False):
            info=self.costmap.info
            grid=dict(width=info.width,height=info.height,resolution=info.resolution,
                      origin_x=info.origin.position.x,origin_y=info.origin.position.y,data=self.costmap.data)
            stamp=self.costmap.header.stamp.sec+self.costmap.header.stamp.nanosec*1e-9
            known=observed_distance(grid,(position.x,position.y,yaw_of(self.pose.pose.orientation)),
                motion.target_steering_angle,math.copysign(1.,speed),self.guard_wheelbase,
                self.guard_offset,self.guard_half_length,self.guard_half_width)
            cap=speed_limit(known,self.guard_latency+max(0.,now-stamp),self.guard_deceleration,self.guard_margin)
            if cap < .02 or abs(vehicle.speed)>cap+1e-3:
                self.stop()
                self.report(f'STOP: observation boundary; known={known:.2f}m cap={cap:.3f}m/s')
                return
            limited=abs(speed)>cap
            speed=math.copysign(min(abs(speed),cap),speed)
        self.cmd_pub.publish(self.command_message(speed,motion.target_steering_angle))
        self.report('DRIVING: observation-limited' if limited and speed>0 else
                    'REVERSING TO RECOVER' if speed < 0 else 'DRIVING')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningCommandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    if rclpy.ok():
        node.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Fuse LiDAR collision evidence with stamped RGB-D terrain preferences."""
from collections import deque
from copy import deepcopy
import math
import time
from threading import Lock

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from nomad_interfaces.msg import ModuleStatus
from tf2_ros import Buffer, TransformListener, TransformException

from nomad_perception.geometry import project_scan, tilt
from nomad_perception.observed_grid import ObservedGrid
from nomad_perception.camera_terrain import CameraTerrain
from nomad_perception.terrain import fuse_costs


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9



class SensorInput(Node):
    def __init__(self):
        super().__init__('nomad_perception')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = p('planning_frame', 'odom')
        self.base_frame = p('base_frame', 'base_link')
        self.timeout = p('sensor_timeout', 0.6)
        self.wall_timeout = p('wall_timeout', 3.0)
        self.max_age = p('local_max_age', 2.0)
        self.max_tilt = p('max_tilt', 0.35)
        self.tilt_guard_enabled = p('tilt_guard_enabled', False)
        self.mapping_range = p('mapping_range', 8.0)
        self.navigation_lidar_enabled = p('navigation.lidar_enabled', False)
        resolution, size = p('resolution', 0.20), p('initial_size', 80.0)
        inflation, maximum = p('inflation', 0.70), p('max_size', 160.0)
        if (not all(math.isfinite(v) and v > 0 for v in
                    [self.timeout, self.wall_timeout, self.max_age, self.max_tilt, self.mapping_range,
                     resolution, size, inflation, maximum]) or maximum < size):
            raise ValueError('Mapping parameters must be positive, finite, and max_size >= initial_size')
        self.grid = ObservedGrid(resolution, size, inflation, maximum)
        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.pose = None
        self.pose_stamp = self.scan_stamp = -math.inf
        self.odom_wall = self.scan_wall = -math.inf
        self.pending = deque(maxlen=10)
        self.pending_goal = None
        self.requested_goal = None
        self.published_bounds = None
        self.input_lock = Lock()
        self.scan_valid = False
        self.fault = None
        self.last_now = None
        self.last_status = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pubs = [self.create_publisher(OccupancyGrid, topic, latched) for topic in
                         ['/nomad/observed_map', '/nomad/costmap', '/nomad/local_costmap']]
        self.health_pub = self.create_publisher(Bool, '/nomad/perception_ready', 10)
        self.status_pub = self.create_publisher(String, '/nomad/mapping_status', 10)
        self.module_pub = self.create_publisher(ModuleStatus, '/nomad/perception/status', 10)
        self.camera = CameraTerrain(self)
        if not self.navigation_lidar_enabled and not (
                self.camera.enabled and self.camera.mode == 'geometry'
                and self.camera.ground_enabled and self.camera.objects_only):
            raise ValueError('Navigation without LiDAR obstacles requires geometry, ground_filter_enabled and objects_only')
        self.navigation_pubs = [self.create_publisher(OccupancyGrid, topic, latched) for topic in
                                ['/nomad/navigation_costmap', '/nomad/navigation_local_costmap']]
        self.terrain_pub = self.create_publisher(OccupancyGrid, '/nomad/terrain_costmap', latched)
        latest_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/nomad/localization/odometry', self.on_odom, latest_sensor)
        self.create_subscription(LaserScan, '/scan', self.on_scan, latest_sensor)
        self.create_subscription(PoseStamped, '/nomad/goal', self.on_goal, 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.mapping_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.1, self.update_maps, clock=self.wall_clock,
                          callback_group=self.mapping_group)
        self.create_timer(0.1, self.tick, clock=self.wall_clock)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def fresh(self, stamp):
        return 0.0 < stamp and -0.05 <= self.now()-stamp <= self.timeout

    def on_odom(self, msg):
        stamp = seconds(msg.header.stamp)
        if self.fault:
            return
        p = msg.pose.pose.position
        try:
            valid = (msg.header.frame_id == self.frame and msg.child_frame_id == self.base_frame
                     and self.fresh(stamp) and all(math.isfinite(v) for v in (p.x, p.y, p.z))
                     and (tilt(msg.pose.pose.orientation) <= self.max_tilt
                          or not self.tilt_guard_enabled))
        except ValueError:
            valid = False
        if not valid:
            self.pose = None
            return
        if stamp <= self.pose_stamp:
            return
        if self.pose is not None:
            prev = self.pose.pose.position
            if math.hypot(p.x-prev.x, p.y-prev.y) > 1.0+2.0*(stamp-self.pose_stamp):
                self.fault = 'ODOM_JUMP: restart planning after localization/world reset'
                return
        self.pose = PoseStamped(header=deepcopy(msg.header), pose=deepcopy(msg.pose.pose))
        self.pose_stamp, self.odom_wall = stamp, time.monotonic()

    def on_scan(self, msg):
        stamp = seconds(msg.header.stamp)
        if not msg.header.frame_id or not self.fresh(stamp):
            self.scan_valid = False
            return
        if stamp > self.scan_stamp and not self.fault:
            with self.input_lock:
                self.pending.append(msg)

    def on_goal(self, msg):
        # Planning owns goal validation/acceptance. Perception only extends maps.
        p = msg.pose.position
        if msg.header.frame_id != self.frame or not all(math.isfinite(v) for v in (p.x, p.y)):
            return
        with self.input_lock:
            self.pending_goal = msg

    def accept_goal(self, msg):
        p = msg.pose.position
        if not self.grid.ensure_bounds(p.x, p.y):
            self.get_logger().warn('Goal exceeds costmap max_size')
            return
        self.publish_maps()

    def process_scan(self):
        # New scans may arrive before their dynamic TF. Retry without blocking ROS.
        with self.input_lock:
            pending = list(self.pending)
            self.pending.clear()
        for scan in reversed(pending):
            stamp = seconds(scan.header.stamp)
            if stamp <= self.scan_stamp or not self.fresh(stamp):
                continue
            try:
                tf = self.buffer.lookup_transform(self.frame, scan.header.frame_id,
                                                  Time.from_msg(scan.header.stamp))
            except TransformException:
                continue
            try:
                if tilt(tf.transform.rotation) > self.max_tilt and self.tilt_guard_enabled:
                    raise ValueError('Scan plane exceeds max_tilt')
                passable = lambda points: self.camera.passable_returns(points, stamp)
                origin, rays = project_scan(scan, tf.transform, self.mapping_range, passable,
                                            include_height=True)
                self.scan_valid = self.grid.integrate_rays(origin, rays, stamp)
            except ValueError:
                self.scan_valid = False
            self.scan_stamp, self.scan_wall = stamp, time.monotonic()
            break
        else:
            # Preserve TF-pending input unless a newer scan arrived meanwhile.
            with self.input_lock:
                if not self.pending:
                    self.pending.extend(pending)

    def update_maps(self):
        """Serial map writer; expensive depth work cannot stall input/health."""
        if self.fault:
            return
        pose = self.pose
        if pose is not None:
            p = pose.pose.position
            self.grid.ensure_bounds(p.x, p.y, self.camera.max_range+1.0)
        self.process_scan()
        self.camera.tick()
        self.publish_maps()
        with self.input_lock:
            goal = self.pending_goal
            self.pending_goal = None
        if goal is not None:
            self.accept_goal(goal)

    def tick(self):
        now, wall = self.now(), time.monotonic()
        if self.last_now is not None and now < self.last_now-0.05:
            self.fault = 'CLOCK_RESET: restart planning to discard the previous map'
        self.last_now = now
        if self.navigation_lidar_enabled:
            sensor_ready = (self.scan_valid and self.fresh(self.scan_stamp)
                            and wall-self.scan_wall <= self.wall_timeout)
            inputs = 'odom/scan/TF'
        else:
            sensor_ready = self.camera.navigation_active()
            inputs = 'odom/depth/TF (navigation LiDAR obstacles OFF)'
        ready = (not self.fault and self.pose is not None and sensor_ready
                 and self.fresh(self.pose_stamp) and wall-self.odom_wall <= self.wall_timeout)
        self.health_pub.publish(Bool(data=bool(ready)))
        status_msg = ModuleStatus()
        status_msg.header.frame_id = self.frame
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.state = ModuleStatus.FAULT if self.fault else (ModuleStatus.READY if ready else ModuleStatus.WAITING)
        status_msg.valid_for.nanosec = 600_000_000
        status_msg.detail = self.fault or inputs
        self.module_pub.publish(status_msg)
        status = self.fault or (('READY: ' if ready else 'WAIT: fresh ') + inputs)
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        maps = list(self.grid.maps(self.now(), self.max_age))
        retain = self.camera.mode == 'geometry' or self.camera.active()
        terrain = self.grid.terrain_map(self.now(), self.camera.ttl if retain else -1.0)
        if self.camera.enabled:
            maps[1:] = fuse_costs(maps[1], maps[2], terrain, self.grid.kernel, self.camera.neutral)
        navigation = (maps[1:] if self.navigation_lidar_enabled else
                      self.grid.navigation_maps_without_lidar_obstacles(
                          self.now(), self.camera.ttl, self.max_age))
        for pub, array in zip(self.map_pubs + [self.terrain_pub] + self.navigation_pubs,
                              maps + [terrain] + list(navigation)):
            msg = OccupancyGrid()
            msg.header.frame_id, msg.header.stamp = self.frame, stamp
            msg.info.resolution = self.grid.resolution
            msg.info.width, msg.info.height = self.grid.width, self.grid.height
            msg.info.origin.position.x, msg.info.origin.position.y = self.grid.origin_x, self.grid.origin_y
            msg.info.origin.orientation.w = 1.0
            # array('b') avoids the per-element Python setter validation cost.
            from array import array as signed_array
            msg.data = signed_array('b', array.ravel().tobytes())
            pub.publish(msg)
        self.published_bounds = (self.grid.origin_x, self.grid.origin_y,
                                 self.grid.origin_x+self.grid.width*self.grid.resolution,
                                 self.grid.origin_y+self.grid.height*self.grid.resolution)


def main(args=None):
    rclpy.init(args=args)
    node = SensorInput()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            node.health_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

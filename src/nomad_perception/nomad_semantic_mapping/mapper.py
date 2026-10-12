"""The same capture-time optical input boundary, with semantic-only computation."""
from array import array
from collections import Counter, deque
from dataclasses import asdict
from pathlib import Path
from threading import Lock
import json
import math
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException

from nomad_perception.semantic_input import decode_semantic_cloud
from nomad_perception.traversability_core import CoreConfig, TraversabilityCore
from nomad_perception.traversability_output import CostViews, transform_points
from .archive import write_snapshot
from .core import SemanticCore
from .cost_policy import SemanticCostPolicy


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SemanticMapper(Node):
    def __init__(self):
        super().__init__('nomad_semantic_mapper')
        p = lambda key, default: self.declare_parameter(key, default).value
        directory = p('session_directory', '')
        if not directory:
            raise ValueError('session_directory is required; start with ros2 run nomad_perception semantic_session')
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.world_file = p('world_file', '')
        self.map_id = p('map_id', 'unknown')
        semantic_defaults = {key: value for key, value in asdict(CoreConfig()).items()
                             if key in ('resolution_m', 'semantic_min_points', 'semantic_reference_points',
                                        'raw_window_s', 'per_cell_max_points',
                                        'global_max_points', 'near_tie_share_difference', 'sampling_seed')}
        self.config = CoreConfig(**{key: p(key, value) for key, value in semantic_defaults.items()},
                                 geometry_only=False, semantic_confidence_alpha=0.)
        self.cost_policy = SemanticCostPolicy(**{
            key: p(key, value) for key, value in asdict(SemanticCostPolicy()).items()})
        self.frame = p('frame_id', 'odom')
        if self.frame != 'odom':
            raise ValueError('The unchanged core expects points in odom')
        self.max_age = p('max_input_age_s', .6)
        self.tf_wait = p('tf_wait_s', .5)
        self.maximum_pending = p('max_pending', 2)
        self.checkpoint_period = p('checkpoint_period_s', 5.)
        self.minimum_z = p('optical_min_z', .4)
        self.maximum_z = p('optical_max_z', 8.)
        period = p('update_period_s', .1)
        self.core = SemanticCore(self.config, self.cost_policy)
        self.views = CostViews(self.config.resolution_m, p('initial_size_m', 80.),
                               p('max_size_m', 160.), inflation=0.)
        self.buffer = Buffer(cache_time=Duration(seconds=10.), node=self)
        self.listener = TransformListener(self.buffer, self)
        self.lock = Lock()
        self.pending = deque()
        self.trajectory = []
        self.latest_odom = None
        self.last_path_stamp = -math.inf
        self.last_input_stamp = -math.inf
        self.last_snapshot_stamp = 0.
        self.last_clock = None
        self.last_checkpoint = time.monotonic()
        self.last_status = 0.
        self.last_trajectory_publish = 0.
        self.estimates = {}
        self.fit_stamps = {}
        self.version = 0
        self.counts = Counter()
        self.last_processing_ms = 0.
        self.last_input_age_s = None
        self.last_update_wall = None
        self.fault = None
        self.outcome = 'running'
        self.layers = self.core.layers(self.views.grid, self.estimates)
        self.mapping_group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cost_pub = self.create_publisher(OccupancyGrid, '/semantic_mapping/costmap', latched)
        self.class_pub = self.create_publisher(Image, '/semantic_mapping/representative_class', latched)
        self.raw_pub = self.create_publisher(Image, '/semantic_mapping/raw_cost', latched)
        self.adjusted_pub = self.create_publisher(Image, '/semantic_mapping/adjusted_cost', latched)
        self.path_pub = self.create_publisher(RosPath, '/semantic_mapping/trajectory', latched)
        self.vehicle_pub = self.create_publisher(Marker, '/semantic_mapping/vehicle', latched)
        self.meta_pub = self.create_publisher(String, '/semantic_mapping/metadata', latched)
        self.create_subscription(PointCloud2, p('input_topic', '/nomad/semantic_points'),
                                 self.on_points, sensor)
        self.create_subscription(Odometry, p('odometry_topic', '/odom'), self.on_odom, sensor)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.update, clock=steady, callback_group=self.mapping_group)
        self.create_timer(.1, self.publish_motion, clock=steady)
        self.save(final=False)
        self.publish_map()
        self.get_logger().info('Semantic cost and representative class mapping ready. '
                               f'Session: {self.directory}')

    def on_points(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if msg.header.frame_id != 'oak_depth_optical_frame' or not math.isfinite(stamp) or stamp <= 0:
            self.counts['invalid_input'] += 1
            return
        with self.lock:
            self.counts['received_frames'] += 1
            if len(self.pending) >= self.maximum_pending:
                self.pending.popleft()
                self.counts['pending_replaced'] += 1
            self.pending.append((msg, time.monotonic()))

    def on_odom(self, msg):
        if self.fault:
            return
        if msg.header.frame_id != self.frame or msg.child_frame_id != 'base_link':
            return
        pose = msg.pose.pose
        xyz, q = pose.position, pose.orientation
        row = [stamp_seconds(msg.header.stamp), xyz.x, xyz.y, xyz.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(row).all() or np.linalg.norm(row[4:]) < 1e-12:
            return
        row[4:] = (np.asarray(row[4:]) / np.linalg.norm(row[4:])).tolist()
        q.x, q.y, q.z, q.w = row[4:]
        with self.lock:
            self.latest_odom = msg
            if row[0] >= self.last_path_stamp + .1:
                self.trajectory.append(row)
                self.last_path_stamp = row[0]

    def update(self):
        self.process_pending(publish=True)

    def process_pending(self, publish=True):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock - 1e-6:
            # Preserve the completed map rather than silently mixing two drives.
            self.fault = 'Simulation clock moved backwards. Start a new mapping session.'
            self.outcome = 'clock_reset'
        self.last_clock = now
        if self.fault:
            if publish:
                self.publish_status()
            if publish and time.monotonic() - self.last_checkpoint >= self.checkpoint_period:
                self.save(final=False)
            return
        with self.lock:
            pending = list(self.pending)
            self.pending.clear()
        pending.sort(key=lambda item: stamp_seconds(item[0].header.stamp))
        retries = []
        ingested = False
        began = time.perf_counter()
        for msg, arrived in pending:
            stamp = stamp_seconds(msg.header.stamp)
            age = now - stamp
            if stamp <= self.last_input_stamp:
                self.counts['out_of_order'] += 1
                continue
            if age < -.05 or age > self.max_age:
                self.counts['stale_input'] += 1
                continue
            try:
                transform = self.buffer.lookup_transform(
                    self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except TransformException:
                if time.monotonic() - arrived < self.tf_wait:
                    retries.append((msg, arrived))
                else:
                    self.counts['tf_timeout'] += 1
                continue
            try:
                xyz, ids = decode_semantic_cloud(msg)
                keep = (np.isfinite(xyz).all(axis=1) &
                        (xyz[:, 2] >= self.minimum_z) & (xyz[:, 2] <= self.maximum_z))
                xyz, ids = transform_points(xyz[keep], transform.transform), ids[keep]
                bounded = self.views.accept_points(xyz)
                self.counts['out_of_bounds_points'] += int(np.count_nonzero(~bounded))
                self.core.ingest(xyz[bounded], ids[bounded], stamp)
            except (ValueError, TypeError, OverflowError) as error:
                self.counts['invalid_input'] += 1
                self.get_logger().warning('Rejected cloud: ' + str(error))
                continue
            self.last_input_stamp = stamp
            self.counts['processed_frames'] += 1
            ingested = True
        if retries:
            with self.lock:
                combined = sorted([*retries, *self.pending],
                                  key=lambda item: stamp_seconds(item[0].header.stamp))
                excess = max(0, len(combined) - self.maximum_pending)
                self.counts['pending_replaced'] += excess
                self.pending = deque(combined[excess:])
        if math.isfinite(self.last_input_stamp):
            self.last_snapshot_stamp = max(now, self.last_input_stamp)
            self.estimates = self.core.snapshot(self.last_snapshot_stamp)
            if ingested or self.core.last_snapshot_changed_keys:
                self.layers = self.core.layers(self.views.grid, self.estimates)
                self.version += 1
            self.last_processing_ms = (time.perf_counter() - began) * 1000.
            self.last_input_age_s = max(0., self.get_clock().now().nanoseconds * 1e-9 - self.last_input_stamp)
            self.last_update_wall = time.time()
            if publish and (ingested or self.core.last_snapshot_changed_keys):
                self.publish_map()
        if publish and time.monotonic() - self.last_checkpoint >= self.checkpoint_period:
            self.save(final=False)
        if publish:
            self.publish_status()

    def metadata(self):
        grid = self.views.grid
        with self.lock:
            trajectory_count = len(self.trajectory)
            pending_count = len(self.pending)
        return {'schema': 'nomad-semantic-mapping-v1', 'frame_id': self.frame,
                'origin_x': float(grid.origin_x), 'origin_y': float(grid.origin_y),
                'resolution_m': self.config.resolution_m, 'width': grid.width, 'height': grid.height,
                'version': self.version, 'stamp_s': self.last_snapshot_stamp,
                'last_input_stamp_s': self.last_input_stamp if math.isfinite(self.last_input_stamp) else None,
                'map_id': self.map_id, 'world_file': self.world_file,
                'settings': asdict(self.config), 'cost_model': self.cost_policy.metadata(),
                'geometry_computed': False,
                'input_provider': 'Gazebo RGBD depth + original GOOSE64 GT labels',
                'projection_pixel_center_offset_px': .5,
                'observed_cells': len(self.estimates),
                'valid_class_cells': int(np.count_nonzero(self.layers['representative_class'] >= 0)),
                'trajectory_samples': trajectory_count, 'counters': dict(self.counts),
                'core_counters': dict(self.core.diagnostics), 'pending_frames': pending_count,
                'retained_points': self.core.active_point_count,
                'last_processing_ms': self.last_processing_ms, 'last_input_age_s': self.last_input_age_s,
                'last_update_wall_unix_s': self.last_update_wall, 'outcome': self.outcome, 'fault': self.fault,
                'cost_definition': 'S = sum(n_c * cost_c)/sum(n_c), excluding IDs 0,8,53,56; '
                                   'q_s=min(1,N_s/20); S_adj=min(99,S+beta*(1-effective_q_s)*(99-S)); '
                                   'one final costmap; original q_s remains internal; half-up rounding; '
                                   'true S=100 stays 100; observed without valid S=100; unobserved=-1',
                'class_definition': 'majority of all retained IDs; 10 percentage-point candidates; '
                                    'eligible nonground first, higher class cost, more points, smaller ID; '
                                    'valid at 20 total retained points; missing representative=-1',
                'design_source': 'https://app.notion.com/p/3f5dcfc04df080d688dfedcb9984ae60',
                'design_revision': '2026-10-12 user-authorized expiry fix and bounded neighbor confidence',
                'cumulative_policy': 'retain last usable semantic cost and representative class throughout '
                                     'the drive; expiry/cap eviction preserves original S, support and q_s; '
                                     'update only on new relevant retained observations; '
                                     'raw points have the shared 10s / 512 per cell / 1M global bounds',
                'geometry_only': False}

    def arrays(self):
        with self.lock:
            path = np.asarray(self.trajectory, dtype=np.float64).reshape(-1, 8).copy()
        keys = sorted(self.estimates)
        hist = np.zeros((len(keys),64), dtype=np.uint32)
        support_hist = np.zeros((len(keys),64), dtype=np.uint32)
        for i,key in enumerate(keys):
            for cls,count in self.estimates[key].class_histogram: hist[i,cls] = count
            for cls,count in self.estimates[key].semantic_support_histogram: support_hist[i,cls] = count
        return {**{k:v.copy() for k,v in self.layers.items()}, 'trajectory':path,
                'sparse_cell_xy':np.asarray(keys,dtype=np.int64).reshape(-1,2),
                'sparse_retained_class_counts':hist,
                'sparse_semantic_support_class_counts':support_hist}

    def save(self, final=False):
        arrays, metadata = self.arrays(), self.metadata()
        metadata['trajectory_samples'] = len(arrays['trajectory'])
        result = write_snapshot(self.directory, arrays, metadata, final=final)
        self.last_checkpoint = time.monotonic()
        return result

    def finalize(self, outcome='stopped'):
        # Executors are stopped; use buffered capture-time transforms and drain
        # fresh pending clouds without publishing on a closed ROS context.
        self.process_pending(publish=False)
        with self.lock:
            self.counts['shutdown_pending_discarded'] += len(self.pending)
            self.pending.clear()
        if self.outcome == 'running':
            self.outcome = outcome
        result = self.save(final=True)
        print(f'\nSaved mapping session: {self.directory}\nFinal map: {result}', flush=True)

    def publish_map(self):
        if not rclpy.ok(context=self.context):
            return
        stamp = self.get_clock().now().to_msg()
        grid = self.views.grid
        cost = OccupancyGrid()
        cost.header.frame_id, cost.header.stamp = self.frame, stamp
        cost.info.map_load_time = stamp
        cost.info.resolution = float(grid.resolution)
        cost.info.width, cost.info.height = grid.width, grid.height
        cost.info.origin.position.x = float(grid.origin_x)
        cost.info.origin.position.y = float(grid.origin_y)
        cost.info.origin.orientation.w = 1.
        cost.data = array('b', self.layers['cost'].ravel().tolist())
        self.cost_pub.publish(cost)
        for name, encoding, dtype, publisher in [
            ('representative_class', '16SC1', '<i2', self.class_pub),
            ('semantic_raw', '32FC1', '<f4', self.raw_pub),
            ('semantic_adjusted', '32FC1', '<f4', self.adjusted_pub)]:
            values = self.layers[name]
            image = Image()
            image.header = cost.header
            image.height, image.width = values.shape
            image.encoding, image.is_bigendian = encoding, False
            image.step = image.width * np.dtype(dtype).itemsize
            image.data = values.astype(dtype, copy=False).tobytes()
            publisher.publish(image)

    def publish_motion(self):
        if not rclpy.ok(context=self.context):
            return
        with self.lock:
            odom = self.latest_odom
            path = list(self.trajectory) if time.monotonic() - self.last_trajectory_publish >= .5 else None
        if odom is not None:
            marker = Marker()
            marker.header = odom.header
            marker.ns, marker.id, marker.type, marker.action = 'vehicle', 0, Marker.ARROW, Marker.ADD
            marker.pose = odom.pose.pose
            marker.scale.x, marker.scale.y, marker.scale.z = 1.1, .3, .3
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .9, 1., 1.
            self.vehicle_pub.publish(marker)
        if path is not None:
            message = RosPath()
            message.header.frame_id = self.frame
            message.header.stamp = self.get_clock().now().to_msg()
            for row in path:
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp = self.frame, message.header.stamp
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, row[1:4])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, row[4:])
                message.poses.append(pose)
            self.path_pub.publish(message)
            self.last_trajectory_publish = time.monotonic()

    def publish_status(self):
        if not rclpy.ok(context=self.context):
            return
        if time.monotonic() - self.last_status < 1.:
            return
        self.last_status = time.monotonic()
        message = String()
        message.data = json.dumps(self.metadata(), allow_nan=False)
        self.meta_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    outcome = 'stopped'
    try:
        node = SemanticMapper()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        outcome = 'mapper_error'
        raise
    finally:
        executor.shutdown(timeout_sec=30., wait_for_threads=True)
        if node is not None:
            try:
                node.finalize(outcome)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

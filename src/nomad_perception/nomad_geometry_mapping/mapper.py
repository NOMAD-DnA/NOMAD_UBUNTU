"""A ROS boundary around the unchanged, tested NOMAD geometry-only core.

No planning, actuation, scan clearing, TTL rendering, or semantic-cost fusion.
The limited raw point window and the persistent estimated map are distinct.
"""
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
from nomad_perception.quality_mapping_core import QualityMappingCore, PlaneQualityConfig
from nomad_perception.traversability_output import CostViews, transform_points
from nomad_perception.stage1_geometry_layers import sparse_state
from .archive import write_snapshot
from .cost_policy import GeometryCostPolicy


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ManualMapper(Node):
    def __init__(self):
        super().__init__('nomad_manual_mapper')
        p = lambda key, default: self.declare_parameter(key, default).value
        directory = p('session_directory', '')
        if not directory:
            raise ValueError('session_directory is required; start with ros2 run nomad_perception geometry_session')
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.world_file = p('world_file', '')
        self.map_id = p('map_id', 'unknown')
        config_values = {key: p(key, value) for key, value in asdict(CoreConfig()).items()}
        self.config = CoreConfig(**config_values)
        if not (self.config.geometry_only and self.config.radius_m == .3 and
                self.config.slope_input == 'raw_points' and self.config.plane_method == 'tls'):
            raise ValueError('This workspace requires geometry_only, 0.3 m, raw_points, TLS')
        self.cost_policy = GeometryCostPolicy(
            name=p('cost_model', 'angle_power_v1'), limit_deg=self.config.slope_limit_deg,
            power=p('cost_power', 2.), confidence_gain=p('cost_confidence_gain', .25),
            neighbor_confidence_enabled=p('neighbor_confidence_enabled', True),
            confidence_low_threshold=p('confidence_low_threshold', .3),
            confidence_neighbor_min=p('confidence_neighbor_min', .7))
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
        self.plane_quality_enabled = p('plane_quality_enabled', True)
        quality_defaults = asdict(PlaneQualityConfig())
        quality_values = {key: p('plane_' + key, list(value) if isinstance(value, tuple) else value)
                          for key, value in quality_defaults.items()}
        quality = PlaneQualityConfig(**quality_values)
        self.core = (QualityMappingCore(self.config, quality) if self.plane_quality_enabled
                     else TraversabilityCore(self.config))
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
        self.cost, self.angles = self.cost_policy.layers(self.estimates, self.views.grid)
        self.mapping_group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cost_pub = self.create_publisher(OccupancyGrid, '/manual_mapping/costmap', latched)
        self.angle_pub = self.create_publisher(Image, '/manual_mapping/slope_degrees', latched)
        self.path_pub = self.create_publisher(RosPath, '/manual_mapping/trajectory', latched)
        self.vehicle_pub = self.create_publisher(Marker, '/manual_mapping/vehicle', latched)
        self.meta_pub = self.create_publisher(String, '/manual_mapping/metadata', latched)
        self.create_subscription(PointCloud2, p('input_topic', '/nomad/semantic_points'),
                                 self.on_points, sensor)
        self.create_subscription(Odometry, p('odometry_topic', '/odom'), self.on_odom, sensor)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.update, clock=steady, callback_group=self.mapping_group)
        self.create_timer(.1, self.publish_motion, clock=steady)
        self.save(final=False)
        self.publish_map()
        self.get_logger().info('Manual mapping ready: raw points / radius 0.3 m / TLS. '
                               f'Session: {self.directory}')

    def on_points(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if not msg.header.frame_id or not math.isfinite(stamp):
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
            before = self.core.diagnostics['plane_fit_attempts']
            self.last_snapshot_stamp = max(now, self.last_input_stamp)
            self.estimates = self.core.snapshot(self.last_snapshot_stamp)
            if ingested or self.core.diagnostics['plane_fit_attempts'] != before:
                for key in self.core.last_snapshot_new_fit_keys:
                    self.fit_stamps[key] = now
                self.cost, self.angles = self.cost_policy.layers(self.estimates, self.views.grid)
                self.version += 1
                self.last_processing_ms = (time.perf_counter() - began) * 1000.
                self.last_input_age_s = max(0., self.get_clock().now().nanoseconds * 1e-9 - self.last_input_stamp)
                self.last_update_wall = time.time()
                if publish:
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
        return {'schema': 'nomad-manual-mapping-v1', 'frame_id': self.frame,
                'origin_x': float(grid.origin_x), 'origin_y': float(grid.origin_y),
                'resolution_m': self.config.resolution_m,
                'width': grid.width, 'height': grid.height, 'version': self.version,
                'stamp_s': self.last_snapshot_stamp,
                'last_input_stamp_s': self.last_input_stamp if math.isfinite(self.last_input_stamp) else None,
                'map_id': self.map_id, 'world_file': self.world_file,
                'candidate': 'quality_raw_tls_height_refit_v1' if self.plane_quality_enabled else 'raw_r0.3_tls',
                'candidate_settings': asdict(self.config),
                'plane_quality': self.core.quality_metadata() if self.plane_quality_enabled else {'enabled': False},
                'input_provider': 'Gazebo RGBD depth + GOOSE64 GT labels',
                'projection_pixel_center_offset_px': .5,
                'observed_cells': len(self.estimates),
                'valid_slope_cells': int(np.count_nonzero(np.isfinite(self.angles))),
                'trajectory_samples': trajectory_count,
                'counters': dict(self.counts), 'core_counters': dict(self.core.diagnostics),
                'pending_frames': pending_count, 'last_processing_ms': self.last_processing_ms,
                'last_input_age_s': self.last_input_age_s,
                'last_update_wall_unix_s': self.last_update_wall,
                'outcome': self.outcome, 'fault': self.fault,
                'cost_model': self.cost_policy.metadata(),
                'cost_definition': f'pre-inflation {self.cost_policy.name} geometry cost; '
                                   '-1 unobserved, 0..99 ordinary, 100 blocked or unfit',
                'cumulative_policy': 'retain last valid estimates for the drive; raw points '
                                     'remain bounded by the tested 10-second/cell/global limits',
                'geometry_only': True}

    def arrays(self):
        grid = self.views.grid
        heights = np.full(self.cost.shape, np.nan, dtype=np.float32)
        for (ix, iy), estimate in self.estimates.items():
            x, y = grid.cell((ix + .5) * grid.resolution, (iy + .5) * grid.resolution)
            if 0 <= x < grid.width and 0 <= y < grid.height and estimate.height is not None:
                heights[y, x] = estimate.height
        with self.lock:
            path = np.asarray(self.trajectory, dtype=np.float64).reshape(-1, 8).copy()
        sparse = sparse_state(self.estimates, self.core,
                              self.last_snapshot_stamp, self.fit_stamps)
        sparse['legacy_output_cost'] = sparse['output_cost'].copy()
        if self.plane_quality_enabled:
            sparse.update(self.core.quality_arrays(sparse['cells']))
        effective, corrected = self.cost_policy.sparse_confidence(sparse)
        sparse['effective_q_g'] = effective
        sparse['confidence_neighbor_corrected'] = corrected
        sparse['output_cost'] = self.cost_policy.apply(
            sparse['output_cost'], sparse['slope_deg'], effective)
        return {'cost': self.cost.copy(), 'slope_deg': self.angles.copy(), 'height_m': heights,
                'trajectory': path,
                **(self.core.diagnostic_arrays() if self.plane_quality_enabled else {}),
                **{'sparse_' + key: np.asarray(value)
                                      for key, value in sparse.items()}}

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
        cost.data = array('b', self.cost.ravel().tolist())
        image = Image()
        image.header = cost.header
        image.height, image.width = self.angles.shape
        image.encoding, image.is_bigendian = '32FC1', False
        image.step = image.width * 4
        image.data = self.angles.astype('<f4', copy=False).tobytes()
        self.cost_pub.publish(cost)
        self.angle_pub.publish(image)

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
        node = ManualMapper()
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

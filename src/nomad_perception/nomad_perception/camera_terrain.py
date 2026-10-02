"""Bounded RGB-D synchronisation and timestamped projection for SensorInput."""
from collections import deque
import math
import time

import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException

from nomad_perception.geometry import rotation
from nomad_perception.terrain import image_array, classify, project_ground
from nomad_perception.geometric_terrain import depth_points, surface_costs
from nomad_perception.grass_perception import GrassEvidence, low_grass
from nomad_perception.ground_perception import GroundEvidence


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class CameraTerrain:
    def __init__(self, node):
        self.node = node
        p = lambda key, default: node.declare_parameter('terrain.' + key, default).value
        self.enabled = p('enabled', True)
        self.mode = p('mode', 'geometry')
        self.grass_enabled = p('grass_enabled', True)
        self.grass_height = p('grass_max_height', 0.60)
        self.grass = GrassEvidence()
        if not math.isfinite(self.grass_height) or not 0 < self.grass_height <= 0.60:
            raise ValueError('grass_max_height must be in (0, 0.60] metres')
        self.patch_radius = p('patch_radius', 0.35)
        self.slope_limit = p('slope_limit', 0.35)
        self.slope_enabled = p('slope_enabled', False)
        self.objects_only = p('objects_only', True)
        self.obstacle_height = p('obstacle_height', .12)
        if not math.isfinite(self.obstacle_height) or self.obstacle_height <= 0:
            raise ValueError('obstacle_height must be positive and finite')
        self.roughness_limit = p('roughness_limit', 0.04)
        self.step_limit = p('step_limit', 0.18)
        self.geometry_min_samples = p('geometry_min_samples', 6)
        self.ground_enabled = p('ground_filter_enabled', True)
        self.ground_memory = p('ground_memory_seconds', 10.)
        if not math.isfinite(self.ground_memory) or self.ground_memory < 0:
            raise ValueError('ground_memory_seconds must be finite and nonnegative')
        self.ground = GroundEvidence(slope_limit=self.slope_limit,
                                     slope_enabled=self.slope_enabled and not self.objects_only,
                                     cell_local=self.objects_only,
                                     memory_seconds=self.ground_memory)
        self.ground_wall = -math.inf
        self.ground_valid = False
        if (self.mode not in ('geometry', 'colour') or self.geometry_min_samples < 3
                or not all(math.isfinite(v) and v > 0 for v in
                           [self.patch_radius, self.slope_limit, self.roughness_limit, self.step_limit])
                or self.slope_limit >= math.pi/2):
            raise ValueError('Invalid geometric terrain parameters')
        self.dirt_cost, self.field_cost = p('dirt_cost', 0), p('field_cost', 65)
        self.neutral = p('neutral_cost', 35)
        self.ttl = p('memory_seconds', 30.0)
        self.max_range = p('max_range', 6.0)
        self.stride = p('pixel_stride', 2)
        self.max_slope = p('max_surface_slope', 0.52)
        self.sync_tolerance = p('sync_tolerance', 0.04)
        self.min_samples = p('min_samples_per_cell', 3)
        self.dirt_hue = p('dirt_hue', [5, 20])
        self.field_hue = p('field_hue', [22, 95])
        self.min_saturation = p('min_saturation', 30)
        self.min_value = p('min_value', 30)
        if (not 0 <= self.dirt_cost < self.neutral < self.field_cost < 80
                or self.stride < 1 or self.min_samples < 1
                or not all(math.isfinite(v) and v > 0 for v in
                           [self.ttl, self.max_range, self.max_slope, self.sync_tolerance])
                or self.max_slope >= math.pi/2 or self.max_range <= 0.4
                or not 0 <= self.min_saturation <= 255 or not 0 <= self.min_value <= 255
                or any(len(h) != 2 or not 0 <= h[0] <= h[1] <= 179
                       for h in [self.dirt_hue, self.field_hue])):
            raise ValueError('Invalid terrain parameters (costs must be below obstacle threshold 80)')
        self.rgb, self.depth = deque(maxlen=8), deque(maxlen=8)
        self.depth_received_wall = -math.inf
        self.info = {}
        self.last_stamp = self.last_wall = -math.inf
        self.last_process_wall = -math.inf
        self.status = 'DISABLED' if not self.enabled else 'WAIT: terrain depth/calibration/TF'
        self.status_pub = node.create_publisher(String, '/nomad/terrain_status', 10)
        self.image_pub = node.create_publisher(Image, '/nomad/terrain_labels', 1)
        if self.enabled:
            latest_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for side, queue in [('rgb', self.rgb), ('depth', self.depth)]:
                if self.mode == 'geometry' and side == 'rgb' and not self.grass_enabled:
                    continue
                callback = self.receive_depth if side == 'depth' else lambda msg, q=queue: q.append(msg)
                node.create_subscription(Image, f'/oak/{side}/image_raw', callback,
                                         latest_sensor)
                node.create_subscription(CameraInfo, f'/oak/{side}/camera_info',
                                         lambda msg, s=side: self.info.__setitem__(s, msg),
                                         latest_sensor)

    def active(self):
        return (self.enabled and self.node.fresh(self.last_stamp)
                and time.monotonic()-self.last_wall <= self.node.wall_timeout)

    def receive_depth(self, msg):
        self.depth.append(msg)
        self.depth_received_wall = time.monotonic()

    def navigation_active(self):
        """Fresh input AND a valid map within the existing local map lifetime.

        Processing a dense cloud takes longer than receiving one image. Do not
        mistake processing cadence for sensor loss; do not refresh map evidence
        merely because more images arrive, either.
        """
        if not self.enabled or not self.depth or not self.ground_valid:
            return False
        wall = time.monotonic()
        return (self.node.fresh(stamp(self.depth[-1]))
                and wall-self.depth_received_wall <= self.node.wall_timeout
                and 0 <= self.node.now()-self.last_stamp <= self.node.max_age
                and wall-self.last_wall <= self.node.wall_timeout)

    def tick(self):
        wall = time.monotonic()
        if self.enabled and wall-self.last_process_wall >= 0.2:
            self.last_process_wall = wall
            self.process()
        status = self.status
        if self.enabled and not self.active():
            status = ('STALE: retained geometry until TTL; unseen/expired neutral; '
                      if self.mode == 'geometry' else 'FALLBACK neutral cost; ') + status
        self.status_pub.publish(String(data=status))

    def process(self):
        if getattr(self, 'mode', 'colour') == 'geometry':
            self.process_geometry()
            return
        n = self.node
        if n.fault or not self.rgb or not self.depth or len(self.info) != 2:
            return
        # Use the newest fresh depth frame that has a matching RGB timestamp.
        pair = None
        for depth in reversed(list(self.depth)):
            if stamp(depth) <= self.last_stamp or not n.fresh(stamp(depth)):
                continue
            rgb = min(list(self.rgb), key=lambda m: abs(stamp(m)-stamp(depth)))
            if n.fresh(stamp(rgb)) and abs(stamp(rgb)-stamp(depth)) <= self.sync_tolerance:
                pair = rgb, depth
                break
        if pair is None:
            self.status = 'WAIT: fresh synchronized RGB-D'
            return
        rgb, depth = pair
        try:
            ri, di = self.info['rgb'], self.info['depth']
            if (rgb.width != depth.width or rgb.height != depth.height
                    or any((i.width, i.height) != (depth.width, depth.height) for i in [ri, di])
                    or ri.header.frame_id != rgb.header.frame_id
                    or di.header.frame_id != depth.header.frame_id
                    or not np.allclose(ri.k, di.k, atol=1e-5)
                    or any(abs(v) > 1e-8 for i in [ri, di] for v in i.d)):
                raise ValueError('Requires aligned undistorted RGB-D with matching CameraInfo')
            # This simulator supplies coincident RGB/depth optical frames. Reject
            # other extrinsics instead of silently treating unaligned depth as RGB.
            extrinsic = n.buffer.lookup_transform(rgb.header.frame_id, depth.header.frame_id,
                                                  Time.from_msg(depth.header.stamp)).transform
            offset = np.array([extrinsic.translation.x, extrinsic.translation.y, extrinsic.translation.z])
            if not np.allclose(offset, 0, atol=1e-5) or not np.allclose(rotation(extrinsic.rotation), np.eye(3), atol=1e-5):
                raise ValueError('RGB/depth optical frames are not coincident')
            tf = n.buffer.lookup_transform(n.frame, depth.header.frame_id,
                                           Time.from_msg(depth.header.stamp)).transform
            r = rotation(tf.rotation)
            t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            if not np.all(np.isfinite(t)):
                raise ValueError('Invalid camera TF translation')
            bgr, metres = image_array(rgb), image_array(depth)
            if bgr.ndim != 3 or metres.ndim != 2:
                raise ValueError('RGB/depth encodings do not match their roles')
            labels = classify(bgr, self.dirt_hue, self.field_hue, self.min_saturation, self.min_value)
            with np.errstate(invalid='ignore', divide='ignore'):
                points, classes = project_ground(metres, labels, di.k, r, t, self.stride,
                                                self.max_range, self.max_slope)
            count = n.grid.integrate_terrain(points, classes, stamp(depth),
                                             self.dirt_cost, self.field_cost, self.neutral,
                                             self.min_samples)
            self.last_stamp, self.last_wall = stamp(depth), time.monotonic()
            self.status = f'RGB-D: cells={count}; dirt={self.dirt_cost}, field={self.field_cost}; colour baseline'
            if self.image_pub.get_subscription_count():
                palette = np.array([[100, 100, 100], [45, 130, 200], [60, 200, 60]], dtype=np.uint8)
                out = Image(header=rgb.header, height=rgb.height, width=rgb.width,
                            encoding='bgr8', is_bigendian=0, step=rgb.width*3,
                            data=palette[labels].tobytes())
                self.image_pub.publish(out)
        except TransformException:
            self.status = 'WAIT: timestamped camera TF'
        except (ValueError, TypeError) as error:
            self.status = 'REJECT: ' + str(error)

    def process_geometry(self):
        n = self.node
        if n.fault or not self.depth or 'depth' not in self.info:
            return
        depth = self.depth[-1]
        if stamp(depth) <= self.last_stamp or not n.fresh(stamp(depth)):
            self.status = 'WAIT: fresh depth'
            return
        try:
            info = self.info['depth']
            if ((info.width, info.height) != (depth.width, depth.height)
                    or info.header.frame_id != depth.header.frame_id
                    or any(not math.isfinite(v) or abs(v) > 1e-8 for v in info.d)):
                raise ValueError('Requires undistorted depth with matching CameraInfo')
            tf = n.buffer.lookup_transform(n.frame, depth.header.frame_id,
                                           Time.from_msg(depth.header.stamp)).transform
            r = rotation(tf.rotation)
            t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
            if not np.all(np.isfinite(t)):
                raise ValueError('Invalid camera TF translation')
            metres = image_array(depth)
            points = depth_points(metres, info.k, r, t,
                                  self.stride, self.max_range)
            grass_points = np.empty((0, 3))
            grass_status = 'disabled'
            if getattr(self, 'grass_enabled', False):
                flags = np.zeros(len(points), dtype=bool)
                try:
                    colours = self.aligned_colours(depth, info, metres)
                    flags = low_grass(points, colours, max_height=self.grass_height)
                    grass_status = 'RGB-D low-grass baseline'
                except (ValueError, TransformException) as error:
                    grass_status = 'unavailable: ' + str(error)
                confirmed = self.grass.update(points, flags, stamp(depth))
                grass_points, points = points[confirmed], points[~confirmed]
            centres, costs, metrics = surface_costs(
                points, n.grid.resolution, self.patch_radius, self.geometry_min_samples,
                self.slope_limit, self.roughness_limit, self.step_limit,
                slope_enabled=self.slope_enabled)
            ground_cells = 0
            cleared_ground_cells = 0
            if getattr(self, 'ground_enabled', False):
                try:
                    base = n.buffer.lookup_transform(n.frame, n.base_frame,
                        Time.from_msg(depth.header.stamp)).transform
                    base_t = np.array([base.translation.x, base.translation.y, base.translation.z])
                    if not np.all(np.isfinite(base_t)):
                        raise ValueError('Invalid base TF translation')
                    ground_cells = self.ground.update(points, rotation(base.rotation), base_t, stamp(depth))
                    self.ground_wall = time.monotonic()
                    cleared_ground_cells = n.grid.reclassify_ground(self.ground.matches, stamp(depth))
                except TransformException:
                    self.ground.clear()
            if self.objects_only:
                centres, costs = self.ground.object_costs(points, self.obstacle_height)
            count = n.grid.integrate_surface_costs(centres, costs, stamp(depth))
            # Grass does not inherit canopy roughness. A solid geometric hazard
            # in the same cell still wins; supporting ground is evaluated above.
            if len(grass_points):
                grass_cells = np.unique(np.floor(grass_points[:, :2]/n.grid.resolution).astype(int), axis=0)
                observed_cells = {tuple(c) for c in np.floor(centres/n.grid.resolution).astype(int)}
                grass_cells = np.array([c for c in grass_cells if tuple(c) not in observed_cells]).reshape(-1, 2)
                n.grid.integrate_surface_costs((grass_cells+.5)*n.grid.resolution,
                    np.full(len(grass_cells), self.neutral, dtype=np.int8), stamp(depth))
            self.last_stamp, self.last_wall = stamp(depth), time.monotonic()
            maxima = np.max(metrics, axis=0) if len(metrics) else np.zeros(3)
            self.ground_valid = (getattr(self, 'ground_enabled', False)
                                 and bool(self.ground.cells) and self.ground.stamp == self.last_stamp)
            self.status = (f'GEOMETRY: cells={count}; blocked={np.count_nonzero(costs >= 80)}; '
                           f'grass_points={len(grass_points)} ({grass_status}); '
                           f'connected_ground_cells={ground_cells}; '
                           f'ground_memory_cells={len(self.ground.history) if hasattr(self, "ground") else 0}; '
                           f'cleared_ground_cells={cleared_ground_cells}; '
                           f'objects_only={self.objects_only}; '
                           f'slope_enabled={self.slope_enabled}; max slope={np.degrees(maxima[0]):.1f}deg, '
                           f'roughness={maxima[1]:.3f}m, residual_span={maxima[2]:.3f}m')
        except TransformException:
            self.ground_valid = False
            if hasattr(self, 'ground'):
                self.ground.clear()
            self.status = 'WAIT: timestamped depth TF'
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            self.ground_valid = False
            if hasattr(self, 'ground'):
                self.ground.clear()
            self.status = 'REJECT: ' + str(error)

    def passable_returns(self, points, at):
        result = np.zeros(len(points), dtype=bool)
        if not self.enabled or self.mode != 'geometry':
            return result
        if self.grass_enabled:
            result |= self.grass.matches(points, at)
        if self.ground_enabled and time.monotonic()-self.ground_wall <= self.node.wall_timeout:
            result |= self.ground.matches(points, at)
        return result

    def aligned_colours(self, depth, info, metres):
        """Use fresh, coincident RGB only; never colour an occluded depth point."""
        if not self.rgb or 'rgb' not in self.info:
            raise ValueError('RGB/calibration missing')
        rgb = min(list(self.rgb), key=lambda m: abs(stamp(m)-stamp(depth)))
        ri = self.info['rgb']
        if (not self.node.fresh(stamp(rgb)) or abs(stamp(rgb)-stamp(depth)) > self.sync_tolerance
                or (rgb.width, rgb.height) != (depth.width, depth.height)
                or (ri.width, ri.height) != (depth.width, depth.height)
                or ri.header.frame_id != rgb.header.frame_id
                or not np.allclose(ri.k, info.k, atol=1e-5)
                or any(not math.isfinite(v) or abs(v) > 1e-8 for v in ri.d)):
            raise ValueError('RGB/depth alignment or timestamp mismatch')
        tf = self.node.buffer.lookup_transform(rgb.header.frame_id, depth.header.frame_id,
                                               Time.from_msg(depth.header.stamp)).transform
        if (not np.allclose([tf.translation.x, tf.translation.y, tf.translation.z], 0, atol=1e-5)
                or not np.allclose(rotation(tf.rotation), np.eye(3), atol=1e-5)):
            raise ValueError('RGB/depth optical frames are not coincident')
        bgr = image_array(rgb)
        if bgr.ndim != 3:
            raise ValueError('RGB encoding required')
        sampled = metres[::self.stride, ::self.stride]
        valid = np.isfinite(sampled) & (sampled >= .4) & (sampled <= self.max_range)
        return bgr[::self.stride, ::self.stride][valid]

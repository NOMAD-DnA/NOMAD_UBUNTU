"""Real RGB-D terrain provider for unchanged NOMAD Planning contracts."""

from array import array
import math, time, json
from types import SimpleNamespace
from collections import deque
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nomad_interfaces.msg import ModuleStatus, PlanningCostLayers
from tf2_ros import TransformException
from .live_terrain import LiveTerrain, project_capture, NAMES, PALETTE
from .terrain_worker import TerrainWorker, persist_result
from std_msgs.msg import String
from .semantic_input import stamp_ns

# Compatibility exports for existing callers; policies live outside ROS IO.
from .navigation_costmap import (
    NavigationLayers,
    navigation_arrays,
    TemporalBlockEvidence,
)


class NavigationTerrain(LiveTerrain):
    def __init__(self):
        super().__init__()
        q = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.navpub = [
            self.create_publisher(OccupancyGrid, t, q)
            for t in ["/nomad/navigation_costmap", "/nomad/navigation_local_costmap"]
        ]
        self.feature_pub = self.create_publisher(PlanningCostLayers, "/nomad/planning_cost_layers", q)
        self.latest_features = None
        self.camera_support = None
        self.support_pub = self.create_publisher(OccupancyGrid, "/nomad/camera_observation_support", q)
        self.health = self.create_publisher(
            ModuleStatus, "/nomad/perception/status", 10
        )
        self.odom = None
        self.odom_wall = 0.0
        self.map_wall = 0.0
        self.map_stamp = -np.inf
        self.last_result_stamp_ns = -1
        self.fault = ""
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/nomad/localization/odometry", self.position, 10
        )
        self.latest_costmaps = None
        self.scan_queue = deque(maxlen=6)
        self.scan_stats = {"received": 0, "integrated": 0, "tf_wait": 0, "expired": 0}
        self.bootstrap = None
        self.bootstrap_active = True
        self.ready_scans = deque(maxlen=6)
        self.last_arrays = None
        self.last_report = None
        self.output_grid = None
        self.last_clock_ns = self.get_clock().now().nanoseconds
        self.declare_parameter("worker_timeout_wall_sec", 30.0)
        self.worker = TerrainWorker({
            'layers':dict(resolution=self.get_parameter('resolution').value,
                          size=self.get_parameter('size').value, max_size=self.get_parameter('max_size').value,
                          ground_ids=self.get_parameter('ground_ids').value,
                          slope_limit=self.get_parameter('slope_limit_deg').value,
                          bootstrap_override=self.get_parameter('bootstrap_override').value,
                          obstacle_cost_radius=self.get_parameter('obstacle_cost_radius').value,
                          passable_semantic_ids=self.get_parameter('passable_semantic_ids').value),
            'slope_limit':self.get_parameter('slope_limit_deg').value,
            'local_age':self.get_parameter('local_max_age').value})
        self.result_timer = self.create_timer(.05, self.collect_result,
                                             clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.health_timer = self.create_timer(
            0.1, self.health_tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def create_layers(self):
        """Configure navigation policies without creating a second terrain grid."""
        for k, v in [
            ("slope_limit_deg", 10.0),
            ("local_max_age", 30.0),
            ("bootstrap_radius", 0.0),
            ("bootstrap_override", False),
            ("obstacle_cost_radius", 1.2),
            ("passable_semantic_ids", [3]),
        ]:
            self.declare_parameter(k, v)
        return NavigationLayers(
            self.get_parameter("resolution").value,
            self.get_parameter("size").value,
            self.get_parameter("max_size").value,
            self.get_parameter("ground_ids").value,
            slope_limit=self.get_parameter("slope_limit_deg").value,
            bootstrap_override=self.get_parameter("bootstrap_override").value,
            obstacle_cost_radius=self.get_parameter("obstacle_cost_radius").value,
            passable_semantic_ids=self.get_parameter("passable_semantic_ids").value,
        )

    def position(self, m):
        if m.header.frame_id != "odom" or m.child_frame_id != "base_link":
            return
        self.odom = m
        self.odom_wall = time.monotonic()
        stamp = stamp_ns(m)*1e-9
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        p = m.pose.pose.position
        radius = self.get_parameter("bootstrap_radius").value
        if not math.isfinite(radius) or radius < 0:
            raise ValueError("bootstrap_radius must be finite and nonnegative")
        if radius == 0:
            self.bootstrap_active = False
            return
        if self.bootstrap is None:
            self.bootstrap = (p.x, p.y, stamp, self.get_parameter('bootstrap_radius').value, yaw)
        if math.hypot(p.x-self.bootstrap[0], p.y-self.bootstrap[1]) > self.bootstrap[3]:
            self.bootstrap_active = False

    def scan(self, m):
        self.scan_stats["received"] += 1
        self.scan_queue.append(m)

    def integrate_pending_scans(self):
        if self.odom is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for m in list(self.scan_queue):
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if now - stamp > 0.6:
                self.scan_queue.remove(m)
                self.scan_stats["expired"] += 1
                continue
            try:
                t = self.buffer.lookup_transform(
                    "odom", m.header.frame_id, Time.from_msg(m.header.stamp)
                ).transform
            except TransformException:
                self.scan_stats["tf_wait"] += 1
                continue
            q = t.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            args = (
                (t.translation.x, t.translation.y, yaw),
                m.ranges,
                m.angle_min,
                m.angle_increment,
                m.range_min,
                m.range_max,
                stamp,
            )
            if hasattr(self, 'ready_scans'):
                if len(self.ready_scans) == self.ready_scans.maxlen:
                    self.trace('scan_overwritten', reason='worker_scan_queue_full')
                self.ready_scans.append(args)
            else:
                self.layers.views.grid.integrate(*args)
            self.scan_queue.remove(m)
            if not hasattr(self, "ready_scans"):
                self.scan_stats["integrated"] += 1

    def process(self):
        if self.fault:
            return
        self.integrate_pending_scans()
        if self.worker.busy or self.pending is None:
            return
        pair, arrived = self.pending
        depth = pair['depth']
        stamp = stamp_ns(depth)
        if time.monotonic()-arrived > 3:
            self.trace('pending_expired', stamp_ns=stamp, wait_wall_s=time.monotonic()-arrived)
            self.pending = None
            return
        try:
            base = self.buffer.lookup_transform('odom', 'base_link', Time.from_msg(depth.header.stamp))
            camera = self.buffer.lookup_transform('odom', depth.header.frame_id, Time.from_msg(depth.header.stamp))
        except TransformException as error:
            self.trace('tf_wait', stamp_ns=stamp, reason=str(error), wait_wall_s=time.monotonic()-arrived)
            return
        self.trace('tf_ready', stamp_ns=stamp, wait_wall_s=time.monotonic()-arrived)
        self.pending = None
        try:
            began = time.perf_counter()
            xyz, ids = project_capture(pair, camera.transform,
                                       self.get_parameter('stride').value,
                                       self.get_parameter('min_confidence').value)
            self.trace('projection_complete', stamp_ns=stamp, elapsed_ms=(time.perf_counter()-began)*1000)
            now_s = self.get_clock().now().nanoseconds*1e-9
            scans = [args for args in self.ready_scans if now_s-args[-1] <= .6]
            self.scan_stats['expired'] += len(self.ready_scans)-len(scans)
            save_directory = self.directory if (self.get_parameter('record_results').value
                              and time.monotonic()-self.last_save > 5) else ''
            metadata = dict(sync_dropped=self.sync.dropped, class_names=NAMES, palette=PALETTE,
                            errors=dict(self.errors), ground_ids=list(self.layers.ground_ids),
                            slope_method='raw_ground_points_tls', height_refit=False)
            self.worker.submit(dict(stamp_ns=stamp, xyz=xyz, ids=ids,
                                    body_z=base.transform.translation.z,
                                    bootstrap=self.bootstrap, bootstrap_active=self.bootstrap_active,
                                    scans=scans, save_directory=save_directory, report_metadata=metadata,
                                    scan_stats=dict(self.scan_stats)))
            if save_directory:
                self.last_save = time.monotonic()
            self.ready_scans.clear()
            self.trace('compute_submitted', stamp_ns=stamp)
        except (ValueError, IndexError) as error:
            self.errors[str(error)] = self.errors.get(str(error), 0)+1
            self.trace('compute_error', stamp_ns=stamp, reason=str(error))

    def collect_result(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock_ns:
            self.fault = 'CLOCK_RESET: restart map'
        self.last_clock_ns = now
        if self.fault:
            return
        result = self.worker.poll()
        if result is None:
            if self.worker.busy and time.monotonic()-self.worker.submitted_at > self.get_parameter('worker_timeout_wall_sec').value:
                self.fault = 'TERRAIN_WORKER_TIMEOUT: restart map'
                self.trace('worker_fault', reason=self.fault)
            return
        if 'error' in result:
            self.fault = 'TERRAIN_WORKER_FAILED: restart map'
            self.get_logger().error(result['error'])
            self.trace('worker_fault', reason=result['error'])
            return
        stamp = result['stamp_ns']
        if stamp <= self.last_result_stamp_ns or stamp > now:
            self.fault = 'INVALID_WORKER_CAPTURE: restart map'
            return
        self.last_result_stamp_ns = stamp
        for event, ms in result['timings'].items():
            self.trace(event, stamp_ns=stamp, elapsed_ms=ms, measured_in='worker')
        self.trace('compute_complete', stamp_ns=stamp, elapsed_ms=result['processing_ms'],
                   dispatch_to_result_ms=(time.monotonic()-self.worker.submitted_at)*1000)
        self.output_grid = SimpleNamespace(**result['grid'])
        self.last_arrays = result['arrays']
        self.latest_costmaps = result['maps']
        self.latest_features = result['features']
        self.camera_support = result['camera_support']
        self.count = result['frames']
        self.scan_stats['integrated'] += result['scans_integrated']
        self.publish(Time(nanoseconds=stamp).to_msg(), self.last_arrays)
        arrays = self.last_arrays
        report = dict(frames=self.count, capture_stamp=stamp*1e-9, processing_ms=result['processing_ms'],
                      height_cells=int(np.isfinite(arrays[0]).sum()), slope_cells=int(np.isfinite(arrays[1]).sum()),
                      class_cells=int((arrays[2]>=0).sum()), sync_dropped=self.sync.dropped,
                      class_names=NAMES, palette=PALETTE, errors=self.errors,
                      ground_ids=list(self.layers.ground_ids), slope_method='raw_ground_points_tls',
                      height_refit=False, plane_sources=result['plane_sources'])
        self.last_report = report
        self.status_pub.publish(String(data=json.dumps(report)))
        self.numeric_log.emit(report['capture_stamp'], *[report[k] for k in
                              ('processing_ms','frames','height_cells','slope_cells','class_cells','sync_dropped')])
        if time.monotonic()-self.last_status > 5:
            self.get_logger().info(str({k:report[k] for k in ('frames','processing_ms','height_cells','slope_cells','class_cells')}))
            self.last_status = time.monotonic()

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()

    def publish(self, stamp, arrays):
        super().publish(stamp, arrays, grid=self.output_grid)
        self.map_stamp = (stamp.sec*1000000000+stamp.nanosec)*1e-9
        maps = self.latest_costmaps
        g = self.output_grid
        self.latest_costmaps = maps
        for pub, data in zip(self.navpub, maps):
            m = OccupancyGrid()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.info.resolution = float(g.resolution)
            m.info.width = g.width
            m.info.height = g.height
            m.info.origin.position.x = g.origin_x
            m.info.origin.position.y = g.origin_y
            m.info.origin.orientation.w = 1.0
            m.data = array("b", data.tobytes())
            pub.publish(m)
            if pub is self.navpub[0] and self.latest_features is not None:
                bundle = PlanningCostLayers()
                bundle.grid = m
                bundle.slope_cost = array('f', self.latest_features[0].ravel())
                bundle.terrain_cost = array('f', self.latest_features[1].ravel())
                if self.camera_support is not None:
                    import copy
                    support = copy.deepcopy(m)
                    support.data = array("b", self.camera_support.tobytes())
                    self.support_pub.publish(support)
                self.feature_pub.publish(bundle)
        self.map_wall = time.monotonic()
        self.trace("map_published", stamp_ns=stamp.sec*1000000000+stamp.nanosec,
                   topics=["/nomad/navigation_costmap", "/nomad/navigation_local_costmap"])

    def save(self, arrays=None, report=None):
        if not self.directory or not self.get_parameter("record_results").value:
            return
        arrays = arrays if arrays is not None else getattr(self, 'last_arrays', None)
        if arrays is None:
            return
        report = dict(report if report is not None else self.last_report)
        report.setdefault('capture_stamp', self.map_stamp)
        grid = getattr(self, 'output_grid', None) or self.layers.views.grid
        bootstrap = getattr(self, 'bootstrap', None)
        persist_result(self.directory, arrays, self.latest_costmaps,
                       {k:getattr(grid,k) for k in ('origin_x','origin_y','resolution')},
                       report, bootstrap, self.scan_stats)
        self.last_save = time.monotonic()

    def health_tick(self):
        now = self.get_clock().now()
        sec = now.nanoseconds * 1e-9
        wall = time.monotonic()
        ready = (
            self.odom is not None
            and 0 <= sec - self.map_stamp < 2
            and wall - self.map_wall < 3
            and wall - self.odom_wall < 1
        )
        if self.odom is not None:
            age = sec - (
                self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
            )
            ready = ready and 0 <= age < 0.6
        if any("Non-monotonic" in e for e in self.errors):
            self.fault = "CLOCK_RESET: restart map"
        m = ModuleStatus()
        m.header.stamp = now.to_msg()
        m.header.frame_id = "odom"
        m.valid_for.nanosec = 600000000
        m.state = (
            ModuleStatus.FAULT
            if self.fault
            else ModuleStatus.READY if ready else ModuleStatus.WAITING
        )
        m.detail = self.fault or (
            "RGBD terrain + fused pose READY"
            if ready
            else "Waiting for fresh fused pose / RGBD map"
        )
        failures = []
        if self.odom is None: failures.append("no_odometry")
        if not 0 <= sec-self.map_stamp < 2: failures.append("map_capture_age")
        if not wall-self.map_wall < 3: failures.append("map_wall_age")
        if not wall-self.odom_wall < 1: failures.append("odom_wall_age")
        if self.odom is not None and not 0 <= age < .6: failures.append("odom_capture_age")
        if self.fault: failures.append(self.fault)
        self.trace("health", state=int(m.state), reasons=failures,
                   map_capture_age_s=sec-self.map_stamp if math.isfinite(self.map_stamp) else None,
                   map_wall_age_s=wall-self.map_wall, odom_wall_age_s=wall-self.odom_wall,
                   odom_capture_age_s=age if self.odom is not None else None)
        self.health.publish(m)


def main():
    rclpy.init()
    n = NavigationTerrain()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.worker.close()
        n.save()
        n.destroy_node()
        rclpy.shutdown() if rclpy.ok() else None

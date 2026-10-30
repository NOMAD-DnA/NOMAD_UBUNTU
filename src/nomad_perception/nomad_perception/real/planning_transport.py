"""NOMAD dev transport contract. No cost computation or frame relabeling.

The recording pipeline uses TimingDiagnostics only. PlanningSnapshotPublisher
accepts already prepared odom maps from a future cost/frame adapter; it never
turns slope angles into costs or infers navigation readiness from transport alone.
"""
from copy import deepcopy
import json
import math
import time
from pathlib import Path

from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def map_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def status_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def set_stamp(header, capture_ns):
    from .capture_timing import positive_ns
    capture_ns = positive_ns(capture_ns)
    header.stamp.sec, header.stamp.nanosec = divmod(capture_ns, 1_000_000_000)


def grid_signature(grid):
    i, h, q = grid.info, grid.header, grid.info.origin.orientation
    return (h.frame_id, h.stamp.sec, h.stamp.nanosec, i.width, i.height,
            i.resolution, i.origin.position.x, i.origin.position.y,
            i.origin.position.z, q.x, q.y, q.z, q.w)


def validate_grid(grid, capture_ns, frame='odom'):
    h, i, q = grid.header, grid.info, grid.info.origin.orientation
    values = (i.resolution, i.origin.position.x, i.origin.position.y,
              i.origin.position.z, q.x, q.y, q.z, q.w)
    if (h.frame_id != frame or h.stamp.sec * 1_000_000_000 + h.stamp.nanosec != capture_ns
            or not 0 <= h.stamp.nanosec < 1_000_000_000
            or i.width <= 0 or i.height <= 0 or i.resolution <= 0
            or not all(map(math.isfinite, values))
            or abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(q.w - 1.) > 1e-6
            or len(grid.data) != i.width * i.height
            or any(v < -1 or v > 100 for v in grid.data)):
        raise ValueError('Invalid Planning grid frame, capture stamp or geometry')


class PlanningSnapshotPublisher:
    """Publish a coherent bundle and expire status without refreshing map stamps."""
    def __init__(self, node, *, frame='odom', max_age_s=2., max_wall_age_s=3., readiness=None):
        from nav_msgs.msg import OccupancyGrid
        from nomad_interfaces.msg import PlanningCostLayers, ModuleStatus
        if frame != 'odom' or not all(math.isfinite(v) and v > 0 for v in (max_age_s, max_wall_age_s)):
            raise ValueError('Expected dev odom contract and finite positive timeouts')
        self.node, self.frame, self.status_type = node, frame, ModuleStatus
        self.readiness=readiness
        self.max_age_ns, self.max_wall_age_s = int(max_age_s * 1e9), max_wall_age_s
        self.maps = [node.create_publisher(cls, topic, map_qos()) for cls, topic in (
            (OccupancyGrid, '/nomad/navigation_costmap'),
            (OccupancyGrid, '/nomad/navigation_local_costmap'),
            (PlanningCostLayers, '/nomad/planning_cost_layers'),
            (OccupancyGrid, '/nomad/camera_observation_support'))]
        self.status = node.create_publisher(ModuleStatus, '/nomad/perception/status', status_qos())
        self.capture_ns = None
        self.received_wall = None
        self.navigation_validated = False
        self.clock_fault = False
        self.closed = False
        self.last_now = None
        self.reason = 'Waiting for validated costs and capture-time odom transforms'
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(.1, self.tick, clock=self.wall_clock)

    def publish(self, global_map, local_map, layers, support, *, capture_ns,
                navigation_validated=False):
        from .capture_timing import positive_ns
        try:
            capture_ns = positive_ns(capture_ns)
            if self.closed:
                raise ValueError('Publisher is closed')
            now = self.node.get_clock().now().nanoseconds
            if self.readiness is not None:
                valid,reason=self.readiness()
                if not valid:raise ValueError(reason)
            self._check_clock(now)
            if self.clock_fault or not -50_000_000 <= now - capture_ns <= self.max_age_ns:
                raise ValueError('Stale/future snapshot or clock reset')
            if self.capture_ns is not None and capture_ns <= self.capture_ns:
                raise ValueError('Duplicate/reordered snapshot cannot renew readiness')
            for grid in (global_map, local_map, layers.grid, support):
                validate_grid(grid, capture_ns, self.frame)
            if grid_signature(global_map) != grid_signature(layers.grid) or grid_signature(global_map) != grid_signature(support):
                raise ValueError('Cost layers and camera support must share one snapshot')
            if list(global_map.data) != list(layers.grid.data):
                raise ValueError('Cost-layer occupancy differs from global map')
            size = global_map.info.width * global_map.info.height
            for values in (layers.slope_cost, layers.terrain_cost):
                if len(values) != size or any(math.isinf(v) or (not math.isnan(v) and not 0 <= v <= 1) for v in values):
                    raise ValueError('Expected normalized feature arrays; unknown is NaN')
            if not navigation_validated:
                raise ValueError('Cost policy, source clocks and odom transforms are not validated')
        except ValueError as exc:
            self.invalidate(str(exc))
            return False
        try:
            # DDS topics are independent. Exact headers let consumers reject a
            # mixed generation; features/support precede standalone costmaps.
            for index, message in ((3, support), (2, layers), (0, global_map), (1, local_map)):
                self.maps[index].publish(message)
        except Exception:
            self.invalidate('Snapshot publication failed')
            raise
        self.capture_ns, self.received_wall = capture_ns, time.monotonic()
        self.navigation_validated = True
        self.reason = 'Validated perception snapshot'
        self.tick()
        return True

    def _check_clock(self, now):
        if self.last_now is not None and now < self.last_now - 50_000_000:
            self.clock_fault = True
        self.last_now = now

    def invalidate(self, reason):
        self.navigation_validated = False
        self.reason = reason
        self.tick()

    def tick(self):
        now = self.node.get_clock().now().nanoseconds
        self._check_clock(now)
        if self.readiness is not None:
            valid,reason=self.readiness()
            if not valid:self.navigation_validated=False;self.reason=reason
        ready = (self.navigation_validated and not self.clock_fault and self.capture_ns is not None
                 and -50_000_000 <= now - self.capture_ns <= self.max_age_ns
                 and time.monotonic() - self.received_wall <= self.max_wall_age_s)
        msg = self.status_type()
        msg.header.frame_id = self.frame
        # Status is a current health assertion. Maps keep capture time.
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.valid_for.nanosec = 300_000_000
        msg.state = msg.READY if ready else msg.STOPPED if self.capture_ns is not None else msg.WAITING
        msg.detail = self.reason if ready else 'Not ready: ' + self.reason + '; capture/clock/wall freshness required'
        self.status.publish(msg)

    def close(self):
        self.closed = True
        self.invalidate('Publisher closed')
        self.node.destroy_timer(self.timer)


class TimingDiagnostics:
    """Active recording diagnostics, deliberately separate from navigation status."""
    def __init__(self, node, directory, *, clock_verified=False, max_age_s=2., frame='world'):
        self.node, self.directory = node, Path(directory)
        self.frame=frame
        self.clock_verified, self.max_age_s = bool(clock_verified), max_age_s
        self.capture_pub = node.create_publisher(DiagnosticArray, '/nomad/semantic_validation/capture_timing', map_qos())
        self.status_pub = node.create_publisher(DiagnosticArray, '/nomad/semantic_validation/transport_status', status_qos())
        self.value = None
        self.last_sequence = None
        self.last_clock = None
        self.clock_fault = False
        self.closed = False
        self.enabled = True
        self.fault = None
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(.1, self.tick, clock=self.wall_clock)

    def _message(self, data, stamp_ns, level, message):
        msg = DiagnosticArray()
        msg.header.frame_id = self.frame
        if stamp_ns > 0:
            set_stamp(msg.header, stamp_ns)
        item = DiagnosticStatus(level=level, name='semantic_validation/timing',
                                hardware_id='diagnostic_only', message=message)
        item.values = [KeyValue(key=k, value=json.dumps(v, allow_nan=False, sort_keys=True)) for k, v in data.items()]
        msg.status = [item]
        return msg

    def report(self):
        now = self.node.get_clock().now().nanoseconds
        data = deepcopy(self.value) if self.value else {}
        capture = data.get('snapshot_capture_ns', 0)
        completed = data.get('worker_completed_monotonic_ns')
        age = (now - capture) / 1e9 if capture else None
        wall_age = (time.monotonic_ns() - completed) / 1e9 if completed is not None else None
        fresh = (self.enabled and not self.fault and not self.clock_fault and self.clock_verified
                 and age is not None and -.05 <= age <= self.max_age_s
                 and wall_age is not None and 0 <= wall_age <= 3.)
        return dict(data, capture_age_s=age, result_wall_age_s=wall_age,
                    contributing_source_age_s=[(now-t)/1e9 if t else None for t in data.get('source_capture_ns',[])],
                    clock_confirmation_from_config=self.clock_verified, timing_fresh=bool(fresh),
                    inspection_enabled=self.enabled, clock_fault=self.clock_fault,
                    error=self.fault, navigation_ready=False)

    def tick(self):
        now = self.node.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock - 50_000_000:
            self.clock_fault = True
        self.last_clock = now
        path = self.directory / 'terrain/timing.json'
        try:
            if path.exists():
                value = json.loads(path.read_text())
                sequence = value['sequence']
                if sequence != self.last_sequence:
                    capture = value['snapshot_capture_ns']
                    if self.value and capture < self.value['snapshot_capture_ns']:
                        self.clock_fault = True
                    self.value, self.last_sequence = value, sequence
                    # Kept with source time even when this snapshot is delayed.
                    self.capture_pub.publish(self._message(value, capture, DiagnosticStatus.WARN,
                                             'Capture provenance only; no navigation authorization'))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.fault = 'Invalid timing snapshot: ' + str(exc)
        data = self.report()
        level = DiagnosticStatus.ERROR if self.fault or self.clock_fault else DiagnosticStatus.WARN
        self.status_pub.publish(self._message(data, now, level,
            'Timing diagnostics only; ' + ('fresh' if data['timing_fresh'] else 'waiting/stale/unverified')))

    def close(self):
        self.closed = True
        self.enabled = False
        self.tick()
        self.node.destroy_timer(self.timer)

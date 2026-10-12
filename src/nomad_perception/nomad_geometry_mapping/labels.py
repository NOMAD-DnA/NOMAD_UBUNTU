"""Numeric slope labels kept separate from the mapping worker.

Each publication is a complete latched MarkerArray, including DELETEALL, so
late RViz subscribers receive the current full set rather than an incomplete
incremental update. Labels are throttled independently of map generation.
"""
import math
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def identity(message):
    h = message.header
    return h.stamp.sec, h.stamp.nanosec, h.frame_id


def build_labels(cost, angles, text_height=.06):
    """Keep map geometry and exact numeric values; never label unknown cells."""
    shape = (cost.info.height, cost.info.width)
    values = np.asarray(cost.data, dtype=np.int8).reshape(shape)
    angles = np.asarray(angles, dtype=np.float32)
    if angles.shape != shape:
        raise ValueError('Cost/slope shapes do not match')
    message = MarkerArray()
    reset = Marker()
    reset.header = cost.header
    reset.action = Marker.DELETEALL
    message.markers.append(reset)
    rows, cols = np.nonzero(values >= 0)
    resolution = cost.info.resolution
    for index, (y, x) in enumerate(zip(rows, cols)):
        marker = Marker()
        marker.header = cost.header
        marker.ns, marker.id = 'slope_degrees', index
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.position.x = float(cost.info.origin.position.x + (x + .5) * resolution)
        marker.pose.position.y = float(cost.info.origin.position.y + (y + .5) * resolution)
        marker.pose.position.z = .06
        marker.pose.orientation.w = 1.
        marker.scale.z = float(text_height)
        shade = .03 if values[y, x] < 90 else 1.
        marker.color.r = marker.color.g = marker.color.b = shade
        marker.color.a = 1.
        angle = float(angles[y, x])
        # The requested live and reopened RViz view shows numbers only.
        # Degree symbols are retained in the exported numbered picture.
        marker.text = f'{angle:.1f}' if math.isfinite(angle) else 'N/A'
        message.markers.append(marker)
    return message


def build_cost_cells(cost):
    """Render exact G values without the RViz indexed-image shader."""
    values = np.asarray(cost.data, dtype=np.int8).reshape(cost.info.height, cost.info.width)
    rows, columns = np.nonzero(values >= 0)
    palette = np.array([[1., 1., 1.], [86/255, 196/255, 184/255],
                        [1., 228/255, 94/255], [232/255, 72/255, 53/255], [0., 0., 0.]])
    scores = values[rows, columns].astype(float)
    rgb = np.stack([np.interp(scores, [0., 25., 50., 75., 100.], palette[:, i])
                    for i in range(3)], axis=1)
    marker = Marker()
    marker.header = cost.header
    marker.ns, marker.id, marker.type, marker.action = 'geometry_cost', 0, Marker.CUBE_LIST, Marker.ADD
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = float(cost.info.resolution * .98)
    marker.scale.z = .004
    marker.color.a = 1.
    for y, x, color in zip(rows, columns, rgb):
        marker.points.append(Point(x=float(cost.info.origin.position.x + (x + .5) * cost.info.resolution),
                                  y=float(cost.info.origin.position.y + (y + .5) * cost.info.resolution), z=0.))
        marker.colors.append(ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=1.))
    return marker


class SlopeLabels(Node):
    def __init__(self):
        super().__init__('nomad_slope_labels')
        self.text_height = float(self.declare_parameter('text_height_m', .06).value)
        period = float(self.declare_parameter('refresh_period_s', 1.).value)
        if not (math.isfinite(self.text_height) and self.text_height > 0 and
                math.isfinite(period) and period > 0):
            raise ValueError('Invalid label height or refresh period')
        self.costs, self.angles = {}, {}
        self.ready = None
        self.dirty = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/manual_mapping/slope_labels', qos)
        self.cells_pub = self.create_publisher(Marker, '/manual_mapping/cost_cells', qos)
        self.create_subscription(OccupancyGrid, '/manual_mapping/costmap', self.on_cost, qos)
        self.create_subscription(Image, '/manual_mapping/slope_degrees', self.on_angle, qos)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.publish, clock=steady)

    def on_cost(self, msg):
        key = identity(msg)
        self.costs[key] = msg
        self.pair(key)

    def on_angle(self, msg):
        if msg.encoding != '32FC1' or msg.step != msg.width * 4:
            self.get_logger().warning('Labels require packed 32FC1 slope degrees')
            return
        key = identity(msg)
        dtype = '>f4' if msg.is_bigendian else '<f4'
        try:
            data = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width)
        except ValueError:
            return
        self.angles[key] = data
        self.pair(key)

    def pair(self, key):
        if key in self.costs and key in self.angles:
            cost, angles = self.costs[key], self.angles[key]
            if angles.shape == (cost.info.height, cost.info.width):
                self.ready = (cost, angles)
                self.dirty = True
        # Independently bounded queues preserve both arrival orders.
        for pending in (self.costs, self.angles):
            while len(pending) > 4:
                pending.pop(next(iter(pending)))

    def publish(self):
        if not self.dirty or self.ready is None:
            return
        began = time.perf_counter()
        message = build_labels(*self.ready, self.text_height)
        self.cells_pub.publish(build_cost_cells(self.ready[0]))
        self.publisher.publish(message)
        self.dirty = False
        if time.perf_counter() - began > 1.:
            self.get_logger().warning('Dense slope labels take over 1 second; '
                                      'RViz display can be toggled off for smoother viewing')


def main(args=None):
    rclpy.init(args=args)
    node = SlopeLabels()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

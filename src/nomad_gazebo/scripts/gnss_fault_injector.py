#!/usr/bin/env python3
"""Manually run GNSS error injection. Never publishes odometry or TF."""
import copy
import heapq
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import NavSatFix
from nomad_interfaces.msg import GnssQuality
from gnss_fault_model import GNSSFaultModel


class GNSSInjector(Node):
    def __init__(self):
        super().__init__('nomad_gnss_fault_injector')
        default = str(Path(get_package_share_directory('nomad_gazebo'))/'config/gnss_sim.yaml')
        path = self.declare_parameter('profile_path', default).value
        source = self.declare_parameter('input_topic', '/gps/fix').value
        target = self.declare_parameter('output_topic', '/experiment/gps/fix').value
        quality = self.declare_parameter('quality_topic', '/experiment/gps/quality').value
        if len({source, target, quality}) != 3:
            raise ValueError('Input, fix output and quality output must be distinct topics')
        self.model = GNSSFaultModel(yaml.safe_load(Path(path).read_text()))
        self.fix_pub = self.create_publisher(NavSatFix, target, 50)
        self.quality_pub = self.create_publisher(GnssQuality, quality, 50)
        self.sub = self.create_subscription(NavSatFix, source, self.receive, qos_profile_sensor_data)
        self.queue, self.seq, self.last_stamp, self.last_clock = [], 0, None, None
        self.timer = self.create_timer(.02, self.flush)
        self.get_logger().info(f'{source} -> {target} + {quality}; profile={path}; manual execution only')

    def receive(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.queue.clear()  # Do not leak delayed packets across bag rewind.
        self.last_stamp = stamp
        pos = (msg.latitude, msg.longitude, msg.altitude)
        if msg.status.status >= 0 and (not all(math.isfinite(v) for v in pos)
                                      or not -90 <= pos[0] <= 90 or not -180 <= pos[1] <= 180):
            self.get_logger().warning('Ignoring invalid source coordinates')
            return
        packet = dict(stamp_ns=stamp, position=pos, status=msg.status.status,
                      covariance=list(msg.position_covariance), covariance_type=msg.position_covariance_type)
        result = self.model.process(packet)
        if result is None:
            return
        fix = copy.deepcopy(msg)
        fix.header.stamp.sec, fix.header.stamp.nanosec = divmod(result['stamp_ns'], 10**9)
        fix.latitude, fix.longitude, fix.altitude = result['position']
        fix.status.status = result['status']
        fix.position_covariance, fix.position_covariance_type = result['covariance'], result['covariance_type']
        q = GnssQuality()
        q.header = copy.deepcopy(fix.header)
        values = result['quality']
        q.solution_type = values['solution_type']
        q.valid_fields = q.SOLUTION_VALID if q.solution_type != q.UNKNOWN else 0
        q.pdop = q.correction_age_s = float('nan')
        for name, bit in (('satellites_used', q.SATELLITES_VALID), ('pdop', q.PDOP_VALID),
                          ('correction_age_s', q.CORRECTION_AGE_VALID)):
            if name in values:
                setattr(q, name, values[name])
                q.valid_fields |= bit
        if result['delay_s']:
            if len(self.queue) >= 5000:
                self.get_logger().error('GNSS delay queue limit reached; dropping packet')
                return
            due = self.get_clock().now().nanoseconds+int(result['delay_s']*10**9)
            self.seq += 1
            heapq.heappush(self.queue, (due, self.seq, fix, q))
        else:
            self.publish(fix, q)

    def publish(self, fix, quality):
        self.fix_pub.publish(fix)
        self.quality_pub.publish(quality)

    def flush(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.queue.clear()
            self.model.reset()
            self.last_stamp = None
        self.last_clock = now
        while self.queue and self.queue[0][0] <= now:
            _, _, fix, quality = heapq.heappop(self.queue)
            self.publish(fix, quality)


def main():
    rclpy.init()
    node = None
    try:
        node = GNSSInjector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

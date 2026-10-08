#!/usr/bin/env python3
"""NavSatFix -> WGS84 local ENU RViz markers. Visualization only, no odometry/TF."""
import math
from collections import deque


def geodetic_to_ecef(latitude, longitude, altitude):
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
        raise ValueError('GPS coordinates must be finite')
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError('GPS latitude/longitude out of range')
    lat, lon = math.radians(latitude), math.radians(longitude)
    a, e2 = 6378137.0, 6.6943799901413165e-3
    n = a / math.sqrt(1.0 - e2 * math.sin(lat)**2)
    return ((n + altitude) * math.cos(lat) * math.cos(lon),
            (n + altitude) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - e2) + altitude) * math.sin(lat))


class LocalENU:
    def __init__(self, latitude, longitude, altitude):
        self.origin = geodetic_to_ecef(latitude, longitude, altitude)
        lat, lon = math.radians(latitude), math.radians(longitude)
        self.rotation = ((-math.sin(lon), math.cos(lon), 0.0),
                         (-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)),
                         (math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)))

    def convert(self, latitude, longitude, altitude):
        ecef = geodetic_to_ecef(latitude, longitude, altitude)
        delta = [v-o for v, o in zip(ecef, self.origin)]
        return tuple(sum(v*d for v, d in zip(row, delta)) for row in self.rotation)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from visualization_msgs.msg import Marker, MarkerArray

    class GPSVisualizer(Node):
        def __init__(self):
            super().__init__('nomad_gps_visualizer')
            # Explicit shared datum: never silently use the first GPS fix as origin.
            datum = [float(self.declare_parameter(k, v).value) for k, v in (
                ('origin_latitude_deg', 37.0), ('origin_longitude_deg', 127.0), ('origin_elevation_m', 0.0))]
            self.enu = LocalENU(*datum)
            self.frame = self.declare_parameter('fixed_frame', 'odom').value
            count = self.declare_parameter('max_points', 1000).value
            if not self.frame or not isinstance(count, int) or not 2 <= count <= 10000:
                raise ValueError('fixed_frame required; max_points must be 2..10000')
            self.points = deque(maxlen=count)
            self.last_stamp = None
            self.publisher = self.create_publisher(MarkerArray, '/nomad/visualization/gps', 1)
            self.subscription = self.create_subscription(NavSatFix, '/gps/fix', self.receive, qos_profile_sensor_data)
            self.get_logger().info(f'GPS visualization: WGS84 {datum} -> {self.frame} ENU meters; antenna position only')

        def marker(self, message, marker_id):
            m = Marker()
            m.header.stamp = message.header.stamp
            m.header.frame_id = self.frame
            m.ns, m.id = 'gps', marker_id
            m.pose.orientation.w = 1.0  # Shape orientation, not a measured GPS attitude.
            m.color.r, m.color.g, m.color.b, m.color.a = 0.75, 0.25, 1.0, 1.0
            m.lifetime.sec = 2  # Expire when simulation/ROS time advances without new fixes.
            return m

        def receive(self, message):
            stamp = message.header.stamp.sec*1000000000 + message.header.stamp.nanosec
            if message.status.status < NavSatStatus.STATUS_FIX:
                self.clear(message)
                return
            try:
                xyz = self.enu.convert(message.latitude, message.longitude, message.altitude)
            except ValueError:
                self.clear(message)
                return
            if self.last_stamp is not None:
                if stamp < self.last_stamp:
                    self.points.clear()  # Bag rewind or simulation restart.
                elif stamp == self.last_stamp:
                    return
            self.last_stamp = stamp
            point = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            self.points.append(point)
            current = self.marker(message, 0)
            current.type = Marker.SPHERE
            current.pose.position = point
            current.scale.x = current.scale.y = current.scale.z = 0.35
            trail = self.marker(message, 1)
            trail.type = Marker.LINE_STRIP
            trail.scale.x = 0.06
            trail.color.a = 0.75
            trail.points = list(self.points)
            if len(trail.points) < 2:
                trail.action = Marker.DELETE
            self.publisher.publish(MarkerArray(markers=[current, trail]))

        def clear(self, message):
            self.points.clear()
            self.last_stamp = None
            markers = [self.marker(message, i) for i in (0, 1)]
            for marker in markers:
                marker.action = Marker.DELETE
            self.publisher.publish(MarkerArray(markers=markers))

    rclpy.init()
    node = GPSVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

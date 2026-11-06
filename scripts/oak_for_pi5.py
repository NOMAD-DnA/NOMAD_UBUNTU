#!/usr/bin/env python3
"""OAK-D Lite stereo/IMU ROS2 publisher using the verified DepthAI 2.32 runtime."""

import os
import time
from pathlib import Path

import depthai as dai
import rclpy
import yaml
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu


CONFIG = Path(os.environ.get('NOMAD_OAK_CONFIG', '/home/user/nomad_ws/config/oak_stereo.yaml'))
NS = 1_000_000_000


def load_fps():
    params = yaml.safe_load(CONFIG.read_text())['/oak']['ros__parameters']
    left, right = params['left'], params['right']
    for camera in (left, right):
        if (int(camera['i_width']), int(camera['i_height'])) != (640, 480):
            raise ValueError('OAK/Basalt calibration requires both cameras at 640x480')
    if float(left['i_fps']) != float(right['i_fps']):
        raise ValueError('Stereo camera FPS must match')
    fps = float(left['i_fps'])
    stereo_params = params.get('stereo', {})
    depth_fps = float(stereo_params.get('i_fps', fps))
    if not 0 < depth_fps <= fps:
        raise ValueError('Depth output FPS must be positive and no higher than stereo camera FPS')
    return fps, bool(stereo_params.get('i_publish_topic', False)), depth_fps


def pipeline(fps, publish_depth, depth_fps):
    p = dai.Pipeline()
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    for camera, socket in ((left, dai.CameraBoardSocket.CAM_B),
                           (right, dai.CameraBoardSocket.CAM_C)):
        camera.setBoardSocket(socket)
        camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        camera.setFps(fps)

    stereo = p.create(dai.node.StereoDepth)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    outputs = [('left', stereo.rectifiedLeft),
               ('right', stereo.rectifiedRight),
               ('imu', imu.out)]
    if publish_depth:
        outputs.append(('depth', stereo.depth))
    for name, output in outputs:
        xout = p.create(dai.node.XLinkOut)
        xout.setStreamName(name)
        if name == 'depth':
            xout.input.setBlocking(False)
            xout.input.setQueueSize(1)
        output.link(xout.input)
    return p, tuple(name for name, _ in outputs)


def ros_stamp(msg, epoch_offset):
    ns = int(msg.getTimestamp().total_seconds() * NS) + epoch_offset
    return ns // NS, ns % NS


def image_msg(frame, stamp, frame_id, encoding):
    msg = Image()
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
    msg.header.frame_id = frame_id
    msg.height = frame.getHeight()
    msg.width = frame.getWidth()
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = msg.width * (2 if encoding == '16UC1' else 1)
    msg.data = frame.getData().tobytes()
    return msg


def imu_msg(packet, epoch_offset):
    accel, gyro = packet.acceleroMeter, packet.gyroscope
    msg = Imu()
    msg.header.stamp.sec, msg.header.stamp.nanosec = ros_stamp(gyro, epoch_offset)
    msg.header.frame_id = 'oak_imu_frame'
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro.x, gyro.y, gyro.z
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel.x, accel.y, accel.z
    return msg


def main():
    fps, publish_depth, depth_fps = load_fps()
    p, names = pipeline(fps, publish_depth, depth_fps)
    rclpy.init()
    node = rclpy.create_node('oak')
    image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                           reliability=ReliabilityPolicy.RELIABLE)
    imu_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                         reliability=ReliabilityPolicy.RELIABLE)
    publishers = {
        'left': node.create_publisher(Image, '/oak/left/image_rect', image_qos),
        'right': node.create_publisher(Image, '/oak/right/image_rect', image_qos),
        'imu': node.create_publisher(Imu, '/oak/imu/data', imu_qos),
    }
    if publish_depth:
        publishers['depth'] = node.create_publisher(Image, '/oak/stereo/image_raw', image_qos)
    try:
        with dai.Device(p) as device:
            node.get_logger().info(f'DepthAI {dai.__version__}; USB {device.getUsbSpeed()}; stereo {fps:g} fps; depth output {depth_fps:g} fps (enabled={publish_depth})')
            queues = {name: device.getOutputQueue(name, maxSize=(256 if name == 'imu' else (1 if name == 'depth' else 8)), blocking=False) for name in names}
            epoch_offset = time.time_ns() - time.monotonic_ns()
            pending = {}
            counts = {'stereo': 0, 'imu': 0, 'depth': 0}
            last_report = time.monotonic()
            next_depth_publish = 0.0
            while rclpy.ok():
                for name in ('imu', 'left', 'right', 'depth'):
                    if name not in queues:
                        continue
                    for frame in queues[name].tryGetAll():
                        if name in ('left', 'right'):
                            seq = frame.getSequenceNum()
                            pair = pending.setdefault(seq, {})
                            pair[name] = frame
                            if len(pair) == 2:
                                stamp = ros_stamp(pair['left'], epoch_offset)
                                for side in ('left', 'right'):
                                    publishers[side].publish(image_msg(
                                        pair[side], stamp, f'oak_{side}_camera_optical_frame', 'mono8'))
                                counts['stereo'] += 1
                                del pending[seq]
                            if len(pending) > 20:
                                del pending[min(pending)]
                        elif name == 'imu':
                            for packet in frame.packets:
                                publishers['imu'].publish(imu_msg(packet, epoch_offset))
                                counts['imu'] += 1
                        else:
                            depth_now = time.monotonic()
                            if depth_now < next_depth_publish:
                                continue
                            next_depth_publish = depth_now + 1.0 / depth_fps
                            publishers['depth'].publish(image_msg(
                                frame, ros_stamp(frame, epoch_offset),
                                'oak_right_camera_optical_frame', '16UC1'))
                            counts['depth'] += 1
                now = time.monotonic()
                if now - last_report >= 5:
                    node.get_logger().info(f'last 5s: stereo={counts["stereo"]}, imu={counts["imu"]}, depth={counts["depth"]}')
                    counts = dict.fromkeys(counts, 0)
                    last_report = now
                time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

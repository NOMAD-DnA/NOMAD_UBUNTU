"""Physical OAK stereo and raw IMU input for the installed Basalt ROS nodes.

Depth snapshots are observations only. This process never estimates a pose or
accumulates a map. Stereo and IMU timestamps share one device-to-host offset.
"""
import argparse
import bisect
import collections
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import numpy as np

def device_ns(delta):
    """Avoid conversion of a device timedelta through floating-point seconds."""
    return ((delta.days * 86400 + delta.seconds) * 1000000 + delta.microseconds) * 1000

def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    os.replace(temporary, path)

class ImuResampler:
    """Interpolate gyro measurements onto measured accelerometer timestamps.

    Samples wait for a bracketing gyro measurement. No extrapolation, reused
    timestamps, or invented measurements are allowed.
    """

    def __init__(self, max_gap_ns=30000000, max_pending=4000):
        self.accel = collections.deque()
        self.gyro = []
        self.gyro_times = []
        self.last_accel_ns = -1
        self.last_gyro_ns = -1
        self.max_gap_ns = max_gap_ns
        self.max_pending = max_pending
        self.duplicate_accel = 0
        self.duplicate_gyro = 0
        self.dropped_accel = 0
        self.gyro_samples = 0
        self.accel_samples = 0

    def add(self, accel_ns, accel, gyro_ns, gyro):
        a = np.asarray(accel, dtype=float)
        g = np.asarray(gyro, dtype=float)
        if a.shape != (3,) or g.shape != (3,) or (not np.isfinite([a, g]).all()):
            raise ValueError('Non-finite or malformed physical IMU measurement')
        if accel_ns > self.last_accel_ns:
            self.accel.append((accel_ns, a))
            self.last_accel_ns = accel_ns
            self.accel_samples += 1
        else:
            self.duplicate_accel += 1
        if gyro_ns > self.last_gyro_ns:
            self.gyro_times.append(gyro_ns)
            self.gyro.append(g)
            self.last_gyro_ns = gyro_ns
            self.gyro_samples += 1
        else:
            self.duplicate_gyro += 1
        while len(self.accel) > self.max_pending:
            self.accel.popleft()
            self.dropped_accel += 1

    def take_ready(self):
        result = []
        while self.accel and self.gyro_times:
            stamp, accel = self.accel[0]
            upper = bisect.bisect_left(self.gyro_times, stamp)
            if upper == len(self.gyro_times):
                break
            if self.gyro_times[upper] == stamp:
                gyro = self.gyro[upper].copy()
            elif upper == 0:
                self.accel.popleft()
                self.dropped_accel += 1
                continue
            else:
                before, after = (self.gyro_times[upper - 1], self.gyro_times[upper])
                if after - before > self.max_gap_ns:
                    self.accel.popleft()
                    self.dropped_accel += 1
                    continue
                weight = (stamp - before) / (after - before)
                gyro = (1 - weight) * self.gyro[upper - 1] + weight * self.gyro[upper]
            self.accel.popleft()
            result.append((stamp, accel, gyro))
        cutoff = self.accel[0][0] if self.accel else self.last_accel_ns
        keep = max(0, bisect.bisect_right(self.gyro_times, cutoff) - 1)
        if keep:
            del self.gyro_times[:keep]
            del self.gyro[:keep]
        if len(self.gyro_times) > self.max_pending:
            del self.gyro_times[:-self.max_pending]
            del self.gyro[:-self.max_pending]
        return result

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ready-file', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--cloud-fps', type=float, default=2.0)
    parser.add_argument('--publish-cloud', action='store_true', default=True, help='Publish sampled CAM_A depth XYZ for the shared map, without dense snapshot recording')
    parser.add_argument('--depth-stride', type=int, default=4)
    parser.add_argument('--depth-min-m', type=float, default=0.3)
    parser.add_argument('--depth-max-m', type=float, default=8.0)
    parser.add_argument('--legacy-unrectified-vio', action='store_true', help='Diagnostic comparison only: keep the old unrectified VIO geometry')
    parser.add_argument('--camera-intrinsics', type=Path, help='Run-only raw CAM_B K/D override')
    parser.add_argument('--device-id', default='')
    parser.add_argument('--usb', choices=['HIGH', 'SUPER'], default='HIGH')
    parser.add_argument('--imu-extrinsics', choices=['factory', 'nominal'], default='factory')
    parser.add_argument('--vio-config-template', type=Path, default=__import__('ament_index_python.packages', fromlist=['get_package_share_directory']).get_package_share_directory('nomad_perception') + '/config/real/nomad_vio_config.json')
    if argv is None:
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 <= args.duration <= 600:
        parser.error('--duration must be between 0 and 600 seconds; 0 means live')
    if not math.isfinite(args.fps) or not 5 <= args.fps <= 60:
        parser.error('--fps must be between 5 and 60')
    if not math.isfinite(args.cloud_fps) or not 0 < args.cloud_fps <= 5:
        parser.error('--cloud-fps must be greater than zero and at most 5')
    if args.depth_stride < 1 or not 0 < args.depth_min_m < args.depth_max_m < float('inf'):
        parser.error('Invalid depth stride or range')
    return args

def main(argv=None):
    args = parse_args(argv)
    import depthai as dai
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, Imu, PointCloud2
    from .oak_basalt_calibration import make_calibration
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config_dir = output / 'config'
    config_dir.mkdir(exist_ok=True)
    ready_path = args.ready_file.resolve()
    ready_path.parent.mkdir(parents=True, exist_ok=True)
    if ready_path.exists():
        raise RuntimeError('Ready file already exists; use a new session directory')
    if (output / 'metadata.json').exists():
        raise RuntimeError('Output already contains a session; use a new directory')
    template = json.loads(args.vio_config_template.read_text())
    atomic_json(config_dir / 'nomad_vio_config.json', template)
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    stats = {'state': 'starting', 'stereo_pairs': 0, 'imu_published': 0, 'unpaired_left': 0, 'unpaired_right': 0, 'cloud_published': 0, 'max_pair_difference_ms': 0.0, 'device_host_offset_ns': None, 'first_frame_stamp_ns': None, 'last_frame_stamp_ns': None, 'first_imu_stamp_ns': None, 'last_imu_stamp_ns': None, 'error': None, 'source': 'physical OAK camera', 'map_accumulation': False}
    metadata = None
    device = None
    pipeline = None
    node = None
    started = False
    resampler = ImuResampler()
    start_wall = time.monotonic()
    first_data_wall = None
    offset_ns = None
    last_stereo_device_ns = None
    last_pair_wall = 0.
    last_stats_wall = 0.0
    last_cloud_device_ns = None
    final_error = None
    vio_maps = None
    rectification = None
    intrinsics_override = None
    try:
        rclpy.init(args=[])
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        node = Node('oak_basalt_physical_bridge')
        qos_image = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        qos_imu = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        left_pub = node.create_publisher(Image, '/oak/left/image_rect', qos_image)
        right_pub = node.create_publisher(Image, '/oak/right/image_rect', qos_image)
        imu_pub = node.create_publisher(Imu, '/oak/imu', qos_imu)
        depth_pub = None
        if args.publish_cloud:
            from .ros_cloud import cloud_message
            from .depth_core import sample_depth, DEPTH_FRAME
            from rclpy.qos import qos_profile_sensor_data
            depth_pub = node.create_publisher(PointCloud2, '/oak/depth/points', qos_profile_sensor_data)
        devices = dai.Device.getAllAvailableDevices()
        if args.device_id:
            devices = [d for d in devices if d.getDeviceId() == args.device_id]
        if len(devices) != 1:
            if not devices:
                connected = dai.Device.getAllConnectedDevices()
                if args.device_id:
                    connected = [d for d in connected if d.getDeviceId() == args.device_id]
                booted = [d.getDeviceId() for d in connected if d.state == dai.XLinkDeviceState.X_LINK_BOOTED]
                if booted:
                    raise RuntimeError('OAK camera is already running: '+', '.join(booted))
            raise RuntimeError(f'Expected exactly one OAK camera, found {len(devices)}')
        device = dai.Device(devices[0], maxUsbSpeed=getattr(dai.UsbSpeed, args.usb))
        imu_name = device.getConnectedIMU()
        if imu_name != 'BMI270':
            raise RuntimeError(f'This physical profile requires BMI270, found {imu_name!r}')
        previous_active_calibration = device.readCalibration()
        calibration = previous_active_calibration
        factory = calibration.eepromToJson()
        if args.camera_intrinsics is not None:
            from .camera_intrinsics_override import apply_model
            intrinsics_override = apply_model(calibration, args.camera_intrinsics, dai)
            device.setCalibration(calibration)
            atomic_json(output / 'runtime_intrinsics.json', intrinsics_override)
        pipeline = dai.Pipeline(device)
        if intrinsics_override is not None:
            pipeline.setCalibrationData(calibration)
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        size = (640, 480)
        left_out = left.requestOutput(size, type=dai.ImgFrame.Type.GRAY8, fps=args.fps, enableUndistortion=True)
        right_out = right.requestOutput(size, type=dai.ImgFrame.Type.GRAY8, fps=args.fps, enableUndistortion=True)
        left_queue = left_out.createOutputQueue(maxSize=30, blocking=False)
        right_queue = right_out.createOutputQueue(maxSize=30, blocking=False)
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(5)
        imu.setMaxBatchReports(20)
        imu_queue = imu.out.createOutputQueue(maxSize=200, blocking=False)
        cloud_queue = None
        if args.publish_cloud:
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            depth_fps = args.cloud_fps
            left.requestOutput(size, type=dai.ImgFrame.Type.GRAY8, fps=depth_fps, enableUndistortion=False).link(stereo.left)
            right.requestOutput(size, type=dai.ImgFrame.Type.GRAY8, fps=depth_fps, enableUndistortion=False).link(stereo.right)
            pc = pipeline.create(dai.node.PointCloud)
            pc.setRunOnHost(True)
            pc.initialConfig.setOrganized(True)
            pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
            pc.initialConfig.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_A)
            stereo.depth.link(pc.inputDepth)
            cloud_queue = pc.outputPointCloud.createOutputQueue(maxSize=2, blocking=False)
        pipeline.start()
        started = True
        left_pending = collections.deque()
        right_pending = collections.deque()
        print(f'Physical OAK {device.getDeviceInfo().getDeviceId()} started', flush=True)

        def stamp_message(message, stamp_ns, frame_id):
            message.header.stamp.sec = stamp_ns // 1000000000
            message.header.stamp.nanosec = stamp_ns % 1000000000
            message.header.frame_id = frame_id

        def publish_image(publisher, pixels, stamp_ns, frame_id):
            message = Image()
            stamp_message(message, stamp_ns, frame_id)
            message.height, message.width = pixels.shape
            message.encoding = 'mono8'
            message.is_bigendian = False
            message.step = pixels.shape[1]
            message.data = pixels.tobytes()
            publisher.publish(message)
        while not stop and rclpy.ok():
            now = time.monotonic()
            if args.duration > 0 and first_data_wall is not None and (now - first_data_wall >= args.duration):
                break
            if first_data_wall is None and now - start_wall > 35:
                raise RuntimeError('No synchronized stereo pair within 35 seconds')
            if first_data_wall is not None and now - last_pair_wall > 5:
                raise RuntimeError('Physical stereo stream stopped for 5 seconds')
            rclpy.spin_once(node, timeout_sec=0)
            for queue, pending in ((left_queue, left_pending), (right_queue, right_pending)):
                while True:
                    frame = queue.tryGet()
                    if frame is None:
                        break
                    pending.append(frame)
            for pending, key in ((left_pending, 'unpaired_left'), (right_pending, 'unpaired_right')):
                while len(pending) > 60:
                    pending.popleft()
                    stats[key] += 1
            while left_pending and right_pending:
                lf, rf = (left_pending[0], right_pending[0])
                lt = device_ns(lf.getTimestampDevice(dai.CameraExposureOffset.MIDDLE))
                rt = device_ns(rf.getTimestampDevice(dai.CameraExposureOffset.MIDDLE))
                if abs(lt - rt) > 1000000:
                    if lt < rt:
                        left_pending.popleft()
                        stats['unpaired_left'] += 1
                    else:
                        right_pending.popleft()
                        stats['unpaired_right'] += 1
                    continue
                left_pending.popleft()
                right_pending.popleft()
                pair_device_ns = (lt + rt) // 2
                if last_stereo_device_ns is not None and pair_device_ns <= last_stereo_device_ns:
                    raise RuntimeError('Physical image device timestamps did not increase')
                last_stereo_device_ns = pair_device_ns
                lp = np.ascontiguousarray(lf.getCvFrame())
                rp = np.ascontiguousarray(rf.getCvFrame())
                if lp.shape != (480, 640) or rp.shape != (480, 640) or lp.dtype != np.uint8 or (rp.dtype != np.uint8):
                    raise RuntimeError(f'Expected mono8 640x480, got {lp.shape}/{lp.dtype} {rp.shape}/{rp.dtype}')
                if offset_ns is None:
                    pair_host_ns = (device_ns(lf.getTimestamp(dai.CameraExposureOffset.MIDDLE)) + device_ns(rf.getTimestamp(dai.CameraExposureOffset.MIDDLE))) // 2
                    offset_ns = time.time_ns() - device_ns(dai.Clock.now()) + pair_host_ns - pair_device_ns
                    stats['device_host_offset_ns'] = offset_ns
                    left_K = np.asarray(lf.getTransformation().getIntrinsicMatrix(), dtype=float)
                    right_K = np.asarray(rf.getTransformation().getIntrinsicMatrix(), dtype=float)
                    baseline_calib, provenance = make_calibration(calibration, left_K, right_K, resolution=size, imu_name=imu_name, imu_rate_hz=200.0, dai=dai, imu_extrinsics=args.imu_extrinsics)
                    if not args.legacy_unrectified_vio:
                        from .stereo_rectification import prepare
                        baseline_calib, vio_maps, rectification = prepare(baseline_calib, provenance, left_K, right_K, size)
                        provenance['vio_rectification'] = rectification
                        provenance['image_geometry'] = 'VIO images rectified on host; T_raw_imu_cameras retain raw optical geometry for mapping'
                    atomic_json(config_dir / 'nomad_calib.json', baseline_calib)
                    metadata = {'source': 'physical OAK camera', 'depthai_version': dai.__version__, 'device_id': device.getDeviceInfo().getDeviceId(), 'usb_speed': str(device.getUsbSpeed()), 'factory_calibration': factory, 'intrinsics_override': intrinsics_override, 'left_output_intrinsics': left_K.tolist(), 'right_output_intrinsics': right_K.tolist(), 'vio_stereo_rectification': rectification, 'resolution': list(size), 'requested_camera_fps': args.fps, 'requested_imu_rate_hz': 200, 'imu_reports': ['ACCELEROMETER_RAW', 'GYROSCOPE_RAW'], 'imu_published_frame': 'raw BMI270; no extra rotation', 'imu_resampling': 'gyro linearly interpolated to accelerometer timestamps; no extrapolation', 'image_undistortion': 'DepthAI undistortion followed by host stereo rectification with matching virtual-camera calibration' if vio_maps is not None else 'DepthAI Camera output with actual output intrinsic matrix', 'stereo_pair_tolerance_ns': 1000000, 'device_host_offset_ns': offset_ns, 'timestamp_policy': 'one SDK host-synchronized exposure timestamp anchor plus device timestamp; USB receipt delay is not treated as acquisition time', 'cloud_coordinate_frame': 'CAM_A optical', 'cloud_length_unit': 'meter', 'cloud_capture_hz': args.cloud_fps, 'cloud_capture_enabled': True, 'cloud_ros_topic': '/oak/depth/points' if args.publish_cloud else None, 'cloud_snapshot_recording': False, 'depth_input_fps': depth_fps, 'depth_sampling_stride': args.depth_stride, 'depth_optical_z_range_m': [args.depth_min_m, args.depth_max_m], 'stereo_images_recorded': False, 'calibration_provenance': provenance, 'duration_limit_s': args.duration, 'map_accumulation': False, 'pose_accuracy_verified': False}
                    atomic_json(output / 'metadata.json', metadata)
                    ready = {'output_dir': str(output), 'config_dir': str(config_dir), 'calib_file': str(config_dir / 'nomad_calib.json'), 'config_file': str(config_dir / 'nomad_vio_config.json'), 'metadata_file': str(output / 'metadata.json'), 'device_id': metadata['device_id'], 'device_host_offset_ns': offset_ns}
                    atomic_json(ready_path, ready)
                    first_data_wall = time.monotonic()
                    stats['state'] = 'streaming'
                    print('READY ' + str(ready_path), flush=True)
                stamp_ns = pair_device_ns + offset_ns
                if vio_maps is not None:
                    from .stereo_rectification import apply
                    lp, rp = apply(lp, rp, vio_maps)
                publish_image(left_pub, lp, stamp_ns, 'oak_left_rectified_optical_frame' if vio_maps is not None else 'oak_left_optical_frame')
                publish_image(right_pub, rp, stamp_ns, 'oak_right_rectified_optical_frame' if vio_maps is not None else 'oak_right_optical_frame')
                stats['stereo_pairs'] += 1
                stats['first_frame_stamp_ns'] = stats['first_frame_stamp_ns'] or stamp_ns
                stats['last_frame_stamp_ns'] = stamp_ns
                stats['max_pair_difference_ms'] = max(stats['max_pair_difference_ms'], abs(lt - rt) / 1000000.0)
                last_pair_wall = time.monotonic()
            while True:
                batch = imu_queue.tryGet()
                if batch is None:
                    break
                for packet in batch.packets:
                    a, g = (packet.acceleroMeter, packet.gyroscope)
                    resampler.add(device_ns(a.getTimestampDevice()), [a.x, a.y, a.z], device_ns(g.getTimestampDevice()), [g.x, g.y, g.z])
            if offset_ns is not None:
                for it, accel, gyro in resampler.take_ready():
                    stamp_ns = it + offset_ns
                    message = Imu()
                    stamp_message(message, stamp_ns, 'oak_imu_frame')
                    message.orientation_covariance[0] = -1.0
                    message.linear_acceleration.x = float(accel[0])
                    message.linear_acceleration.y = float(accel[1])
                    message.linear_acceleration.z = float(accel[2])
                    message.angular_velocity.x = float(gyro[0])
                    message.angular_velocity.y = float(gyro[1])
                    message.angular_velocity.z = float(gyro[2])
                    imu_pub.publish(message)
                    stats['imu_published'] += 1
                    stats['first_imu_stamp_ns'] = stats['first_imu_stamp_ns'] or stamp_ns
                    stats['last_imu_stamp_ns'] = stamp_ns
            if cloud_queue is not None:
                latest = None
                while True:
                    cloud = cloud_queue.tryGet()
                    if cloud is None:
                        break
                    latest = cloud
                if latest is not None and offset_ns is not None:
                    ct = device_ns(latest.getTimestampDevice())
                    if last_cloud_device_ns is None or ct - last_cloud_device_ns >= 950000000.0 / args.cloud_fps:
                        if not latest.isOrganized():
                            raise RuntimeError('Expected an organized physical point cloud')
                        h, w = (latest.getHeight(), latest.getWidth())
                        points = np.asarray(latest.getPoints(), dtype=np.float32)
                        if points.shape != (h * w, 3):
                            raise RuntimeError('Unexpected physical point cloud buffers')
                        if depth_pub is not None:
                            sampled = sample_depth(points.reshape(h, w, 3), args.depth_stride, args.depth_min_m, args.depth_max_m)
                            depth_pub.publish(cloud_message(sampled, ct + offset_ns, DEPTH_FRAME))
                            stats['cloud_published'] += 1
                        last_cloud_device_ns = ct
            if now - last_stats_wall >= 1:
                stats.update({'elapsed_s': now - start_wall, 'stream_elapsed_s': None if first_data_wall is None else now - first_data_wall, 'imu_raw_accel_samples': resampler.accel_samples, 'imu_raw_gyro_samples': resampler.gyro_samples, 'imu_duplicate_accel': resampler.duplicate_accel, 'imu_duplicate_gyro': resampler.duplicate_gyro, 'imu_unbracketed_or_gap_drops': resampler.dropped_accel})
                for prefix, count_key, first_key, last_key in (('stereo', 'stereo_pairs', 'first_frame_stamp_ns', 'last_frame_stamp_ns'), ('imu', 'imu_published', 'first_imu_stamp_ns', 'last_imu_stamp_ns')):
                    first, last = (stats[first_key], stats[last_key])
                    stats[prefix + '_measured_hz'] = (stats[count_key] - 1) * 1000000000.0 / (last - first) if first is not None and last is not None and (last > first) else None
                atomic_json(output / 'bridge_stats.json', stats)
                last_stats_wall = now
            time.sleep(0.001)
        stats['state'] = 'stopped' if stop else 'completed'
    except BaseException as exc:
        final_error = exc
        stats['state'] = 'failed'
        stats['error'] = f'{type(exc).__name__}: {exc}'
        print(stats['error'], file=sys.stderr, flush=True)
    finally:
        stats['elapsed_s'] = time.monotonic() - start_wall
        stats['ended_unix_ns'] = time.time_ns()
        atomic_json(output / 'bridge_stats.json', stats)
        if started:
            try:
                pipeline.stop()
            except Exception as exc:
                print(f'Pipeline stop: {exc}', file=sys.stderr, flush=True)
        if device is not None:
            device.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if final_error is not None else 0
if __name__ == '__main__':
    raise SystemExit(main())

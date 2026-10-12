"""Local ROS-boundary contracts, without Gazebo, DDS delivery, or an executor."""
from pathlib import Path
import sys

import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE))

rclpy = pytest.importorskip("rclpy", reason="Source the ROS 2 Jazzy environment for boundary checks")
from rclpy.time import Time
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nomad_perception.semantic_input import encode_semantic_cloud
from nomad_geometry_mapping.mapper import ManualMapper, stamp_seconds
from nomad_geometry_mapping.archive import load_snapshot


@pytest.fixture
def mapper(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    monkeypatch.setenv("ROS_DOMAIN_ID", "215")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    rclpy.init(args=[
        "--ros-args", "--params-file",
        str(PACKAGE / "config/geometry_mapping.yaml"),
        "-p", "session_directory:=" + str(tmp_path / "session"),
        "-p", "initial_size_m:=4.0", "-p", "max_size_m:=4.0",
    ])
    node = ManualMapper()
    node.get_clock().set_ros_time_override(Time(seconds=1.1))
    try:
        yield node
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def add_camera_transform(mapper):
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "synthetic_optical"
    transform.transform.rotation.w = 1.0
    mapper.buffer.set_transform_static(transform, "boundary_test")


def cloud(stamp=1.0):
    axis = np.linspace(-.25, .25, 21)
    xx, yy = np.meshgrid(axis + .1, axis + .1)
    zz = 1.0 + np.tan(np.deg2rad(12.0)) * (xx - .1)
    xyz = np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())).astype(np.float32)
    header = Header()
    header.frame_id = "synthetic_optical"
    header.stamp = Time(seconds=stamp).to_msg()
    return encode_semantic_cloud(xyz, np.full(len(xyz), 31, np.uint32), header)


def odometry(stamp=1.0, quaternion_w=1.0):
    message = Odometry()
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.pose.pose.orientation.w = quaternion_w
    return message


def test_empty_startup_snapshot_is_finite_and_readable(mapper):
    arrays, metadata = load_snapshot(mapper.directory / "checkpoint.npz")
    assert metadata["version"] == 0
    assert metadata["stamp_s"] == 0.0
    assert arrays["sparse_stamp_s"] == 0.0
    assert np.all(arrays["cost"] == -1)
    assert np.all(np.isnan(arrays["slope_deg"]))


def test_tf_retries_preserve_both_captures_and_exact_lookup_times(mapper, monkeypatch):
    mapper.on_points(cloud(1.0))
    mapper.on_points(cloud(1.05))
    mapper.process_pending(publish=False)
    assert [stamp_seconds(item[0].header.stamp) for item in mapper.pending] == [1.0, 1.05]
    assert mapper.counts["processed_frames"] == 0
    add_camera_transform(mapper)
    captured_stamps = []
    original = mapper.buffer.lookup_transform

    def capture_lookup(target, source, capture_time):
        captured_stamps.append(capture_time.nanoseconds)
        return original(target, source, capture_time)

    monkeypatch.setattr(mapper.buffer, "lookup_transform", capture_lookup)
    mapper.process_pending(publish=False)
    assert captured_stamps == [1_000_000_000, 1_050_000_000]
    assert mapper.counts["processed_frames"] == 2
    assert mapper.counts["out_of_order"] == 0
    assert not mapper.pending
    assert mapper.estimates[(0, 0)].slope_deg == pytest.approx(12.0, abs=1e-5)


def test_shutdown_drains_latest_capture_and_saves_after_ros_context_closes(mapper):
    add_camera_transform(mapper)
    mapper.on_points(cloud(1.0))
    mapper.process_pending(publish=False)
    mapper.on_points(cloud(1.1))
    rclpy.shutdown()
    mapper.finalize()
    # Closed-context publish guards must also remain safe if invoked directly.
    mapper.publish_map()
    mapper.publish_motion()
    mapper.publish_status()
    arrays, metadata = load_snapshot(mapper.directory / "map.npz")
    assert metadata["status"] == "closed"
    assert metadata["outcome"] == "stopped"
    assert metadata["version"] == 2
    assert metadata["last_input_stamp_s"] == pytest.approx(1.1)
    assert metadata["stamp_s"] == pytest.approx(1.1)
    assert float(arrays["sparse_stamp_s"]) == pytest.approx(1.1)
    assert metadata["counters"]["processed_frames"] == 2
    assert metadata["counters"]["shutdown_pending_discarded"] == 0
    assert metadata["pending_frames"] == 0
    assert metadata["cost_model"]["name"] == "angle_power_v1"
    cx, cy = mapper.views.grid.cell(.1, .1)
    assert arrays["cost"][cy, cx] == 36
    index = int(np.flatnonzero(np.all(arrays["sparse_cells"] == [0, 0], axis=1))[0])
    assert arrays["sparse_output_cost"][index] == arrays["cost"][cy, cx]
    assert arrays["sparse_legacy_output_cost"][index] == 62


def test_shutdown_reports_unresolvable_final_capture(mapper):
    mapper.on_points(cloud(1.0))
    rclpy.shutdown()
    mapper.finalize()
    _, metadata = load_snapshot(mapper.directory / "map.npz")
    assert metadata["pending_frames"] == 0
    assert metadata["counters"]["shutdown_pending_discarded"] == 1
    assert metadata["observed_cells"] == 0


def test_odometry_rejects_zero_quaternion_and_stores_unit_orientation(mapper):
    mapper.on_odom(odometry(quaternion_w=0.0))
    assert not mapper.trajectory
    assert mapper.latest_odom is None
    mapper.on_odom(odometry(quaternion_w=2.0))
    assert len(mapper.trajectory) == 1
    assert np.linalg.norm(mapper.trajectory[0][4:]) == pytest.approx(1.0)
    mapper.save()
    arrays, _ = load_snapshot(mapper.directory / "checkpoint.npz")
    assert arrays["trajectory"][0, 7] == 1.0


def test_clock_reset_preserves_completed_map_and_freezes_motion(mapper):
    add_camera_transform(mapper)
    mapper.on_points(cloud(1.0))
    mapper.process_pending(publish=False)
    initial_cost = mapper.cost.copy()
    initial_version = mapper.version
    mapper.get_clock().set_ros_time_override(Time(seconds=.1))
    mapper.process_pending(publish=False)
    assert mapper.fault
    assert mapper.outcome == "clock_reset"
    mapper.on_odom(odometry(stamp=2.0))
    assert not mapper.trajectory
    assert mapper.latest_odom is None
    assert mapper.version == initial_version
    assert np.array_equal(mapper.cost, initial_cost)


def test_neighbor_confidence_roundtrip_uses_effective_cost_and_preserves_raw_support(mapper):
    from dataclasses import replace

    add_camera_transform(mapper)
    mapper.on_points(cloud(1.0))
    mapper.process_pending(publish=False)
    template = mapper.estimates[(0, 0)]
    mapper.estimates = {
        (dx, dy): replace(template, q_g=0.2 if (dx, dy) == (0, 0) else 0.8,
                          slope_deg=10.0, geometry_cost=53)
        for dy in (-1, 0, 1) for dx in (-1, 0, 1)
    }
    mapper.cost, mapper.angles = mapper.cost_policy.layers(mapper.estimates, mapper.views.grid)
    mapper.save()
    arrays, metadata = load_snapshot(mapper.directory / "checkpoint.npz")
    center = int(np.flatnonzero(np.all(arrays['sparse_cells'] == [0, 0], axis=1))[0])
    cx, cy = mapper.views.grid.cell(.1, .1)
    assert arrays['sparse_q_g'][center] == pytest.approx(0.2)
    assert arrays['sparse_effective_q_g'][center] == pytest.approx(0.8)
    assert arrays['sparse_confidence_neighbor_corrected'][center]
    assert np.count_nonzero(arrays['sparse_confidence_neighbor_corrected']) == 1
    assert arrays['sparse_output_cost'][center] == arrays['cost'][cy, cx] == 26
    assert arrays['sparse_slope_deg'][center] == arrays['slope_deg'][cy, cx] == 10
    assert mapper.estimates[(0, 0)].q_g == 0.2
    assert metadata['cost_model']['neighbor_confidence']['target_below'] == 0.3
    assert metadata['cost_model']['neighbor_confidence']['neighbor_min'] == 0.7


def test_quality_refit_is_used_by_online_map_and_archive_after_raw_expiry(mapper):
    add_camera_transform(mapper)
    mapper.on_points(cloud(1.0))
    mapper.process_pending(publish=False)
    x = np.array([.09, .1, .11]); y = np.array([.0999, .1001, .0999])
    z = 1.0 + np.tan(np.deg2rad(12.0)) * (x - .1) + np.tan(np.deg2rad(85.0)) * (y - .1)
    xyz = np.column_stack((x, y, z)).astype(np.float32)
    header = Header(); header.frame_id = 'synthetic_optical'
    header.stamp = Time(seconds=20.0).to_msg()
    mapper.get_clock().set_ros_time_override(Time(seconds=20.0))
    mapper.on_points(encode_semantic_cloud(xyz, np.full(3, 31, np.uint32), header))
    mapper.process_pending(publish=False)
    mapper.save()
    arrays, metadata = load_snapshot(mapper.directory / 'checkpoint.npz')
    index = int(np.flatnonzero(np.all(arrays['sparse_cells'] == [0, 0], axis=1))[0])
    cx, cy = mapper.views.grid.cell(.1, .1)
    assert arrays['sparse_plane_source'][index] == 2
    assert arrays['sparse_plane_raw_slope_deg'][index] > 80
    assert 10 < arrays['sparse_slope_deg'][index] < 13
    assert arrays['cost'][cy, cx] == arrays['sparse_output_cost'][index] < 100
    assert arrays['diagnostic_raw_xyz'].shape[1] == 3
    assert metadata['plane_quality']['enabled']
    assert metadata['candidate'] == 'quality_raw_tls_height_refit_v1'

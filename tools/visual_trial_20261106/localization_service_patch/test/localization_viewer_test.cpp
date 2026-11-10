#include <nomad_localization/localization_viewer.hpp>
#include <nomad_localization/public_output.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace nomad_localization;
using V3 = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
namespace {
int checks = 0;
void check(bool condition, const char* reason) {
  ++checks;
  if (!condition) throw std::runtime_error(reason);
}
Q attitude(double yaw, double pitch = 0., double roll = 0.) {
  return Q(Eigen::AngleAxisd(yaw, V3::UnitZ()) * Eigen::AngleAxisd(pitch, V3::UnitY()) *
           Eigen::AngleAxisd(roll, V3::UnitX()));
}
void frozen_reference_and_new_epoch() {
  FrozenVioFrame frame;
  const ViewerPose raw{V3(100., 30., 5.), attitude(.8, .2)};
  const ViewerPose output{V3(2., 4., .1), attitude(-.6, .2)};
  frame.anchor(raw, output);
  check(frame.initialized() && frame.epoch() == 0, "initial frozen segment");
  check((frame.transform(raw).p - output.p).norm() < 1e-10, "initial raw origin attached to actual output");
  const ViewerPose next = viewerCompose(raw, {V3(1., .2, .1), attitude(.1)});
  const auto expected = viewerCompose(output, {V3(1., .2, .1), attitude(.1)});
  const auto before = frame.transform(next);
  check((before.p - expected.p).norm() < 1e-10, "VIO relative motion in frozen output");
  const ViewerPose corrected_output{V3(10., -20., 3.), attitude(1.9)};
  // Merely receiving a new fused output does not change the reference path.
  check((frame.transform(next).p - before.p).norm() == 0., "GNSS/Ceres cannot move frozen VIO");
  frame.anchor(next, corrected_output);
  check(frame.epoch() == 1, "explicit new segment");
  check((frame.transform(next).p - corrected_output.p).norm() < 1e-10, "new epoch starts at current output");
  const ViewerPose reset_raw{V3::Zero(), attitude(-2.)};
  frame.anchor(reset_raw, corrected_output);
  check(frame.epoch() == 2, "raw VIO restart has own segment");
  check((frame.transform(reset_raw).p - corrected_output.p).norm() < 1e-10, "restart has no artificial position jump");
}
void reset_pivots_at_actual_vehicle_base() {
  PublicOutputConfig mounting;
  mounting.t_base_imu = V3(.35, -.12, .2);
  mounting.q_base_imu = attitude(.4, -.1, .2);
  mounting.base_imu_transform_confirmed = true;
  PublicOutputAdapter adapter(mounting);
  const ViewerPose original{V3(3., -5., .8), attitude(.3, .4, -.2)};
  const auto before = adapter.make(original.p, original.q, true, true);
  const auto reset = yawResetReferencePose(original, -1.2, mounting.t_base_imu, mounting.q_base_imu);
  const auto after = adapter.make(reset.p, reset.q, true, true);
  check((after.p_world_base - before.p_world_base).norm() < 1e-10, "base lever arm position retained");
  check(std::abs(HeadingEstimator::wrap_angle(HeadingEstimator::yaw_of(reset.q) + 1.2)) < 1e-10,
        "requested magnetic reference yaw");
  check((reset.q.conjugate() * V3::UnitZ() - original.q.conjugate() * V3::UnitZ()).norm() < 1e-10,
        "roll/pitch gravity direction unchanged");
  const ViewerPose local{V3(.2, .4, .8), attitude(.7, -.1)};
  const auto map = viewerCompose(reset, viewerInverse(local));
  check((viewerCompose(map, local).p - reset.p).norm() < 1e-10, "map transform realizes pivot");
}
HeadingConfig magnetic_config() {
  HeadingConfig config;
  config.extrinsics_confirmed = config.calibration_confirmed = false;
  config.allow_uncalibrated = true;
  config.extrapolate = false;
  return config;
}
void magnetic_observation_is_independent_of_vio_seed() {
  const auto config = magnetic_config();
  HeadingEstimator ordinary(config), independent(config);
  const Q magnetic_yaw = attitude(.9, .2, -.1);
  const auto acceleration = magnetic_yaw.conjugate() * V3(0., 0., 9.80665);
  const auto field = magnetic_yaw.conjugate() * V3(0., 50e-6, -10e-6);
  check(ordinary.addImu(100., V3::Zero(), acceleration, 100.), "ordinary IMU");
  check(independent.addImu(100., V3::Zero(), acceleration, 100.), "independent IMU");
  check(ordinary.addMag(100., field, 100.), "ordinary magnetic observation");
  check(independent.addMag(100., field, 100.), "independent magnetic observation");
  check(ordinary.seed(100.01, attitude(-.7), false), "live VIO seed accepted");
  const auto magnetic = independent.estimate(100.02);
  check(ordinary.estimate(100.02).anchor_source == HeadingSource::VIO_SEED, "ordinary state overwritten by VIO");
  check(magnetic.anchor_source == HeadingSource::MAGNETOMETER, "reset observation remains magnetic");
  check(std::abs(HeadingEstimator::wrap_angle(HeadingEstimator::yaw_of(magnetic.q_world_reference) - .9)) < 1e-10,
        "independent magnetic yaw preserved");
  check(magnetic.valid && magnetic.using_uncalibrated && !magnetic.global_heading_valid,
        "provisional calibration remains explicit");
  check(!independent.estimate(102.).mag_fresh, "stale magnetic data cannot reset");
  check(!independent.addMag(103., field, 102.), "future magnetic data rejected");
}
void reset_gate_reports_missing_input_without_state_mutation() {
  HeadingResetInputs in;
  in.vio_healthy = in.have_pose = in.frame_valid = true;
  in.vio_only_duration_s = 2.; in.stationary_duration_s = 1.;
  in.magnetic.valid = in.magnetic.gyro_fresh = in.magnetic.mag_fresh = true;
  in.magnetic.anchor_source = HeadingSource::MAGNETOMETER;
  in.magnetic.using_uncalibrated = true;
  check(headingResetBlockReason(in).empty(), "usable provisional magnetic reset");
  in.gnss_usable = true;
  check(headingResetBlockReason(in) == "VIO_ONLY_REQUIRED", "fused state rejected");
  in.gnss_usable = false; in.stationary_duration_s = .1;
  check(headingResetBlockReason(in) == "HOLD_VEHICLE_STILL", "motion/stationary hold required");
  in.stationary_duration_s = 1.; in.vio_only_duration_s = .5;
  check(headingResetBlockReason(in) == "WAITING_FOR_CONTINUOUS_VIO_ONLY", "mode continuity required");
  in.vio_only_duration_s = 2.; in.magnetic.mag_fresh = false;
  check(headingResetBlockReason(in) == "MAGNETOMETER_NOT_FRESH", "stale mag rejected");
  in.magnetic.mag_fresh = true; in.magnetic.gyro_fresh = false;
  check(headingResetBlockReason(in) == "MAG_IMU_NOT_FRESH", "stale tilt/gyro rejected");
  in.magnetic.gyro_fresh = true; in.magnetic.anchor_source = HeadingSource::VIO_SEED;
  check(headingResetBlockReason(in) == "MAGNETIC_HEADING_UNAVAILABLE", "VIO seed never masquerades as magnetic");
  in.vio_healthy = false;
  check(headingResetBlockReason(in) == "VIO_INPUT_NOT_READY", "expired VIO rejected");
}
}  // namespace
int main() {
  try {
    frozen_reference_and_new_epoch();
    reset_pivots_at_actual_vehicle_base();
    magnetic_observation_is_independent_of_vio_seed();
    reset_gate_reports_missing_input_without_state_mutation();
    std::cout << "localization_viewer: " << checks << " checks passed; synthetic_only=true\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "localization_viewer failed: " << error.what() << '\n';
    return 1;
  }
}

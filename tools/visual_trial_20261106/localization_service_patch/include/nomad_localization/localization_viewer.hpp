#pragma once

#include <nomad_localization/heading_estimator.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <string>

namespace nomad_localization {

struct ViewerPose {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

inline ViewerPose viewerCompose(const ViewerPose& a, const ViewerPose& b) {
  return {a.p + a.q * b.p, (a.q * b.q).normalized()};
}
inline ViewerPose viewerInverse(const ViewerPose& a) {
  const auto q = a.q.conjugate();
  return {-(q * a.p), q};
}

// A reference trajectory, never another estimator. Its first raw VIO pose is
// rigidly attached to the then-current local output. Subsequent optimizer map
// corrections cannot change this frozen transform. A new raw epoch/manual
// heading initialization creates an explicit segment instead of drawing a
// fictitious line between coordinate conventions.
class FrozenVioFrame {
 public:
  void anchor(const ViewerPose& raw, const ViewerPose& output_reference) {
    output_from_raw_ = viewerCompose(output_reference, viewerInverse(raw));
    if (initialized_) ++epoch_;
    initialized_ = true;
  }
  ViewerPose transform(const ViewerPose& raw) const {
    return viewerCompose(output_from_raw_, raw);
  }
  bool initialized() const { return initialized_; }
  std::uint64_t epoch() const { return epoch_; }
 private:
  ViewerPose output_from_raw_;
  bool initialized_ = false;
  std::uint64_t epoch_ = 0;
};

struct HeadingResetInputs {
  bool vio_healthy = false;
  bool gnss_usable = false;
  bool have_pose = false;
  bool frame_valid = false;
  double vio_only_duration_s = -1.;
  double stationary_duration_s = -1.;
  double required_vio_only_s = 1.;
  double required_stationary_s = .5;
  HeadingEstimate magnetic;
};

// Deliberately reports each failed precondition to the GUI. Calibration and
// freshness are the same policies as the live helper, never silently bypassed
// by clicking a button. Provisional magnetic direction stays provisional.
inline std::string headingResetBlockReason(const HeadingResetInputs& in) {
  if (!in.vio_healthy || !in.have_pose) return "VIO_INPUT_NOT_READY";
  if (in.gnss_usable) return "VIO_ONLY_REQUIRED";
  if (!in.frame_valid) return "OUTPUT_FRAME_INVALID";
  if (!std::isfinite(in.vio_only_duration_s) || in.vio_only_duration_s < in.required_vio_only_s)
    return "WAITING_FOR_CONTINUOUS_VIO_ONLY";
  if (!std::isfinite(in.stationary_duration_s) || in.stationary_duration_s < in.required_stationary_s)
    return "HOLD_VEHICLE_STILL";
  if (!in.magnetic.gyro_fresh) return "MAG_IMU_NOT_FRESH";
  if (!in.magnetic.mag_fresh) return "MAGNETOMETER_NOT_FRESH";
  if (!in.magnetic.valid || in.magnetic.anchor_source != HeadingSource::MAGNETOMETER)
    return in.magnetic.last_rejection.empty() ? "MAGNETIC_HEADING_UNAVAILABLE" : in.magnetic.last_rejection;
  if (!in.magnetic.global_heading_valid && !in.magnetic.using_uncalibrated)
    return "MAG_CALIBRATION_OR_EXTRINSICS_UNCONFIRMED";
  return "";
}

// Left-multiplying yaw changes heading while keeping gravity/tilt unchanged.
// The public vehicle base is the pivot; its IMU may have a mounting lever arm.
inline ViewerPose yawResetReferencePose(const ViewerPose& global_reference,
                                        double desired_yaw,
                                        const Eigen::Vector3d& t_base_imu,
                                        const Eigen::Quaterniond& q_base_imu) {
  const auto q_world_base = (global_reference.q * q_base_imu.conjugate()).normalized();
  const Eigen::Vector3d p_world_base = global_reference.p - q_world_base * t_base_imu;
  const double old_yaw = HeadingEstimator::yaw_of(global_reference.q);
  const Eigen::Quaterniond correction(Eigen::AngleAxisd(
      HeadingEstimator::wrap_angle(desired_yaw - old_yaw), Eigen::Vector3d::UnitZ()));
  const auto new_q = (correction * global_reference.q).normalized();
  const auto new_q_base = (new_q * q_base_imu.conjugate()).normalized();
  return {p_world_base + new_q_base * t_base_imu, new_q};
}

}  // namespace nomad_localization

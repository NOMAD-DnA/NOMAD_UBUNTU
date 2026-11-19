#pragma once
#include <nomad_costmap/core.hpp>

namespace nomad {
// Mapper-local camera/IMU alignment. Both poses below are IMU <- rectified CAM_B.
// Captured 2026-11-19 from Jetson factory-derived calibration and Pi5 VIO
// calibration respectively. This is not a rotation applied to public VIO/TF.
// Keep the camera centre fixed, matching the unchanged VIO extrinsic translation.
inline M4 mapper_vio_extrinsic_correction() {
  M4 reference = M4::Identity(), calibrated = M4::Identity();
  reference.block<3,3>(0,0) = Eigen::Quaterniond(0.012215014987780827, 0.999922769439659, 0.0003027675657712804, -0.002270880175625642).normalized().toRotationMatrix();
  reference.block<3,1>(0,3) = V3(-0.005870614692479775, 0.006331938845222788, -0.00047068094732958496);
  calibrated.block<3,3>(0,0) = Eigen::Quaterniond(-0.009513829865992664, 0.9999521180918164, 0.0002533552252682195, -0.002276922506534182).normalized().toRotationMatrix();
  calibrated.block<3,1>(0,3) = V3(-0.0058706146924797755, 0.006331938845222788, -0.00047068094732958496);
  return calibrated * reference.inverse();
}

inline std::array<M4,2> mapper_vio_sensor_mounts(const std::array<M4,2>& factory) {
  const M4 correction = mapper_vio_extrinsic_correction();
  // CAM_B and CAM_A must share one left correction. Derived LiDAR/ZED mounts
  // then stay consistent with the measured camera-to-camera/sensor transforms.
  return {correction * factory[0], correction * factory[1]};
}
}

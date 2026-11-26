#pragma once
#include "nomad_costmap/detail.hpp"
#include "nomad_costmap/moving_front_free_region.hpp"

namespace nomad {
inline constexpr const char* indoor_build_id = "nomad-indoor-mapper-20261124-obstacle-lock-1";
struct IndoorOptions {
  Config fusion;
  double floor_z = .05, fixed_z = 0., stationary_hold = 2.;
  int64_t pose_gap_ns = 200000000;
  size_t pending_limit = 80;
  std::string imu_frame = "oak_imu_frame"; // Existing VIO frame name, not a camera input.
  M4 imu_lidar = M4::Identity();
  // IMU <- rectified CAM_B from the existing VIO extrinsic calibration.
  V3 camera_forward_axis_imu = Eigen::Quaterniond(-0.009513829865992664,
    0.9999521180918164,0.0002533552252682195,-0.002276922506534182).normalized()*V3::UnitZ();
  double half_fov_deg = 80.;
  bool lock_obstacle_cells = false;
  static constexpr double ray_range_m = 2.;
  static IndoorOptions load(const Json&);
};

// Clamp the public base pose BEFORE composing the static IMU lever arm.
// This retains XY, all three rotation components and 3D sensor geometry.
Pose indoor_imu_pose(Pose base, const M4& base_imu, double fixed_z);
ScanPair indoor_scan_pair(const ScanInput&, const CloudInput&);
bool ray_crosses_cell(const V3& origin, const V3& hit, Key, double resolution);
std::vector<Key> trace_ray_cells(const V3& origin,const V3& hit,const V3& vehicle,double resolution,double radius);

struct IndoorCell {
  bool obstacle = false;
  int64_t last_point = 0, last_obstacle = 0, cleared = 0, ray_observed = 0;
  std::string source;
};

class IndoorMap {
public:
  IndoorOptions options;
  StartupFreeRegion startup;
  MovingFrontFreeRegion moving;
  std::map<Key, IndoorCell> cells;
  int64_t last_scan = 0, capture = 0;
  uint64_t rays_cleared = 0, frames = 0, rays_used = 0, rays_outside_fov = 0;
  M4 base_imu = M4::Identity();
  bool reference_set = false;
  explicit IndoorMap(const IndoorOptions& o) : options(o), startup(o.fusion), moving(o.fusion) {}
  void initialize(const Pose& imu, const M4& base_imu);
  void ingest(const Frame&);
  void prune(const MapWindow&);
  CostSnapshot snapshot(const Pose& imu, const M4& base_imu);
  Json provenance() const;
};
}

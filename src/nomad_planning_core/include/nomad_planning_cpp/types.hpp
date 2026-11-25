#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace nomad_planning_cpp {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
struct Pose { double x=0, y=0, yaw=0; };
using Path = std::vector<Pose>;
struct Cell { int x=0,y=0; bool operator==(const Cell& b) const {return x==b.x&&y==b.y;} };
struct Grid {
  double resolution=0.12;
  int width=0,height=0;
  double origin_x=0,origin_y=0,origin_yaw=0;
  std::string frame="odom";
  std::int64_t stamp_ns=0;
  std::vector<std::int8_t> data;
};
struct Layers { Grid grid; std::vector<double> slope_cost,terrain_cost; };
struct VehicleConfig {
  double wheelbase=.72, track=.60, wheel_radius=.15, joint_limit=.55;
  double max_steer=.35,reference_offset=-.36;
  double body_length=1.0,body_width=.65;
  bool rear_steer=true;
  // Preview dimensions are not commissioning values; ROS output must gate this.
  bool geometry_confirmed=false;
};
struct CostConfig {
  bool multi=true;
  double obstacle_weight=4,slope_weight=1,terrain_weight=1;
  double radius=2,missing_cost=1,unknown_surcharge=4.5,unknown_penalty=4.5;
  double cluster_weight=0,cluster_radius=4,cluster_influence=0,cluster_saturation=1;
  // Soft centre-to-blocked-cell-edge preference, independent of body geometry.
  // Applies to unknown transit too; never changes occupancy/permission.
  double clearance_weight=0,clearance_radius=.45;
  // 2026-11-25: relative clearance cost. A cell nearer than clearance_radius costs
  // base*(1+clearance_factor*(1-d/clearance_radius)^2), so the preference scales with the
  // proximity/cluster/terrain base instead of being diluted by it. 0 = previous behaviour.
  double clearance_factor=0;
};
struct SearchConfig {
  double seconds=0;
  int max_expansions=50000;
  double goal_tolerance=.29;
  // Opt-in short local waypoints: keep legacy primitive/parity by default.
  double primitive_length=0;
  bool require_known_endpoint=false;
};
struct PlanResult {
  Path path;
  std::string detail;
  int expansions=0;
  double cost=kInf;
};
struct Motion { double speed=0,steering=0; };
}  // namespace nomad_planning_cpp

#pragma once
// Minimal compatibility types extracted from the improved mapper. No camera or plane-fitting API.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>
namespace nomad {
  class MovingFrontFreeRegion;
  using Json=nlohmann::json;
  using V3=Eigen::Vector3d;
  using M3=Eigen::Matrix3d;
  using M4=Eigen::Matrix4d;
  using Points=std::vector<V3>;
  using Key=std::pair<int64_t,int64_t>;
  constexpr double nan=std::numeric_limits<double>::quiet_NaN();
  struct Pose {
    int64_t stamp = 0;
    V3 p = V3::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  };
  struct Frame {
    int64_t stamp = 0;
    int source = 0;
    Points points;
    std::vector<int64_t> stamps;
    Pose pose;
    // Real per-return LiDAR origins in odom, before local semantic sampling.
    Points ray_origins;
    std::vector<Pose> capture_poses; // Actual world<-IMU pose at each return acquisition time.
  };
  struct Config {
    Json semantic;
    double resolution=.125, map_length=10., map_keep_radius=6.;
    bool cost_known_cells=true, startup_free_region_enabled=true;
    size_t max_map_cells=0;
    V3 slope_forward_axis_base_link=V3::UnitX();
    double slope_score_per_deg=8., slope_block_deg=10.;
    // Compatibility fields referenced by the unchanged legacy cost function.
    // The selected class10 is protected from moving-zero overrides. No person input exists.
    bool person_ray_restore=false, startup_person_cost_overlay=false;
    bool retain_last_observed_costs = true; // Indoor ray map: no age-based local unknown conversion.
    int person_class_id=10;
  };
  // Vehicle-centred square map window in cell keys (half-open [min, min+size)).
  struct MapWindow {
    bool valid = false;
    int64_t xmin = 0, ymin = 0, width = 0, height = 0;
    bool contains(Key k) const {
      return !valid || (k.first >= xmin && k.first < xmin + width && k.second >= ymin && k.second < ymin + height);
    }
    // Cell cap (2026-11-14). Applied by each store's prune() with its own ages.
    size_t max_cells = 0;
    Key centre{0, 0};
    int64_t keep_cells = 0;
    bool active() const { return valid || max_cells > 0; }
    bool protects(Key k) const {
      const int64_t dx = k.first - centre.first, dy = k.second - centre.second;
      return dx * dx + dy * dy <= keep_cells * keep_cells;
    }
  };
  // Keys to drop so that at most w.max_cells of `keys` remain: oldest age first,
  // never a key inside the keep radius. Empty when the cap is off or not reached.
  template <class Keys, class AgeFn>
  std::set<Key> cells_over_cap(const Keys& keys, const MapWindow& w, AgeFn age)
  {
    std::set<Key> out;
    if (!w.max_cells || keys.size() <= w.max_cells) return out;
    std::vector<std::pair<int64_t, Key> > candidates;
    candidates.reserve(keys.size());
    for (const Key& k:keys)
      if (!w.protects(k)) candidates.emplace_back(age(k), k);
    const size_t excess = std::min(keys.size() - w.max_cells, candidates.size());
    std::nth_element(candidates.begin(), candidates.begin() + excess, candidates.end());
    for (size_t i = 0; i < excess; ++i) out.insert(candidates[i].second);
    return out;
  }
  MapWindow make_window(const Config&, const Eigen::Vector2d& center_xy);
  // Former Ground::snapshot() JSON row, same values and float rounding.
  struct SlopeRow {
    int64_t ix = 0, iy = 0;
    double angle = nan, height = nan, confidence = nan;
    bool reused = false;
    double calculated_s = 0.;
    int64_t observed = 0;
    double tilt = nan;
  };
  struct SlopeSnapshot {
    std::vector<SlopeRow> rows;          // sorted by (iy, ix)
    std::vector<Key> startup_frozen_cells;
    int64_t startup_frozen_stamp = 0;
    int64_t capture = 0;
    std::array<int64_t, 2> source_capture{0, 0};
    Json counters = Json::object();      // diagnostics only
  };
  // Former SemanticGrid::snapshot() JSON row (fields read by build_cost/publish).
  struct SemanticRow {
    int64_t ix = 0, iy = 0;
    int class_id = -1;                   // -1 = null
    int cost = -1;                       // -1 = null
    int64_t last_geometry_ns = 0, last_semantic_ns = 0, camera_observed_ns = 0;
    std::vector<int64_t> camera_capture_ns;
    // Present only with person_ray_restore.
    bool has_person = false, person_unresolved = false;
    int64_t base_last_semantic_ns = 0, person_last_geometry_ns = 0, person_clear_observed_ns = 0;
    std::string person_clear_source;
  };
  struct SemanticSnapshot {
    std::vector<SemanticRow> rows;       // sorted by key
    int64_t stamp = 0;
  };
  // Former build_cost() JSON result, row-major (iy, ix) arrays.
  struct CostSnapshot {
    int64_t capture = 0;
    double resolution = 0., origin_x = 0., origin_y = 0.;
    int64_t width = 0, height = 0;
    std::vector<float> slope_deg, slope_magnitude_deg, slope_score, semantic_score, slope_cost, terrain_cost;
    std::vector<int8_t> global, local, support;
  };
  // A user-declared startup prior, never camera or slope measurement evidence.
  // Its cells remain fixed in odom and are permanently retired on observation.
  class StartupFreeRegion {
    double resolution;
    bool enabled, seeded = false;
    Pose anchor;
    std::set<Key> remaining, slope_remaining, semantic_remaining, original;
    size_t original_count = 0;
  public:
    double radius_m = 1.5;
    static constexpr double field_of_view_deg = 360.;
    explicit StartupFreeRegion(const Config&);
    bool initialize_from_imu(const Pose& world_imu, const M4& base_from_imu);
    bool initialized() const { return seeded; }
    const std::set<Key>& cells() const { return remaining; }
    const std::set<Key>& original_cells() const { return original; }
    const std::set<Key>& slope_cells() const { return slope_remaining; }
    const std::set<Key>& semantic_cells() const { return semantic_remaining; }
    void retire_slope(Key key) { slope_remaining.erase(key); if (!semantic_remaining.count(key)) remaining.erase(key); }
    void retire_semantic(Key key) { semantic_remaining.erase(key); if (!slope_remaining.count(key)) remaining.erase(key); }
    Json metadata() const;
  };
  M4 rigid(const Json&);
  Json matrix_json(const M4&);
  Json points_json(const Points&);
  Points points_from_json(const Json&);
  Pose pose_from_json(const Json&);
  Json pose_json(const Pose&);
  Pose interpolate(const std::vector<Pose>&,int64_t,int64_t max_gap=200000000);
  Points transform(const Points&,const M4&);
  double median(std::vector<double>);
  int64_t nearest_even(double);
  Pose checked_pose(const Pose&);
  CostSnapshot build_cost(const SlopeSnapshot&,const SemanticSnapshot&,const Config&,StartupFreeRegion* startup=nullptr,
    const MovingFrontFreeRegion* moving=nullptr,const MapWindow* window=nullptr);
  struct ScanInput {
    int64_t stamp_ns = 0;
    std::vector<double> ranges;          // NaN = invalid (former JSON null)
    double angle_min = 0., angle_increment = 0., range_min = 0., range_max = 0.;
  };
  struct CloudInput {
    int64_t stamp_ns = 0, received_ros_ns = 0;
    Points points;
    std::vector<double> offsets;
  };
  struct ScanPair {
    int64_t stamp_ns = 0;
    Points points_lidar_m;
    std::vector<double> point_time_offsets_s;
    std::vector<int64_t> bin_indices;
  };
  Frame deskew(const ScanPair&,const std::vector<Pose>&,const M4&,double offset_s=0.,int64_t max_gap_ns=200000000);
}

#pragma once
#include "nomad_costmap/core.hpp"
#include <cmath>
#include <stdexcept>
namespace nomad {
// A moving user assumption. Never insert it into measured terrain or TLS state.
class MovingFrontFreeRegion {
  bool enabled_ = false, initialized_ = false, keep_measured_slope_ = false;
  double resolution_, radius_ = .5, rear_length_ = 0., rear_half_width_ = 0.;
  V3 forward_axis_;
  Pose pose_;
  Eigen::Vector2d heading_ = Eigen::Vector2d::Zero();
  std::set<Key> cells_;
  // Cells the vehicle body has actually covered (odom). Permanent: driven = drivable.
  std::set<Key> driven_;
  bool driven_enabled_ = false;
  double driven_front_ = .025;
  size_t driven_limit_ = 400000;
  uint64_t version_ = 0;
public:
  explicit MovingFrontFreeRegion(const Config& cfg)
    : resolution_(cfg.resolution), forward_axis_(cfg.slope_forward_axis_base_link) {
    auto options = cfg.semantic.value("moving_front_free_region", Json::object());
    enabled_ = options.value("enabled", false);
    radius_ = options.value("radius_m", .5);
    if (!std::isfinite(radius_) || radius_ <= 0. || radius_ > 5.)
      throw std::runtime_error("Moving front free radius must be in (0,5] m");
    // 2026-11-11 user request: keep measured slope; also keep the cells under the
    // vehicle body (rear band) free so the footprint does not stand on cells
    // that just left the semicircle while the map/pose are still in transit.
    keep_measured_slope_ = options.value("override_policy", std::string("all_costs_zero_except_person")) ==
      "semantic_zero_measured_slope_kept_except_person";
    rear_length_ = options.value("rear_length_m", 0.);
    rear_half_width_ = options.value("rear_half_width_m", 0.);
    if (!std::isfinite(rear_length_) || rear_length_ < 0. || rear_length_ > radius_ ||
        !std::isfinite(rear_half_width_) || rear_half_width_ < 0. || rear_half_width_ > radius_)
      throw std::runtime_error("Moving front free rear band must be within [0,radius] m");
    // 2026-11-11 user request: the path the vehicle drove over stays drivable
    // (all costs 0 except person) and is never restored to late/false labels.
    driven_enabled_ = options.value("driven_path_memory", false);
    driven_front_ = options.value("driven_front_m", .025);
    if (driven_enabled_ && (!std::isfinite(driven_front_) || driven_front_ < 0. || driven_front_ > radius_ ||
                            rear_length_ <= 0. || rear_half_width_ <= 0.))
      throw std::runtime_error("Driven path memory needs a rear band and driven_front_m in [0,radius]");
  }
  bool keep_measured_slope() const { return keep_measured_slope_; }
  bool driven_enabled() const { return enabled_ && driven_enabled_; }
  const std::set<Key>& driven() const { return driven_; }
  // 2026-11-13: forget driven cells that left the vehicle-centred map window.
  void prune(const MapWindow& window) {
    if (!window.valid) return;
    for (auto it = driven_.begin(); it != driven_.end();)
      it = window.contains(*it) ? std::next(it) : driven_.erase(it);
  }
  bool enabled() const { return enabled_; }
  bool initialized() const { return initialized_; }
  uint64_t version() const { return version_; }
  const std::set<Key>& cells() const { return cells_; }
  bool update_from_imu(const Pose& world_imu, const M4& base_from_imu) {
    if (!enabled_) return true;
    auto invalid = [&] { if (initialized_ || !cells_.empty()) ++version_;
      cells_.clear(); initialized_ = false; return false; };
    if (world_imu.stamp <= 0 || !world_imu.p.allFinite() ||
        !world_imu.q.coeffs().allFinite() || std::abs(world_imu.q.norm()-1.) > 1e-5)
      return invalid();
    try { rigid(matrix_json(base_from_imu)); } catch (...) { return invalid(); }
    const M3 rotation = world_imu.q.toRotationMatrix()*base_from_imu.block<3,3>(0,0).transpose();
    const V3 origin = world_imu.p-rotation*base_from_imu.block<3,1>(0,3);
    Eigen::Vector2d heading = (rotation*forward_axis_).head<2>();
    if (!origin.allFinite() || !heading.allFinite() || heading.norm() < 1e-8 ||
        !std::isfinite(resolution_) || resolution_ <= 0. ||
        (origin.head<2>().cwiseAbs().array()+radius_).maxCoeff()/resolution_ > double(INT64_MAX)/2. ||
        std::pow(2.*std::ceil(radius_/resolution_)+3.,2) > 4000000.) return invalid();
    heading.normalize();
    std::set<Key> next;
    size_t driven_added = 0;
    const int64_t xmin = int64_t(std::floor((origin.x()-radius_)/resolution_));
    const int64_t xmax = int64_t(std::floor((origin.x()+radius_)/resolution_));
    const int64_t ymin = int64_t(std::floor((origin.y()-radius_)/resolution_));
    const int64_t ymax = int64_t(std::floor((origin.y()+radius_)/resolution_));
    for (int64_t y=ymin;y<=ymax;++y) for (int64_t x=xmin;x<=xmax;++x) {
      const Eigen::Vector2d delta((x+.5)*resolution_-origin.x(), (y+.5)*resolution_-origin.y());
      const double along = delta.dot(heading);
      const double lateral = std::abs(heading.x()*delta.y()-heading.y()*delta.x());
      const bool front = delta.squaredNorm() <= radius_*radius_+1e-12 && along >= -1e-12;
      const bool rear = along < 0. && along >= -rear_length_-1e-12 && lateral <= rear_half_width_+1e-12;
      if (front || rear)
        next.insert({x,y});
      if (driven_enabled_ && driven_.size() < driven_limit_ && along >= -rear_length_-1e-12 &&
          along <= driven_front_+1e-12 && lateral <= rear_half_width_+1e-12 && driven_.insert({x,y}).second)
        ++driven_added;
    }
    if (!initialized_ || next != cells_ || driven_added) ++version_;
    cells_ = std::move(next); heading_ = heading;
    pose_ = Pose{world_imu.stamp,origin,Eigen::Quaterniond(rotation).normalized()};
    initialized_ = true;
    return true;
  }
  Json metadata() const {
    Json keys = Json::array();
    for (const auto& key:cells_) keys.push_back({key.first,key.second});
    Json r = {{"enabled",enabled_},{"initialized",initialized_},{"frame","odom"},
      {"radius_m",radius_},{"field_of_view_deg",180.},{"cell_count",cells_.size()},
      {"center_reference","current_public_base_link"},
      {"heading_reference","Pi4_slope_forward_axis_base_link_projected_to_odom_XY"},
      {"rasterization","cell_center_inside_forward_semicircle_or_rear_band"},
      {"rear_length_m",rear_length_},{"rear_half_width_m",rear_half_width_},
      {"measured_slope_kept",keep_measured_slope_},
      {"driven_path_memory",driven_enabled_},{"driven_cell_count",driven_.size()},
      {"driven_policy","all_costs_zero_except_person_permanent"},
      {"source","user_declared_moving_free_cost_override"},
      {"overrides_nonperson_obstacles",true},{"person_class_excluded",true},{"measurement_evidence",false},
      {"applied_cost_layers",keep_measured_slope_?Json{"semantic","global","local"}:Json{"slope","semantic","global","local"}},
      {"version",version_},{"cells",keys}};
    if (initialized_) {
      r["pose"] = pose_json(pose_);
      r["heading_xy"] = {heading_.x(),heading_.y()};
    }
    return r;
  }
};
}

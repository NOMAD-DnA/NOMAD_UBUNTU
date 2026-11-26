#include "nomad_costmap/detail.hpp"
#include <cmath>
namespace nomad {
  M4 rigid(const Json& j)
  {
    require(j.is_array()&&j.size() == 4, "Expected 4x4 transform");
    M4 m;
    for (int r = 0; r < 4; ++r)
    {
      require(j[r].size() == 4, "Expected 4x4 transform");
      for (int c = 0; c < 4; ++c)
        m(r, c) = j[r][c].get<double>();
    }
    require(m.allFinite(), "Nonfinite transform");
    require((m.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).cwiseAbs().maxCoeff() <= 1e-8, "Bad homogeneous row");
    auto R = m.block<3, 3>(0, 0);
    require((R.transpose() * R - M3::Identity()).cwiseAbs().maxCoeff() <= 1.1e-5&&std::abs(R.determinant() - 1) <= 1.1e-5, "Invalid rigid rotation");
    return m;
  }
  Json matrix_json(const M4& m)
  {
    Json j = Json::array();
    for (int r = 0; r < 4; ++r)
      j.push_back({m(r, 0), m(r, 1), m(r, 2), m(r, 3)});
    return j;
  }
  Json points_json(const Points& p)
  {
    Json a = Json::array();
    for (auto& v:p)
      a.push_back({num(v.x()), num(v.y()), num(v.z())});
    return a;
  }
  Points points_from_json(const Json& j)
  {
    require(j.is_array(), "Expected XYZ array");
    Points p;
    p.reserve(j.size());
    for (auto& a:j)
    {
      require(a.size() == 3, "Expected XYZ triples");
      V3 v(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
      require(v.allFinite(), "Nonfinite XYZ");
      p.push_back(v);
    }
    return p;
  }
  Pose pose_from_json(const Json& j)
  {
    require(j.size() >= 8, "Expected integer-ns pose row");
    Pose p;
    p.stamp = j[0].get<int64_t>();
    p.p = V3(j[1].get<double>(), j[2].get<double>(), j[3].get<double>());
    p.q = Eigen::Quaterniond(j[7].get<double>(), j[4].get<double>(), j[5].get<double>(), j[6].get<double>());
    require(p.stamp > 0&&p.p.allFinite()&&p.q.coeffs().allFinite()&&p.q.norm() > .99&&p.q.norm() < 1.01, "Invalid pose");
    p.q.normalize();
    return p;
  }
  Json pose_json(const Pose& p)
  {
    return {p.stamp, p.p.x(), p.p.y(), p.p.z(), p.q.x(), p.q.y(), p.q.z(), p.q.w()};
  }
  Pose interpolate(const std::vector<Pose>& rows, int64_t stamp, int64_t gap)
  {
    require(!rows.empty(), "no_vio_pose");
    auto it = std::lower_bound(rows.begin(), rows.end(), stamp, [](const Pose& p, int64_t t){
      return p.stamp < t;
    });
    require(it != rows.end(), "pose_outside_history");
    if (it->stamp == stamp)
      return *it;
    require(it != rows.begin(), "pose_outside_history");
    auto& b = *it;
    auto& a = *(it - 1);
    require(b.stamp > a.stamp&&b.stamp - a.stamp <= gap, "pose_gap");
    double u = double(stamp - a.stamp) / double(b.stamp - a.stamp);
    Pose p;
    p.stamp = stamp;
    p.p = (1 - u) * a.p + u * b.p;
    p.q = a.q.slerp(u, b.q).normalized();
    return p;
  }
  Points transform(const Points& p, const M4& m)
  {
    Points result;
    result.reserve(p.size());
    for (auto& v:p)
      result.push_back(m.block<3, 3>(0, 0) * v + m.block<3, 1>(0, 3));
    return result;
  }
  double median(std::vector<double> v)
  {
    if (v.empty())
      return nan;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2?v[n / 2]:(v[n / 2 - 1] + v[n / 2]) * .5;
  }
  int64_t nearest_even(double x)
  {
    require(std::isfinite(x)&&x > double(INT64_MIN) + 2048&&x < double(INT64_MAX) - 2048, "Invalid integer rounding");
    return int64_t(std::nearbyint(x));
  }
  StartupFreeRegion::StartupFreeRegion(const Config& cfg)
    : resolution(cfg.resolution), enabled(cfg.startup_free_region_enabled) {
    if (cfg.semantic.contains("startup_free_region"))
      radius_m = cfg.semantic.at("startup_free_region").value("radius_m",1.5);
    require(std::isfinite(radius_m) && radius_m > 0.,
            "Startup free region radius must be finite and positive");
  }

  bool StartupFreeRegion::initialize_from_imu(const Pose& world_imu, const M4& base_from_imu)
  {
    if (seeded) return true;
    if (!enabled || world_imu.stamp <= 0 || !world_imu.p.allFinite() ||
        !world_imu.q.coeffs().allFinite() || std::abs(world_imu.q.norm()-1.) > 1e-5 ||
        !std::isfinite(resolution) || resolution <= 0.) return false;
    try { rigid(matrix_json(base_from_imu)); }
    catch (const std::exception&) { return false; }
    // W_T_I = W_T_B * B_T_I; recover the actual body origin and +X heading.
    const M3 body_rotation = world_imu.q.toRotationMatrix() *
                            base_from_imu.block<3,3>(0,0).transpose();
    const V3 body_position = world_imu.p -
                            body_rotation * base_from_imu.block<3,1>(0,3);
    if (!body_position.allFinite())
      return false;
    const double diameter_cells = 2.*std::ceil(radius_m/resolution)+3.;
    if (!std::isfinite(diameter_cells) || diameter_cells*diameter_cells > 4000000. ||
        (body_position.head<2>().cwiseAbs().array()+radius_m).maxCoeff()/resolution >
          double(INT64_MAX)/2.) return false;
    anchor = Pose{world_imu.stamp, body_position, Eigen::Quaterniond(body_rotation).normalized()};
    const Eigen::Vector2d origin = body_position.head<2>();
    const int64_t xmin = int64_t(std::floor((origin.x()-radius_m)/resolution));
    const int64_t xmax = int64_t(std::floor((origin.x()+radius_m)/resolution));
    const int64_t ymin = int64_t(std::floor((origin.y()-radius_m)/resolution));
    const int64_t ymax = int64_t(std::floor((origin.y()+radius_m)/resolution));
    for (int64_t y = ymin; y <= ymax; ++y)
      for (int64_t x = xmin; x <= xmax; ++x) {
        const Eigen::Vector2d delta = center({x,y},resolution)-origin;
        if (delta.squaredNorm() <= radius_m*radius_m+1e-12)
          remaining.insert({x,y});
      }
    original = remaining;
    slope_remaining = semantic_remaining = remaining;
    original_count = remaining.size();
    seeded = true;
    return true;
  }

  Json StartupFreeRegion::metadata() const
  {
    Json result = {{"enabled",enabled}, {"initialized",seeded},
      {"source","user_declared_startup_prior"}, {"frame","odom"},
      {"radius_m",radius_m}, {"field_of_view_deg",field_of_view_deg},
      {"rasterization","cell_center_inside_vehicle_centered_circle"},
      {"applied_layers",{"slope","semantic","fused"}},
      {"initial_cells",original_count}, {"remaining_cells",remaining.size()},
      {"remaining_slope_cells",slope_remaining.size()}, {"remaining_semantic_cells",semantic_remaining.size()}};
    if (seeded) result["anchor_pose"] = pose_json(anchor);
    return result;
  }


  Frame deskew(const ScanPair& pair, const std::vector<Pose>& poses, const M4& il, double offset, int64_t max_gap_ns)
  {
    Frame f;
    f.stamp = pair.stamp_ns;
    f.source = 0;
    const auto& p = pair.points_lidar_m;
    const auto& times = pair.point_time_offsets_s;
    require(p.size() == times.size(), "LiDAR offsets size mismatch");
    for (size_t i = 0; i < p.size(); ++i)
    {
      int64_t ns = f.stamp + nearest_even((times[i] + offset) * 1e9);
      auto pose = interpolate(poses, ns, max_gap_ns);
      f.capture_poses.push_back(pose);
      V3 q = il.block<3, 3>(0, 0) * p[i] + il.block<3, 1>(0, 3);
      f.points.push_back(pose.q * q + pose.p);
      f.stamps.push_back(ns);
      f.ray_origins.push_back(pose.q * il.block<3,1>(0,3) + pose.p);
    }
    if (!poses.empty())
      f.pose = poses.back();
    return f;
  }
  Pose checked_pose(const Pose& in)
  {
    Pose p = in;
    require(p.stamp > 0&&p.p.allFinite()&&p.q.coeffs().allFinite()&&p.q.norm() > .99&&p.q.norm() < 1.01, "Invalid pose");
    p.q.normalize();
    return p;
  }
  MapWindow make_window(const Config& cfg, const Eigen::Vector2d& c)
  {
    MapWindow w;
    if (!c.allFinite()) return w;
    const Key centre{int64_t(std::floor(c.x() / cfg.resolution)), int64_t(std::floor(c.y() / cfg.resolution))};
    w.centre = centre;
    w.max_cells = cfg.max_map_cells;
    w.keep_cells = int64_t(std::ceil(cfg.map_keep_radius / cfg.resolution));
    if (!(cfg.map_length > 0.)) return w;
    const int64_t cells = int64_t(std::llround(cfg.map_length / cfg.resolution));
    w.valid = true;
    w.width = w.height = cells;
    w.xmin = centre.first - cells / 2;
    w.ymin = centre.second - cells / 2;
    return w;
  }
}

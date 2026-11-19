#include "nomad_costmap/detail.hpp"
#include "nomad_costmap/moving_front_free_region.hpp"
#include <Eigen/Eigenvalues>
#include <nanoflann.hpp>
#include <algorithm>
#include <numeric>
#include <tuple>

namespace nomad {
  Config Config::load(const Json& semantic, const Json& slope)
  {
    Config c;
    c.semantic = semantic;
    const auto moving_free = semantic.value("moving_front_free_region", Json::object());
    if (moving_free.value("enabled",false)) {
      const double radius = moving_free.value("radius_m",.5);
      require(std::isfinite(radius) && radius > 0. && radius <= 5., "Invalid moving front free radius");
      require(moving_free.value("field_of_view_deg",180.) == 180., "Moving free region must be a forward semicircle");
      require(moving_free.value("center_reference",std::string("current_public_base_link")) == "current_public_base_link",
              "Unexpected moving free center reference");
      const auto policy = moving_free.value("override_policy",std::string("all_costs_zero_except_person"));
      require(policy == "all_costs_zero_except_person" || policy == "semantic_zero_measured_slope_kept_except_person",
              "Unexpected moving free override policy");
    }
    if (slope.contains("slope_forward_axis_base_link")) {
      const auto& a = slope.at("slope_forward_axis_base_link");
      require(a.is_array() && a.size() == 3, "Forward axis requires three base_link components");
      c.slope_forward_axis_base_link = V3(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
      require(c.slope_forward_axis_base_link.allFinite() &&
              std::isfinite(c.slope_forward_axis_base_link.norm()) &&
              c.slope_forward_axis_base_link.norm() >= 1e-8, "Invalid slope forward axis");
    }
    const auto restore = semantic.value("person_ray_restore", Json::object());
    c.person_ray_restore = restore.value("enabled", false);
    c.person_class_id = restore.value("person_class_id", 1);
    if (c.person_ray_restore) {
      require(!restore.value("height_check", false), "Person ray restoration uses XY only");
      const auto& person = semantic.at("classes").at(std::to_string(c.person_class_id));
      require(person.at("kind") == "dynamic" && person.at("cost") == 100,
              "Person restoration requires a dynamic cost100 class");
    }
    require(semantic.value("visibility_mode", "") == "projection_only", "C++ selected profile requires projection_only");
    require(semantic.value("camera_source", "") == "zed"&&semantic.value("oak_input", "") == "external_pointcloud", "Expected external Pi5 OAK and ZED labels");
    require(semantic.at("local_ground").value("enabled", false), "Selected local geometry mode required");
    c.resolution = slope.value("resolution_m", .12);
    c.radius = slope.value("radius_m", .18);
    c.window = slope.value("raw_window_s", 3.);
    c.forward_radius = slope.value("forward_update_radius_m", 3.);
    c.max_size = slope.value("max_size_m", 160.);
    c.bin = slope.value("plane_bin_size_m", .03);
    c.minor = slope.value("plane_min_minor_std_m", .01);
    c.variance_ratio = slope.value("plane_min_xy_variance_ratio", .02);
    c.jump = slope.value("plane_max_connected_height_jump_m", .12);
    c.rms = slope.value("plane_max_refit_rms_m", .01);
    c.target_error = slope.value("plane_max_target_error_m", .02);
    c.oak_lidar_height_gate = slope.value("oak_lidar_height_gate_enabled", true);
    c.freeze_slope_after_first_valid = slope.value("freeze_slope_after_first_valid", false);
    c.retry_slope_score_at_least_100 = slope.value("retry_slope_score_at_least_100", false);
    c.slope_compute_threads = slope.value("slope_compute_threads", 1);
    c.slope_parallel_min_cells = slope.value("slope_parallel_min_cells", 32);
    require(c.slope_compute_threads >= 1 && c.slope_compute_threads <= 32 &&
            c.slope_parallel_min_cells >= 1, "Invalid slope parallel configuration");
    c.startup_free_region_enabled = semantic.value("startup_free_region", Json::object()).value("enabled", false);
    c.startup_freeze_initial_slope = c.startup_free_region_enabled&&
      semantic.value("startup_free_region", Json::object()).value("freeze_initial_slope", false);
    c.startup_person_cost_overlay = semantic.value("startup_free_region", Json::object())
      .value("person_cost_overlay_enabled", false);
    if (c.startup_person_cost_overlay) {
      require(c.startup_freeze_initial_slope && c.person_ray_restore,
              "Startup person costs require initial frozen slopes and person-ray restoration");
      require(semantic.at("startup_free_region").value("person_clear_policy",std::string()) ==
              "lidar_xy_ray_only", "Startup person costs require actual LiDAR ray clearance");
    }
    if (semantic.contains("startup_free_region")) {
      const auto& startup = semantic.at("startup_free_region");
      const double startup_radius = startup.value("radius_m",1.5);
      require(std::isfinite(startup_radius) && startup_radius > 0. &&
              startup.value("field_of_view_deg",360.) == 360.,
              "Selected startup free region requires a finite positive radius and 360 degrees");
    }
    c.oak_lidar_xy_radius = slope.value("oak_lidar_xy_radius_m", .06);
    c.oak_lidar_z_reject = slope.value("oak_lidar_max_abs_z_difference_m", .12);
    require(std::isfinite(c.oak_lidar_xy_radius)&&c.oak_lidar_xy_radius > 0&&
            std::isfinite(c.oak_lidar_z_reject)&&c.oak_lidar_z_reject > 0,
            "Invalid OAK/LiDAR height gate distances");
    c.tls_gap = slope.value("tls_gap_ratio", 1e-8);
    c.xy_rank = slope.value("xy_rank_ratio", 1e-6);
    c.min_raw = slope.value("plane_min_raw_points", 12);
    c.min_bins = slope.value("plane_min_xy_bins", 6);
    c.min_heights = slope.value("plane_min_height_cells", 6);
    c.raw_reference = slope.value("raw_reference_points", 30);
    c.per_cell = slope.value("per_cell_max_points", 512);
    c.global_points = slope.value("global_max_points", 1000000);
    c.space_fraction = slope.value("space_reference_fraction", .1);
    // 2026-11-13 additions (absent keys keep the legacy behaviour).
    const std::string reference = slope.value("slope_angle_reference", std::string("heading"));
    require(reference == "heading" || reference == "gravity", "slope_angle_reference must be heading or gravity");
    c.gravity_slope_angle = reference == "gravity";
    c.startup_seed_frozen = slope.value("startup_seed_keep_frozen", true);
    c.map_length = slope.value("map_length_m", 0.);
    require(std::isfinite(c.map_length) && (c.map_length == 0. || (c.map_length >= 4. && c.map_length <= 100.)),
            "map_length_m must be 0 (unbounded) or 4..100 m");
    c.keep_slope_on_failure = slope.value("keep_slope_on_failure", false);
    {
      const std::string area = slope.value("cost_area", std::string("map_window"));
      require(area == "map_window" || area == "known_cells", "cost_area must be map_window or known_cells");
      c.cost_known_cells = area == "known_cells";
      const int64_t cap = slope.value("max_map_cells", int64_t(0));
      require(cap == 0 || (cap >= 1000 && cap <= 1000000), "max_map_cells must be 0 (off) or 1000..1000000");
      c.max_map_cells = size_t(cap);
      c.map_keep_radius = slope.value("map_keep_radius_m", 6.);
      require(std::isfinite(c.map_keep_radius) && c.map_keep_radius >= 1. && c.map_keep_radius <= 30.,
              "map_keep_radius_m must be 1..30 m");
    }
    {
      // 2026-11-16: slope score coefficient k and blocking angle B (defaults 4 and 20).
      const Json k = slope.value("slope_score_per_deg", Json());
      require(k.is_null() || k.is_number(), "slope_score_per_deg must be a number");
      c.slope_score_per_deg = k.is_null() ? 4. : k.get<double>();
      require(std::isfinite(c.slope_score_per_deg) && c.slope_score_per_deg > 0. && c.slope_score_per_deg <= 100.,
              "slope_score_per_deg must be > 0 and <= 100");
      const Json b = slope.value("slope_block_deg", Json());
      require(b.is_null() || b.is_number(), "slope_block_deg must be a number");
      c.slope_block_deg = b.is_null() ? 80. / c.slope_score_per_deg : b.get<double>();
      require(std::isfinite(c.slope_block_deg) && c.slope_block_deg > 0. && c.slope_block_deg <= 90.,
              "slope_block_deg must be > 0 and <= 90");
      require(c.slope_score_per_deg * c.slope_block_deg <= 100. + 1e-9,
              "slope_score_per_deg * slope_block_deg must be <= 100 (score must not pass 100 below the block angle)");
    }
    {
      const auto accumulation = semantic.at("local_ground").value("label_accumulation", Json::object());
      c.label_accumulation = accumulation.value("enabled", false);
      c.label_accumulation_max = accumulation.value("max_labels_per_tick", 4);
      require(c.label_accumulation_max >= 1 && c.label_accumulation_max <= 32,
              "label_accumulation.max_labels_per_tick must be 1..32");
    }
    require(c.resolution > 0&&c.radius > 0&&c.window > 0&&c.bin > 0&&c.per_cell >= 2&&c.global_points >= 2, "Invalid slope configuration");
    require(std::abs(semantic.at("grid_resolution_m").get<double>() - c.resolution) <= 1e-9, "Semantic and slope resolutions differ");
    auto local = semantic.at("local_ground");
    require(local.value("retention_s", 3.) <= 3.&&local.value("sample_period_s", .2) >= .2, "Local geometry limits exceed selected profile");
    require(semantic.at("label_image").value("encoding", "") == "mono8"&&semantic.at("label_image").value("geometry", "") == "full_frame_resize", "Unsupported semantic image geometry");
    require(!semantic.at("ground_class_ids").empty(), "No ground class definitions");
    for (auto it = semantic.at("classes").begin(); it != semantic.at("classes").end(); ++it)
    {
      int id = std::stoi(it.key());
      require(id >= 0&&id <= 255&&std::to_string(id) == it.key(), "Invalid class id");
      auto v = it.value();
      require(v.contains("kind")&&v.contains("cost"), "Invalid class");
      if (!v["cost"].is_null())
        require(v["cost"].is_number_integer()&&v["cost"].get<int>() >= 0&&v["cost"].get<int>() <= 100, "Invalid class cost");
      auto& info = c.classes[size_t(id)];
      info.known = true;
      info.ignore = v.value("is_ignore", false);
      info.kind = v.at("kind").get<std::string>();
      info.dynamic = info.kind == "dynamic";
      info.cost = v["cost"].is_null()?-1:v["cost"].get<int>();
    }
    return c;
  }
  V3 slope_forward_axis_in_imu(const Config& cfg, const M4& base_from_imu)
  {
    // R_world_imu * R_base_imu^T * a_base = R_world_base * a_base.
    return base_from_imu.block<3,3>(0,0).transpose() * cfg.slope_forward_axis_base_link;
  }
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
  void ROI::set_pose(const Pose& p)
  {
    pose = p;
    M3 R = p.q.toRotationMatrix();
    for (int s = 0; s < 2; ++s)
    {
      origins[s] = (R * mounts.sensor[s].block<3, 1>(0, 3) + p.p).head<2>();
      V3 axis = mounts.sensor[s].block<3, 3>(0, 0).col(s == 0?0:2);
      forward[s] = (R * axis).head<2>();
      require(forward[s].norm() >= 1e-8, "Sensor forward has no horizontal projection");
      forward[s].normalize();
    }
    valid = true;
  }
  bool ROI::contains(const Eigen::Vector2d& xy, double margin)const
  {
    if (!valid)
      return false;
    for (int s = 0; s < 2; ++s)
    {
      auto d = xy - origins[s];
      if (d.squaredNorm() <= (radius + margin) * (radius + margin) + 1e-10&&d.dot(forward[s]) >= -margin - 1e-10)
        return true;
    }
    return false;
  }
  bool spread(const Points& p, const Config& c)
  {
    if (p.size() < 3)
      return false;
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (auto& v:p)
      mean += v.head<2>();
    mean /= double(p.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (auto& v:p)
    {
      Eigen::Vector2d d = v.head<2>() - mean;
      cov += d * d.transpose();
    }
    cov /= double(p.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> e(cov);
    auto a = e.eigenvalues();
    return e.info() == Eigen::Success&&std::sqrt(std::max(0., a[0])) >= c.minor&&a[1] > 0&&std::max(0., a[0]) / a[1] >= c.variance_ratio;
  }
  std::optional<Fit> fit_tls(const Points& p, const std::vector<double>& supplied, const Config& c, bool weighted)
  {
    if (p.size() < size_t(weighted?c.min_bins:3)||(!supplied.empty()&&supplied.size() != p.size()))
      return {}
    ;
    for (auto& v:p)
      if (!v.allFinite())
        return {}
    ;
    if (weighted&&!spread(p, c))
      return {}
    ;
    std::vector<double> w = supplied.empty()?std::vector<double>(p.size(), 1.):supplied;
    double sum = std::accumulate(w.begin(), w.end(), 0.);
    if (!(sum > 0))
      return {}
    ;
    for (auto& x:w)
      x /= sum;
    V3 mean = V3::Zero();
    for (size_t i = 0; i < p.size(); ++i)
      mean += p[i] * w[i];
    M3 cov = M3::Zero();
    double scale = weighted?1.:c.radius;
    for (size_t i = 0; i < p.size(); ++i)
    {
      V3 d = (p[i] - mean) / scale;
      cov.noalias() += w[i] * d*d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> xy(cov.topLeftCorner<2, 2>());
    Eigen::SelfAdjointEigenSolver<M3> eigen(cov);
    if (xy.info() != Eigen::Success||eigen.info() != Eigen::Success)
      return {}
    ;
    auto x = xy.eigenvalues();
    auto e = eigen.eigenvalues();
    if (x[1] <= 0||e[2] <= 0||e[1] - e[0] <= c.tls_gap * e[2])
      return {}
    ;
    if (weighted)
    {
      if (x[0] < c.minor * c.minor||x[0] / x[1] < c.variance_ratio)
        return {}
      ;
    }
    else if (x[0] <= c.xy_rank * c.xy_rank * x[1])
      return {}
    ;
    V3 normal = eigen.eigenvectors().col(0).normalized();
    if (normal.z() < 0)
      normal = -normal;
    double res = 0;
    for (size_t i = 0; i < p.size(); ++i)
    {
      double d = (p[i] - mean).dot(normal);
      res += w[i] * d * d;
    }
    res = std::sqrt(res);
    if (weighted&&res > c.rms)
      return {}
    ;
    double q = std::clamp(std::max(0., x[0]) / (weighted?c.radius * c.radius:1.) / c.space_fraction, 0., 1.);
    return Fit{normal, -normal.dot(mean), std::atan2(std::hypot(normal.x(), normal.y()), std::abs(normal.z())) * 180 / pi, res, q};
  }
  struct CloudAdaptor {
    const Points& p;
    size_t kdtree_get_point_count()const
    {
      return p.size();
    }
    double kdtree_get_pt(size_t i, size_t d)const
    {
      return p[i][d];
    }
    template<class B>bool kdtree_get_bbox(B&)const
    {
      return false;
    }
  };
  Points frame_voxels(const Points& camera, const Points& world)
  {
    require(camera.size() == world.size(), "Camera/world points differ");
    Points p;
    for (size_t i = 0; i < world.size(); ++i)
    {
      require(world[i].allFinite(), "Nonfinite world point");
      if (camera[i].allFinite()&&camera[i].z() >= .3&&camera[i].z() <= 4.)
        p.push_back(world[i]);
    }
    if (p.size() > 80)
    {
      CloudAdaptor cloud{p};
      using Tree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, CloudAdaptor>, CloudAdaptor, 3, size_t>;
      Tree tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(16));
      tree.buildIndex();
      std::vector<double> distances(p.size());
      std::vector<size_t> indices(81);
      std::vector<double> dd(81);
      for (size_t i = 0; i < p.size(); ++i)
      {
        require(tree.knnSearch(p[i].data(), 81, indices.data(), dd.data()) == 81, "KNN query incomplete");
        double sum = 0;
        for (int k = 1; k < 81; ++k)
          sum += std::sqrt(dd[k]);
        distances[i] = sum / 80.;
      }
      double mean = std::accumulate(distances.begin(), distances.end(), 0.) / distances.size();
      double variance = 0;
      for (auto d:distances)
        variance += (d - mean) * (d - mean);
      double limit = mean + std::sqrt(variance / distances.size());
      Points kept;
      for (size_t i = 0; i < p.size(); ++i)
        if (distances[i] <= limit)
          kept.push_back(p[i]);
      p.swap(kept);
    }
    using K3 = std::tuple<int64_t, int64_t, int64_t>;
    struct Sum {
      V3 p = V3::Zero();
      int n = 0;
    };
    std::map<K3, Sum> bins;
    for (auto& v:p)
    {
      K3 k{int64_t(std::floor(v.x() / .03)), int64_t(std::floor(v.y() / .03)), int64_t(std::floor(v.z() / .03))};
      auto& s = bins[k];
      s.p += v;
      ++s.n;
    }
    Points out;
    for (auto& [k, s]:bins)
      out.push_back(s.p / double(s.n));
    return out;
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


  std::array<M4, 2> factory_mounts(const Json& message, const Json& config)
  {
    require(message.at("calibration_source") == "factory"&&message.at("device_id") == config.at("oak_device_id"), "OAK factory provenance/serial mismatch");
    const auto& factory = message.at("factory_calibration");
    require(factory.at("productName").get<std::string>().rfind("OAK-D-LITE", 0) == 0, "Expected OAK-D-LITE raw BMI270");
    std::map<int, Json> cams;
    for (auto& pair:factory.at("cameraData"))
    {
      require(pair.size() == 2, "Bad cameraData");
      cams[pair[0].get<int>()] = pair[1];
    }
    const auto& exp = config.at("expected_oak_camera_info");
    auto& b = cams.at(exp.at("socket").get<int>());
    require(b.at("width") == exp.at("width")&&b.at("height") == exp.at("height"), "OAK resolution differs from calibration");
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        require(std::abs(b.at("intrinsicMatrix")[r][c].get<double>() - exp.at("K")[r * 3 + c].get<double>()) <= exp.at("pixel_tolerance").get<double>(), "OAK K differs from calibration");
    require(b.at("distortionCoeff").size() == exp.at("D").size(), "OAK distortion dimensions differ");
    for (size_t i = 0; i < exp.at("D").size(); ++i)
      require(std::abs(b.at("distortionCoeff")[i].get<double>() - exp.at("D")[i].get<double>()) <= exp.at("distortion_tolerance").get<double>(), "OAK distortion differs from calibration");
    std::map<int, std::vector<std::pair<int, M4> > > graph;
    for (auto& [source, cam]:cams)
    {
      auto e = cam.value("extrinsics", Json::object());
      int target = e.value("toCameraSocket", -1);
      if (target < 0)
        continue;
      require(e.value("lengthUnit", -1) == 1, "Factory translation must use centimeters");
      M4 t = M4::Identity();
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          t(r, c) = e.at("rotationMatrix")[r][c];
      for (int i = 0; i < 3; ++i)
        t(i, 3) = e.at("translation").at(std::array<const char*, 3>{"x", "y", "z"}[i]).get<double>() / 100.;
      rigid(matrix_json(t));
      graph[source].push_back({target, t});
      graph[target].push_back({source, t.inverse()});
    }
    auto chain = [&](int target){
                   std::deque<std::pair<int, M4> > q{{0, M4::Identity()}};
                   std::set<int> seen;
                   while (!q.empty())
                   {
                     auto cur = q.front();
                     q.pop_front();
                     if (cur.first == target)
                       return cur.second;
                     if (!seen.insert(cur.first).second)
                       continue;
                     for (auto& [dst, t]:graph[cur.first])
                       if (!seen.count(dst))
                         q.push_back({dst, t * cur.second});
                   }
                   throw std::runtime_error("Factory camera transform chain missing");
                 };
    M4 raw = M4::Identity();
    raw(1, 1) = -1;
    raw(2, 2) = -1;
    raw.block<3, 1>(0, 3) = V3(-.0315, .006439, -.002);
    M4 a_b = chain(1), a_c = chain(2);
    double baseline = (a_b.inverse().block<3, 1>(0, 3) - a_c.inverse().block<3, 1>(0, 3)).norm();
    require(baseline >= .05&&baseline <= .1, "Invalid stereo baseline");
    auto transpose_inverse = [](const M4& t){
                               M4 inv = M4::Identity();
                               inv.block<3, 3>(0, 0) = t.block<3, 3>(0, 0).transpose();
                               inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * t.block<3, 1>(0, 3);
                               return inv;
                             };
    return {transpose_inverse(a_b * raw), transpose_inverse(raw)};
  }
  Mounts sensor_mounts(const M4& ib, const M4& bl, const M4& ia)
  {
    V3 d(bl(2, 0), bl(2, 1), 0);
    require(d.norm() >= 1e-8, "LiDAR forward cannot be projected");
    d.normalize();
    M4 basis = M4::Identity();
    basis.block<3, 1>(0, 0) = d;
    basis.block<3, 1>(0, 1) = V3(-d.y(), d.x(), 0);
    Mounts m;
    m.sensor = {ib*bl*basis, ia};
    return m;
  }
  ScanPair scan_pair(const ScanInput& scan, const CloudInput& cloud, const M4& cl)
  {
    int64_t stamp = scan.stamp_ns;
    require(cloud.stamp_ns == stamp, "Scan/cloud stamps differ");
    // points_from_json() rejected any nonfinite XYZ.
    for (const auto& v:cloud.points)
      require(v.allFinite(), "Nonfinite XYZ");
    const auto& p = cloud.points;
    const auto& times = cloud.offsets;
    require(p.size() >= 2&&p.size() == times.size(), "Bad point/time buffers");
    double previous = -1;
    for (auto t:times)
    {
      require(std::isfinite(t)&&t >= 0&&t <= .3&&t > previous, "Invalid SDK acquisition time order");
      previous = t;
    }
    double amin = scan.angle_min, inc = scan.angle_increment, lo = scan.range_min, hi = scan.range_max;
    require(inc > 0, "Expected increasing scan bins");
    const auto& ranges = scan.ranges;
    std::map<int64_t, size_t> last;
    for (size_t i = 0; i < p.size(); ++i)
    {
      auto bin = int64_t(std::ceil((std::atan2(p[i].y(), p[i].x()) - amin) / inc));
      if (bin >= 0&&bin < int64_t(ranges.size()))
        last[bin] = i;
    }
    std::vector<size_t> indices;
    for (auto [bin, i]:last)
      indices.push_back(i);
    std::sort(indices.begin(), indices.end());
    ScanPair out;
    out.stamp_ns = stamp;
    require(std::hypot(cl(2, 0), cl(2, 1)) >= 1e-6, "Camera forward perpendicular to scan plane");
    for (auto i:indices)
    {
      int64_t bin = int64_t(std::ceil((std::atan2(p[i].y(), p[i].x()) - amin) / inc));
      double rr = ranges[bin];   // NaN replaces the former JSON null
      if (!std::isfinite(rr)||rr < lo||rr > hi||std::abs(rr - p[i].head<2>().norm()) >= .003)
        continue;
      double angle = amin + bin * inc;
      V3 local(rr * std::cos(angle), rr * std::sin(angle), 0);
      if (cl.block<1, 3>(2, 0).dot(local) < -1e-10)
        continue;
      out.points_lidar_m.push_back(local);
      out.point_time_offsets_s.push_back(times[i]);
      out.bin_indices.push_back(bin);
    }
    require(out.points_lidar_m.size() >= 5, "Too few forward matched points");
    return out;
  }
  Frame deskew(const ScanPair& pair, const std::vector<Pose>& poses, const M4& il, double offset)
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
      auto pose = interpolate(poses, ns);
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

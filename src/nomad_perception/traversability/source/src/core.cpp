#include "nomad_costmap/core.hpp"
#include <Eigen/Eigenvalues>
#include <nanoflann.hpp>
#include <algorithm>
#include <cfenv>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace nomad {
  static constexpr double pi = 3.14159265358979323846;
  static void require(bool b, const std::string& s)
  {
    if (!b)
      throw std::runtime_error(s);
  }
  static Json num(double v)
  {
    return std::isfinite(v)?Json(v):Json(nullptr);
  }
  static double val(const Json& v)
  {
    return v.is_null()?nan:v.get<double>();
  }
  static void count(Json& c, const char* k, uint64_t n = 1)
  {
    c[k] = c.value(k, uint64_t(0)) + n;
  }
  static Key key_of(const V3& v, double size)
  {
    return {int64_t(std::floor(v.x() / size)), int64_t(std::floor(v.y() / size))};
  }
  static Eigen::Vector2d center(Key k, double size)
  {
    return {(k.first + .5) * size, (k.second + .5) * size};
  }
  Config Config::load(const Json& semantic, const Json& slope)
  {
    Config c;
    c.semantic = semantic;
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
    c.tls_gap = slope.value("tls_gap_ratio", 1e-8);
    c.xy_rank = slope.value("xy_rank_ratio", 1e-6);
    c.min_raw = slope.value("plane_min_raw_points", 12);
    c.min_bins = slope.value("plane_min_xy_bins", 6);
    c.min_heights = slope.value("plane_min_height_cells", 6);
    c.raw_reference = slope.value("raw_reference_points", 30);
    c.per_cell = slope.value("per_cell_max_points", 512);
    c.global_points = slope.value("global_max_points", 1000000);
    c.space_fraction = slope.value("space_reference_fraction", .1);
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
    }
    return c;
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
  static bool spread(const Points& p, const Config& c)
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
  Ground::Ground(const Config& c, const Mounts& m) : cfg(c)
  {
    roi.mounts = m;
    roi.radius = c.forward_radius;
  }
  void Ground::ingest(const Frame& frame, bool revision)
  {
    require(frame.source == 0||frame.source == 1, "Invalid source");
    require(frame.stamp > 0&&frame.points.size() == frame.stamps.size(), "Invalid geometry stamps");
    int source = frame.source;
    int64_t stamp = frame.stamp;
    if (revision&&latest > 0&&double(stamp) / 1e9 < double(latest) / 1e9 - cfg.window)
      return;
    if (!revision)
      require(stamp > last_stamp[source], "Duplicate/reordered geometry");
    if (revision)
    {
      for (auto it = samples.begin(); it != samples.end();)
      {
        auto& group = it->second.sources[source];
        auto n = group.size();
        group.erase(std::remove_if(group.begin(), group.end(), [&](auto& v){
          return double(v.stamp) / 1e9 == double(stamp) / 1e9;
        }), group.end());
        if (n != group.size())
          dirty.insert(it->first);
        if (group.empty()&&it->second.sources[1 - source].empty())
          it = samples.erase(it);
        else
          ++it;
      }
      ++revisions;
      insertion.erase(std::remove_if(insertion.begin(), insertion.end(),
                                     [this](Key key){
        return !samples.count(key);
      }), insertion.end());
    }
    if (frame.pose.stamp >= roi.pose.stamp)
      roi.set_pose(frame.pose);
    auto b = stamp / 200000000;
    if (!revision&&b == last_bucket[source])
    {
      last_stamp[source] = stamp;
      return;
    }
    last_stamp[source] = std::max(stamp, last_stamp[source]);
    last_bucket[source] = std::max(b, last_bucket[source]);
    latest = std::max(latest, stamp);
    int half = int(std::ceil(cfg.max_size / cfg.resolution / 2));
    double bound = half * cfg.resolution;
    std::map<Key, Points> groups;
    for (size_t i = 0; i < frame.points.size(); ++i)
    {
      auto p = frame.points[i];
      require(p.allFinite()&&frame.stamps[i] > 0, "Invalid selected point");
      if (p.x() < -bound||p.x() >= bound||p.y() < -bound||p.y() >= bound)
        continue;
      auto key = key_of(p, cfg.resolution);
      observed.insert(key);
      dirty.insert(key);
      last_observed[key] = std::max(last_observed[key], frame.stamps[i]);
      source_capture[source] = std::max(source_capture[source], frame.stamps[i]);
      groups[key].push_back(p.cast<float>().cast<double>());
    }
    for (auto& [k, p]:groups)
    {
      if (!samples.count(k))
        insertion.push_back(k);
      auto& q = samples[k].sources[source];
      q.push_back({stamp, std::move(p)});
      size_t total = 0;
      for (auto& v:q)
        total += v.points.size();
      size_t excess = total > size_t(cfg.per_cell / 2)?total - cfg.per_cell / 2:0;
      while (excess&&!q.empty())
      {
        auto& old = q.front();
        size_t remove = std::min(excess, old.points.size()), keep = old.points.size() - remove;
        if (keep)
        {
          Points subset;
          subset.reserve(keep);
          for (size_t i = 0; i < keep; ++i)
          {
            size_t ix = keep == 1?0:size_t(double(i) * double(old.points.size() - 1) / double(keep - 1));
            subset.push_back(old.points[ix]);
          }
          old.points.swap(subset);
        }
        else
          q.pop_front();
        excess -= remove;
        count(counters, "capacity_evictions", remove);
      }
    }
    double cutoff = double(latest) / 1e9 - cfg.window;
    for (auto it = samples.begin(); it != samples.end();)
    {
      for (auto& q:it->second.sources)
        while (!q.empty()&&double(q.front().stamp) / 1e9 < cutoff)
        {
          q.pop_front();
          dirty.insert(it->first);
          count(counters, "expired_captures");
        }
      if (it->second.sources[0].empty()&&it->second.sources[1].empty())
        it = samples.erase(it);
      else
        ++it;
    }
    insertion.erase(std::remove_if(insertion.begin(), insertion.end(),
                                   [this](Key key){
      return !samples.count(key);
    }), insertion.end());
    for (int s = 0; s < 2; ++s)
    {
      std::vector<std::pair<int64_t, Key> > oldest;
      size_t total = 0;
      for (auto& [k, cell]:samples)
      {
        auto& q = cell.sources[s];
        if (!q.empty())
          oldest.push_back({q.front().stamp, k});
        for (auto& v:q)
          total += v.points.size();
      }
      if (total > size_t(cfg.global_points / 2))
      {
        std::sort(oldest.begin(), oldest.end());
        for (auto& [t, k]:oldest)
        {
          auto& q = samples[k].sources[s];
          while (!q.empty()&&total > size_t(cfg.global_points / 2))
          {
            total -= q.front().points.size();
            count(counters, "global_cap_evictions", q.front().points.size());
            q.pop_front();
            dirty.insert(k);
          }
        }
      }
    }
  }
  std::vector<Ground::Rep> Ground::prepare()const
  {
    std::vector<Rep> out;
    struct Bin {
      V3 sum = V3::Zero();
      int n = 0;
      double lo = INFINITY, hi = -INFINITY;
    };
    // Insertion order is retained explicitly; expired entries are skipped.
    std::vector<Key> order;
    std::set<Key> seen;
    for (auto k:insertion)
      if (samples.count(k)&&seen.insert(k).second)
        order.push_back(k);
    for (int s = 0; s < 2; ++s)
    {
      std::map<Key, Bin> bins;
      for (auto k:order)
      {
        if (!roi.contains(center(k, cfg.resolution), cfg.radius + cfg.resolution / std::sqrt(2.)))
          continue;
        for (auto& capture:samples.at(k).sources[s])
          for (auto& p:capture.points)
          {
            auto key = key_of(p, cfg.bin);
            auto& b = bins[key];
            b.sum += p;
            ++b.n;
            b.lo = std::min(b.lo, p.z());
            b.hi = std::max(b.hi, p.z());
          }
      }
      for (auto& [k, b]:bins)
        out.push_back({b.sum / double(b.n), k, b.n, b.hi - b.lo});
    }
    return out;
  }
  static bool contains_target(const Points& p, const Eigen::Vector2d& target)
  {
    std::set<Key> unused;
    (void)unused;
    std::vector<Eigen::Vector2d> xy;
    for (auto& v:p)
      xy.push_back(v.head<2>());
    auto less = [](auto& a, auto& b){
                  return a.x() < b.x()||(a.x() == b.x()&&a.y() < b.y());
                };
    std::sort(xy.begin(), xy.end(), less);
    xy.erase(std::unique(xy.begin(), xy.end(), [](auto& a, auto& b){
      return a == b;
    }), xy.end());
    if (xy.size() < 3)
      return false;
    auto cross = [](auto& a, auto& b, auto& c){
                   return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
                 };
    std::vector<Eigen::Vector2d> lo, hi;
    for (auto& v:xy)
    {
      while (lo.size() >= 2&&cross(lo[lo.size() - 2], lo.back(), v) <= 0)
        lo.pop_back();
      lo.push_back(v);
    }
    for (auto it = xy.rbegin(); it != xy.rend(); ++it)
    {
      auto v = *it;
      while (hi.size() >= 2&&cross(hi[hi.size() - 2], hi.back(), v) <= 0)
        hi.pop_back();
      hi.push_back(v);
    }
    lo.pop_back();
    hi.pop_back();
    lo.insert(lo.end(), hi.begin(), hi.end());
    if (lo.size() < 3)
      return false;
    for (size_t i = 0; i < lo.size(); ++i)
    {
      auto a = lo[i], b = lo[(i + 1) % lo.size()], c = target;
      if (cross(a, b, c) < -1e-10)
        return false;
    }
    return true;
  }
  std::optional<Fit> Ground::refit(Key target)const
  {
    auto h = heights.find(target);
    if (h == heights.end()||!std::isfinite(h->second.first))
      return {}
    ;
    std::set<Key> candidates;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
      {
        Key k{target.first + dx, target.second + dy};
        auto it = heights.find(k);
        if (it != heights.end()&&std::isfinite(it->second.first))
          candidates.insert(k);
      }
    std::set<Key> connected{target};
    std::vector<Key> pending{target};
    while (!pending.empty())
    {
      auto cur = pending.back();
      pending.pop_back();
      for (auto d:std::array<Key, 4>{Key{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
      {
        Key k{cur.first + d.first, cur.second + d.second};
        if (!candidates.count(k))
          continue;
        if (std::abs(heights.at(k).first - heights.at(cur).first) > cfg.jump)
          return {}
        ;
        if (connected.insert(k).second)
          pending.push_back(k);
      }
    }
    connected.erase(target);
    if (connected.size() < size_t(cfg.min_heights))
      return {}
    ;
    Points p;
    for (auto k:connected)
    {
      auto xy = center(k, cfg.resolution);
      p.push_back({xy.x(), xy.y(), heights.at(k).first});
    }
    auto xy = center(target, cfg.resolution);
    if (!spread(p, cfg)||!contains_target(p, xy))
      return {}
    ;
    auto fitted = fit_tls(p, {}, cfg, false);
    if (!fitted||fitted->residual > cfg.rms||std::abs(fitted->normal.z()) < 1e-8)
      return {}
    ;
    V3 v(xy.x(), xy.y(), h->second.first);
    if (std::abs(fitted->normal.dot(v) + fitted->offset) / std::abs(fitted->normal.z()) > cfg.target_error)
      return {}
    ;
    return fitted;
  }
  Json Ground::snapshot()
  {
    std::set<Key> active, changed;
    for (auto k:observed)
      if (roi.contains(center(k, cfg.resolution)))
        active.insert(k);
    std::set_difference(active.begin(), active.end(), previous_active.begin(), previous_active.end(), std::inserter(changed, changed.end()));
    int reach = int(std::ceil(cfg.radius / cfg.resolution));
    for (auto k:dirty)
      if (roi.contains(center(k, cfg.resolution), cfg.radius + 2 * cfg.resolution))
        for (int dx = -reach; dx <= reach; ++dx)
          for (int dy = -reach; dy <= reach; ++dy)
          {
            Key q{k.first + dx, k.second + dy};
            if (active.count(q))
              changed.insert(q);
          }
    if (!changed.empty())
    {
      auto reps = prepare();
      std::map<Key, std::vector<size_t> > bins;
      for (size_t i = 0; i < reps.size(); ++i)
        bins[reps[i].bin].push_back(i);
      for (auto k:changed)
      {
        auto xy = center(k, cfg.resolution);
        int rr = int(std::ceil((cfg.radius + 1e-9) / cfg.bin)) + 1;
        Key bc{int64_t(std::floor(xy.x() / cfg.bin)), int64_t(std::floor(xy.y() / cfg.bin))};
        std::vector<size_t> neighbors;
        for (int dx = -rr; dx <= rr; ++dx)
          for (int dy = -rr; dy <= rr; ++dy)
          {
            auto it = bins.find({bc.first + dx, bc.second + dy});
            if (it != bins.end())
              for (auto i:it->second)
                if ((reps[i].mean.head<2>() - xy).squaredNorm() <= std::pow(cfg.radius + 1e-9, 2))
                  neighbors.push_back(i);
          }
        std::sort(neighbors.begin(), neighbors.end());
        int raw = 0;
        bool conflict = false;
        std::map<Key, int> hist;
        Points p;
        for (auto i:neighbors)
        {
          raw += reps[i].count;
          conflict |= reps[i].span > cfg.jump;
          ++hist[reps[i].bin];
          p.push_back(reps[i].mean);
        }
        std::optional<Fit> fit;
        if (!conflict&&raw >= cfg.min_raw&&hist.size() >= size_t(cfg.min_bins))
        {
          std::vector<double> w;
          for (auto i:neighbors)
            w.push_back(1. / hist[reps[i].bin]);
          fit = fit_tls(p, w, cfg, true);
          if (fit)
            count(counters, "direct_tls_success");
        }
        if (conflict)
        {
          heights.erase(k);
          count(counters, "stage3_within_sensor_conflicts");
        }
        Points central;
        std::vector<double> medians;
        auto cell = samples.find(k);
        if (cell != samples.end())
          for (auto& group:cell->second.sources)
          {
            std::vector<double> zs;
            for (auto& cap:group)
              for (auto& v:cap.points)
              {
                zs.push_back(v.z());
                central.push_back(v);
              }
            if (!zs.empty())
              medians.push_back(median(zs));
          }
        if (!medians.empty()&&!conflict)
          heights[k] = {std::accumulate(medians.begin(), medians.end(), 0.) / medians.size(), latest}
        ;
        if (!fit&&!conflict)
        {
          count(counters, "height_refit_attempts");
          fit = refit(k);
          if (fit)
            count(counters, "height_refit_success");
        }
        auto old = estimates.find(k);
        bool reused = false;
        if (!fit&&old != estimates.end()&&old->second.valid&&!conflict)
        {
          std::vector<double> distances;
          for (auto& v:central)
            distances.push_back(std::abs(v.dot(old->second.fit.normal) + old->second.fit.offset));
          if (distances.empty()||median(distances) <= cfg.target_error)
          {
            old->second.reused = true;
            reused = true;
            count(counters, "previous_slope_reused");
          }
        }
        if (!reused)
        {
          Estimate e;
          e.height = heights.count(k)?heights.at(k).first:nan;
          e.raw_count = raw;
          if (fit)
          {
            e.valid = true;
            e.fit = *fit;
            e.confidence = std::min({1., double(hist.size()) / cfg.raw_reference, fit->qspace});
          }
          estimates[k] = e;
        }
        calculated[k] = roi.pose.stamp;
      }
      count(counters, "calculated_cells", changed.size());
    }
    if (roi.valid)
    {
      previous_active = std::move(active);
      dirty.clear();
    }
    Eigen::Vector2d heading = (roi.pose.q * forward_axis.normalized()).head<2>();
    require(heading.norm() >= 1e-8||estimates.empty(), "Invalid heading");
    if (heading.norm() > 0)
      heading.normalize();
    Json rows = Json::array();
    int64_t capture = 0;
    for (auto& [k, e]:estimates)
    {
      double angle = nan, tilt = nan;
      if (e.valid)
      {
        auto n = e.fit.normal;
        tilt = float(e.fit.tilt);
        if (std::abs(n.z()) > 1e-8)
        {
          angle = std::atan2(-n.head<2>().dot(heading), n.z()) * 180 / pi;
          if (std::abs(angle) < 1e-10)
            angle = 0;
          angle = float(angle);
        }
      }
      int64_t observed = last_observed[k];
      capture = std::max(capture, observed);
      rows.push_back({k.first, k.second, num(angle), num(float(e.height)), num(float(e.confidence)), e.reused, double(calculated[k]) / 1e9, observed, num(tilt)});
    }
    std::sort(rows.begin(), rows.end(), [](const Json& a, const Json& b){
      return std::make_pair(a[1].get<int64_t>(), a[0].get<int64_t>()) < std::make_pair(b[1].get<int64_t>(), b[0].get<int64_t>());
    });
    return {{"modes", {{"fused", {{"resolution_m", cfg.resolution}, {"columns", {"ix", "iy", "slope_deg", "height_m", "confidence", "slope_reused", "last_calculated_stamp_s", "last_observed_stamp_ns", "slope_magnitude_deg"}}, {"cells", rows}}}}}, {"metadata", {{"frame_id", "odom"}, {"slope_value_semantics", "signed_longitudinal_deg"}}}, {"snapshot_capture_ns", capture}, {"source_capture_ns", source_capture}, {"counters", counters}};
  }
  void SemanticGrid::add(const Frame& f, const std::vector<int>& ids, const std::vector<int64_t>& labels, int64_t camera)
  {
    require(f.points.size() == ids.size()&&ids.size() == labels.size()&&labels.size() == f.stamps.size(), "Invalid semantic observation sizes");
    if (f.points.empty())
      return;
    ++revision;
    std::map<std::tuple<int64_t, int64_t, int>, std::pair<int64_t, int64_t> > grouped;
    for (size_t i = 0; i < ids.size(); ++i)
    {
      require(f.stamps[i] > 0&&labels[i] >= 0, "Invalid semantic timestamps");
      auto k = key_of(f.points[i], cfg.resolution);
      auto& g = grouped[{k.first, k.second, ids[i]}];
      g.first = std::max(g.first, f.stamps[i]);
      g.second = std::max(g.second, labels[i]);
    }
    for (auto& [key, t]:grouped)
    {
      auto [x, y, id] = key;
      auto& cell = cells[{x, y}];
      cell.geometry = std::max(cell.geometry, t.first);
      if (id < 0)
        continue;
      const auto& spec = cfg.semantic.at("classes").at(std::to_string(id));
      if (spec.value("is_ignore", false))
        continue;
      int64_t label = t.second > 0?t.second:camera;
      require(label > 0, "Positive camera stamp required");
      if (cell.captures.empty()||label > cell.captures.back())
      {
        cell.captures.push_back(label);
        cell.captures.erase(cell.captures.begin(), std::lower_bound(cell.captures.begin(), cell.captures.end(), label - 3000000000LL));
        cell.camera = label;
      }
      if(!cell.classes.count(id))cell.class_order.push_back(id);
      auto& e = cell.classes[id];
      int64_t before = e.last;
      if (t.second > e.last)
        ++e.votes[f.source];
      e.last = std::max(e.last, t.second);
      if (spec.at("kind") == "dynamic"&&e.last > before)
        expirations.emplace(e.last + nearest_even(cfg.semantic.at("dynamic_retention_s").get<double>() * 1e9) + 1, Key{x, y}, id, e.last);
    }
  }
  uint64_t SemanticGrid::version(int64_t now)
  {
    while (!expirations.empty()&&std::get<0>(expirations.top()) <= now)
    {
      auto [expiry, key, id, stamp] = expirations.top();
      expirations.pop();
      auto cell = cells.find(key);
      if (cell != cells.end())
      {
        auto evidence = cell->second.classes.find(id);
        if (evidence != cell->second.classes.end()&&evidence->second.last == stamp)
          ++revision;
      }
    }
    return revision;
  }
  Json SemanticGrid::snapshot(int64_t now)const
  {
    Json rows = Json::array();
    auto priority = [](const std::string& kind){
                      return kind == "ground"?0:kind == "other"?1:kind == "obstacle"?2:3;
                    };
    int64_t retain = nearest_even(cfg.semantic.at("dynamic_retention_s").get<double>() * 1e9);
    for (auto& [k, cell]:cells)
    {
      int selected = -1, best = -1;
      int64_t stamp = 0, last_dynamic = 0;
      bool expired = false;
      int64_t newer = 0;
      Json evidence = Json::object();
      for (int id:cell.class_order)
      {
        const auto& e=cell.classes.at(id);
        const auto& spec = cfg.semantic.at("classes").at(std::to_string(id));
        std::string kind = spec.at("kind");
        evidence[std::to_string(id)] = {{"last_ns", e.last}, {"votes", e.votes}};
        if (kind == "dynamic")
        {
          last_dynamic = std::max(last_dynamic, e.last);
          if (now - e.last > retain)
          {
            expired = true;
            continue;
          }
        }
        newer = std::max(newer, e.last);
        int pr = priority(kind);
        if (pr > best||(pr == best&&e.last > stamp))
        {
          selected = id;
          best = pr;
          stamp = e.last;
        }
      }
      if (expired&&newer <= last_dynamic)
      {
        selected = -1;
        stamp = 0;
      }
      Json cost = nullptr, name = nullptr;
      if (selected >= 0)
      {
        const auto& spec = cfg.semantic.at("classes").at(std::to_string(selected));
        cost = spec.at("cost");
        name = spec.at("name");
      }
      rows.push_back({{"ix", k.first}, {"iy", k.second}, {"class_id", selected < 0?Json(nullptr):Json(selected)}, {"class_name", name}, {"cost", cost}, {"last_geometry_ns", cell.geometry}, {"last_semantic_ns", stamp}, {"camera_observed_ns", cell.camera}, {"camera_capture_ns", cell.captures}, {"class_evidence", evidence}});
    }
    return {{"resolution_m", cfg.resolution}, {"frame", "odom"}, {"cells", rows}, {"snapshot_stamp_ns", now}};
  }
  void label_safe(Label& l, int margin)
  {
    require(l.width > 0&&l.height > 0&&l.ids.size() == size_t(l.width) * l.height, "Bad label shape");
    l.safe.assign(l.ids.size(), 1);
    if (!margin)
      return;
    for (int y = 0; y < l.height; ++y)
      for (int x = 0; x < l.width; ++x)
      {
        bool good = x >= margin&&y >= margin&&x < l.width - margin&&y < l.height - margin;
        uint8_t a = l.ids[size_t(y) * l.width + x];
        if (good)
          for (int dy = -margin; dy <= margin&&good; ++dy)
            for (int dx = -margin; dx <= margin; ++dx)
              if (l.ids[size_t(y + dy) * l.width + x + dx] != a)
              {
                good = false;
                break;
              }
        l.safe[size_t(y) * l.width + x] = good;
      }
  }
  std::vector<int> project(const Points& p, const Label& label, const Camera& cam, const Config& cfg)
  {
    require(cam.width > 0&&cam.height > 0&&cam.width * label.height == cam.height * label.width, "Label aspect ratio mismatch");
    require(label.safe.size() == label.ids.size(), "Missing boundary mask");
    std::vector<int> ids(p.size(), -1);
    M3 R = cam.world.block<3, 3>(0, 0);
    V3 t = cam.world.block<3, 1>(0, 3);
    for (size_t i = 0; i < p.size(); ++i)
    {
      V3 q = R.transpose() * (p[i] - t);
      if (!q.allFinite()||q.z() <= 0)
        continue;
      double ux = cam.k(0, 0) * q.x() / q.z() + cam.k(0, 2), vy = cam.k(1, 1) * q.y() / q.z() + cam.k(1, 2);
      if (!std::isfinite(ux)||!std::isfinite(vy)||std::abs(ux) > 1e12||std::abs(vy) > 1e12)
        continue;
      auto u = nearest_even(ux), v = nearest_even(vy);
      if (u < 0||v < 0||u >= cam.width||v >= cam.height)
        continue;
      size_t ix = size_t(v * label.height / cam.height) * label.width + size_t(u * label.width / cam.width);
      int id = label.ids[ix];
      if (label.safe[ix]&&cfg.semantic.at("classes").contains(std::to_string(id))&&!cfg.semantic.at("classes").at(std::to_string(id)).value("is_ignore", false))
        ids[i] = id;
    }
    return ids;
  }
  void LocalGround::submit(Frame f, const Points* camera)
  {
    int s = f.source;
    require((s == 0||s == 1)&&f.stamp > last[s], "reordered local geometry");
    last[s] = f.stamp;
    auto b = f.stamp / nearest_even(cfg.semantic.at("local_ground").at("sample_period_s").get<double>() * 1e9);
    if (b == bucket[s])
      return;
    bucket[s] = b;
    if (s == 1)
    {
      require(camera, "Missing optical XYZ");
      f.points = frame_voxels(*camera, f.points);
      f.stamps.assign(f.points.size(), f.stamp);
    }
    require(f.points.size() == f.stamps.size(), "Invalid point timestamps");
    ROI roi;
    roi.mounts = mounts;
    roi.radius = cfg.semantic.at("local_ground").at("radius_m");
    roi.set_pose(f.pose);
    Frame selected = f;
    selected.points.clear();
    selected.stamps.clear();
    double halo = cfg.semantic.at("local_ground").at("support_halo_m");
    for (size_t i = 0; i < f.points.size(); ++i)
      if (roi.contains(f.points[i].head<2>(), halo))
      {
        selected.points.push_back(f.points[i]);
        selected.stamps.push_back(f.stamps[i]);
      }
    Geometry g;
    g.frame = std::move(selected);
    g.ids.assign(g.frame.points.size(), -1);
    g.label_stamps.assign(g.ids.size(), 0);
    frames[s].push_back(std::move(g));
    size_t max = cfg.semantic.at("local_ground").at("max_frames_per_source");
    while (frames[s].size() > max)
      frames[s].pop_front();
  }
  std::vector<Frame> LocalGround::process(const std::vector<Pose>& rows, const std::deque<Label>& labels, const M4& mount, const M3& k, int width, int height)
  {
    std::vector<Frame> out;
    if (rows.empty())
      return out;
    int64_t now = rows.back().stamp, window = nearest_even(cfg.semantic.at("local_ground").at("retention_s").get<double>() * 1e9);
    for (auto& q:frames)
      while (!q.empty()&&q.front().frame.stamp < now - window)
        q.pop_front();
    const Label* selected = nullptr;
    Camera camera;
    camera.k = k;
    camera.width = width;
    camera.height = height;
    for (auto it = labels.rbegin(); it != labels.rend(); ++it)
    {
      if (it->stamp > now||now - it->stamp > window)
        continue;
      try{
        auto p = interpolate(rows, it->stamp, nearest_even(cfg.semantic.at("max_pose_gap_ms").get<double>() * 1e6));
        camera.world = M4::Identity();
        camera.world.block<3, 3>(0, 0) = p.q.toRotationMatrix() * mount.block<3, 3>(0, 0);
        camera.world.block<3, 1>(0, 3) = p.p + p.q * mount.block<3, 1>(0, 3);
        selected = &*it;
        break;
      }
      catch (const std::exception&){}
    }
    if (!selected)
      return out;
    int64_t lt = selected->stamp;
    bool did = false;
    std::set<int> grounds = cfg.semantic.at("ground_class_ids").get<std::set<int> >();
    int64_t max_dt = nearest_even(cfg.semantic.at("max_point_label_dt_ms").get<double>() * 1e6), dynamic = nearest_even(cfg.semantic.at("dynamic_retention_s").get<double>() * 1e9);
    for (auto& source:frames)
      for (auto& f:source)
      {
        if (f.attempt >= lt)
          continue;
        auto ids = project(f.frame.points, *selected, camera, cfg);
        std::vector<uint8_t> mask(ids.size());
        for (size_t i = 0; i < ids.size(); ++i)
        {
          if (f.frame.stamps[i] < lt - window||f.frame.stamps[i] > lt + max_dt)
            ids[i] = -1;
          if (ids[i] >= 0&&lt >= f.label_stamps[i])
          {
            f.ids[i] = ids[i];
            f.label_stamps[i] = lt;
          }
          if (f.ids[i] >= 0&&cfg.semantic.at("classes").at(std::to_string(f.ids[i])).at("kind") == "dynamic"&&now - f.label_stamps[i] > dynamic)
            f.ids[i] = -1;
          mask[i] = grounds.count(f.ids[i]);
        }
        f.attempt = lt;
        grid.add(f.frame, f.ids, f.label_stamps, lt);
        last_stamp = std::max(last_stamp, f.frame.stamp);
        if (!f.emitted||*f.emitted != mask)
        {
          Frame ground = f.frame;
          ground.points.clear();
          ground.stamps.clear();
          ground.pose = rows.back();
          for (size_t i = 0; i < mask.size(); ++i)
            if (mask[i])
            {
              ground.points.push_back(f.frame.points[i]);
              ground.stamps.push_back(f.frame.stamps[i]);
            }
          out.push_back(std::move(ground));
          f.emitted = mask;
        }
        did = true;
      }
    if (did)
      ++sequence;
    return out;
  }
  Json build_cost(const Json& live, const Json& semantic, const Config& cfg)
  {
    const auto& slope = live.at("modes").at("fused");
    require(slope.at("resolution_m").get<double>() == cfg.resolution&&semantic.at("resolution_m").get<double>() == cfg.resolution, "Grid resolutions differ");
    require(semantic.at("frame") == "odom"&&live.at("metadata").at("frame_id") == "odom", "Frame mismatch");
    std::map<Key, Json> slopes, sems;
    for (auto& r:slope.at("cells"))
      slopes[{r[0].get<int64_t>(), r[1].get<int64_t>()}] = r;
    for (auto& r:semantic.at("cells"))
      sems[{r.at("ix").get<int64_t>(), r.at("iy").get<int64_t>()}] = r;
    std::set<Key> keys;
    for (auto& v:slopes)
      keys.insert(v.first);
    for (auto& v:sems)
      keys.insert(v.first);
    require(!keys.empty(), "No measured cells for cost output");
    int64_t xmin = INT64_MAX, xmax = INT64_MIN, ymin = INT64_MAX, ymax = INT64_MIN;
    for (auto k:keys)
    {
      xmin = std::min(xmin, k.first);
      xmax = std::max(xmax, k.first);
      ymin = std::min(ymin, k.second);
      ymax = std::max(ymax, k.second);
    }
    int64_t w = xmax - xmin + 1, h = ymax - ymin + 1;
    require(w > 0&&h > 0&&w <= 4000000&&h <= 4000000&&w * h <= 4000000, "Cost grid exceeds limit");
    size_t n = size_t(w * h);
    std::vector<double> angles(n, nan), tilts(n, nan), scores(n, nan), terrain(n, nan);
    std::vector<int64_t> st(n), semt(n), camt(n);
    std::vector<std::vector<int64_t> > captures(n);
    int64_t capture = 0;
    for (auto& [k, r]:slopes)
    {
      size_t i = size_t((k.second - ymin) * w + k.first - xmin);
      st[i] = r[7].is_null()?0:r[7].get<int64_t>();
      if (st[i] > 0)
      {
        angles[i] = val(r[2]);
        tilts[i] = val(r[8]);
        capture = std::max(capture, st[i]);
      }
    }
    for (auto& [k, r]:sems)
    {
      size_t i = size_t((k.second - ymin) * w + k.first - xmin);
      semt[i] = r.value("last_semantic_ns", int64_t(0));
      camt[i] = std::max(int64_t(0), r.value("camera_observed_ns", int64_t(0)));
      if (!r.at("cost").is_null()&&semt[i] > 0)
        terrain[i] = float(r.at("cost").get<int>());
      else
        semt[i] = 0;
      if (r.contains("camera_capture_ns"))
        captures[i] = r["camera_capture_ns"].get<std::vector<int64_t> >();
      else if (camt[i] > 0)
        captures[i].push_back(camt[i]);
      capture = std::max(capture, camt[i]);
      if (std::isfinite(terrain[i]))
        capture = std::max(capture, semt[i]);
    }
    require(capture > 0, "Positive capture required");
    Json angle = Json::array(), magnitude = Json::array(), slope_score = Json::array(), semantic_score = Json::array(), slope_cost = Json::array(), terrain_cost = Json::array(), global = Json::array(), local = Json::array(), support = Json::array(), counts = Json::array(), spans = Json::array();
    for (size_t i = 0; i < n; ++i)
    {
      double a = angles[i], t = tilts[i];
      float score = std::numeric_limits<float>::quiet_NaN();
      if (std::isfinite(a)&&std::isfinite(t))
      {
        if (a >= -10&&a <= 0)
          score = 0;
        if (a > 0)
          score = float(4 * t);
        if (a < -10)
          score = 100;
      }
      if (std::isfinite(t)&&t >= 20)
        score = 100;
      if (st[i] <= 0)
        score = std::numeric_limits<float>::quiet_NaN();
      bool blocked = score == 100||terrain[i] == 100;
      bool known = std::isfinite(score)&&std::isfinite(terrain[i]);
      int occ = -1;
      if (known)
      {
        float v = 79.f * (.7f * score / 100.f + .3f * float(terrain[i]) / 100.f);
        occ = std::min<int64_t>(79, nearest_even(v));
      }
      if (blocked)
        occ = 100;
      int lc = known&&(double(capture - st[i]) / 1e9 <= 30.)&&(double(capture - semt[i]) / 1e9 <= 30.)?occ:-1;
      if (blocked)
        lc = 100;
      auto ts = captures[i];
      ts.erase(std::remove_if(ts.begin(), ts.end(), [&](int64_t v){
        return v <= 0||v > capture + 1000;
      }), ts.end());
      std::sort(ts.begin(), ts.end());
      ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
      int sup = 0;
      int64_t span = 0;
      if (!ts.empty())
      {
        auto last = ts.back();
        ts.erase(ts.begin(), std::lower_bound(ts.begin(), ts.end(), last - 3000001000LL));
        span = last - ts.front();
        double strength = std::min(ts.size() / 3., 1.) * std::min(span / 500000000., 1.);
        double age = std::max(int64_t(0), capture - last) / 1e9;
        double fresh = age <= 10?1:std::max(0., (30 - age) / 20.);
        sup = int(nearest_even(100 * strength * fresh));
      }
      angle.push_back(num(float(a)));
      magnitude.push_back(num(float(t)));
      slope_score.push_back(num(score));
      semantic_score.push_back(num(float(terrain[i])));
      slope_cost.push_back(num(float(score / 100.f)));
      terrain_cost.push_back(num(float(float(terrain[i]) / 100.f)));
      global.push_back(occ);
      local.push_back(lc);
      support.push_back(sup);
      counts.push_back(ts.size());
      spans.push_back(span);
    }
    return {{"frame", "odom"}, {"capture_ns", capture}, {"resolution_m", cfg.resolution}, {"origin_xy", {xmin*cfg.resolution, ymin*cfg.resolution}}, {"width", w}, {"height", h}, {"slope_deg", angle}, {"slope_magnitude_deg", magnitude}, {"slope_score", slope_score}, {"semantic_score", semantic_score}, {"slope_cost", slope_cost}, {"terrain_cost", terrain_cost}, {"global_costmap", global}, {"local_costmap", local}, {"camera_support", support}, {"camera_capture_count", counts}, {"camera_capture_span_ns", spans}, {"slope_observed_ns", st}, {"semantic_observed_ns", semt}, {"camera_observed_ns", camt}};
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
  Json scan_pair(const Json& scan, const Json& cloud, const M4& cl)
  {
    int64_t stamp = scan.at("stamp_ns");
    require(cloud.at("stamp_ns") == stamp, "Scan/cloud stamps differ");
    auto p = points_from_json(cloud.at("points"));
    auto times = cloud.at("offsets").get<std::vector<double> >();
    require(p.size() >= 2&&p.size() == times.size(), "Bad point/time buffers");
    double previous = -1;
    for (auto t:times)
    {
      require(std::isfinite(t)&&t >= 0&&t <= .3&&t > previous, "Invalid SDK acquisition time order");
      previous = t;
    }
    double amin = scan.at("angle_min"), inc = scan.at("angle_increment"), lo = scan.at("range_min"), hi = scan.at("range_max");
    require(inc > 0, "Expected increasing scan bins");
    const auto& ranges = scan.at("ranges");
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
    Points points;
    std::vector<double> offsets;
    std::vector<int64_t> bins;
    size_t matched = 0, rear = 0;
    require(std::hypot(cl(2, 0), cl(2, 1)) >= 1e-6, "Camera forward perpendicular to scan plane");
    for (auto i:indices)
    {
      int64_t bin = int64_t(std::ceil((std::atan2(p[i].y(), p[i].x()) - amin) / inc));
      if (ranges[bin].is_null())
        continue;
      double rr = ranges[bin].get<double>();
      if (!std::isfinite(rr)||rr < lo||rr > hi||std::abs(rr - p[i].head<2>().norm()) >= .003)
        continue;
      ++matched;
      double angle = amin + bin * inc;
      V3 local(rr * std::cos(angle), rr * std::sin(angle), 0);
      if (cl.block<1, 3>(2, 0).dot(local) < -1e-10)
      {
        ++rear;
        continue;
      }
      points.push_back(local);
      offsets.push_back(times[i]);
      bins.push_back(bin);
    }
    require(points.size() >= 5, "Too few forward matched points");
    return {{"stamp_ns", stamp}, {"points_lidar_m", points_json(points)}, {"point_time_offsets_s", offsets}, {"bin_indices", bins}, {"input_points", p.size()}, {"matched_bins", matched}, {"rear_discarded", rear}, {"unmatched_points", p.size() - matched}};
  }
  Frame deskew(const Json& pair, const std::vector<Pose>& poses, const M4& il, double offset)
  {
    Frame f;
    f.stamp = pair.at("stamp_ns");
    f.source = 0;
    auto p = points_from_json(pair.at("points_lidar_m"));
    auto times = pair.at("point_time_offsets_s").get<std::vector<double> >();
    require(p.size() == times.size(), "LiDAR offsets size mismatch");
    for (size_t i = 0; i < p.size(); ++i)
    {
      int64_t ns = f.stamp + nearest_even((times[i] + offset) * 1e9);
      auto pose = interpolate(poses, ns);
      V3 q = il.block<3, 3>(0, 0) * p[i] + il.block<3, 1>(0, 3);
      f.points.push_back(pose.q * q + pose.p);
      f.stamps.push_back(ns);
    }
    if (!poses.empty())
      f.pose = poses.back();
    return f;
  }
}

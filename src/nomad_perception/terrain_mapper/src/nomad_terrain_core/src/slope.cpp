#include "nomad_costmap/detail.hpp"
#include "nomad_costmap/cell_executor.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <numeric>

namespace nomad {
  Ground::Ground(const Config& c, const Mounts& m) : cfg(c)
  {
    roi.mounts = m;
    roi.radius = c.forward_radius;
  }
  void Ground::seed_initial_flat_slopes(const std::set<Key>& keys, int64_t stamp)
  {
    if (!cfg.startup_freeze_initial_slope) return;
    require(stamp > 0&&!keys.empty(), "Invalid initial flat-slope seed");
    if (initial_flat_stamp > 0) {
      require(initial_flat_stamp == stamp&&initial_flat_slopes == keys, "Initial flat slope seed changed");
      return;
    }
    require(estimates.empty(), "Initial flat slopes must precede real slope processing");
    initial_flat_slopes = keys;
    initial_flat_stamp = stamp;
    for (auto key:keys) {
      Estimate e;
      e.valid = e.startup_seeded = true;
      e.fit.normal = V3::UnitZ();
      e.fit.tilt = 0.;
      // Normal only: no physical ground elevation or measured support is claimed.
      e.fit.offset = nan;
      estimates[key] = e;
      calculated[key] = stamp;
      last_observed[key] = 0;
    }
    count(counters,"startup_flat_slopes_initialized",keys.size());
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
  bool Ground::slope_frozen(const Estimate& estimate, const Eigen::Vector2d& heading)const
  {
    if (estimate.valid&&estimate.startup_seeded) return cfg.startup_seed_frozen;
    if (!cfg.freeze_slope_after_first_valid||!estimate.valid)
      return false;
    if (!cfg.retry_slope_score_at_least_100)
      return true;
    const auto angles = slope_angles(estimate, heading, cfg.gravity_slope_angle);
    const float score = slope_score_of(angles.first, angles.second, cfg.slope_score_per_deg, cfg.slope_block_deg);
    return std::isfinite(score)&&score < 100;
  }
  std::set<Key> Ground::filter_oak_against_lidar(const Eigen::Vector2d& heading)
  {
    std::set<Key> empty_after_filter;
    if (!cfg.oak_lidar_height_gate)
      return empty_after_filter;
    struct Reference {
      const V3* point;
      int64_t stamp;
    };
    // Both sources are already pose-compensated ground points in odom. Index
    // only retained LiDAR captures; never use OAK or cached cell heights as truth.
    // Slightly wider buckets preserve the circular boundary after float32 XY
    // quantization. The acceptance tolerance below still uses each point pair.
    const double float_epsilon = 2. * std::numeric_limits<float>::epsilon();
    const double bucket_width = cfg.oak_lidar_xy_radius +
      float_epsilon * std::max(1., cfg.max_size + cfg.resolution);
    std::map<Key, std::vector<Reference> > index;
    for (const auto& entry:samples)
      for (const auto& capture:entry.second.sources[0])
        for (const auto& point:capture.points)
          index[key_of(point, bucket_width)].push_back({&point, capture.stamp});
    uint64_t examined = 0, matched = 0, rejected = 0;
    for (auto it = samples.begin(); it != samples.end();)
    {
      const Key cell_key = it->first;
      auto& cell = it->second;
      bool changed = false;
      for (auto& capture:cell.sources[1])
      {
        auto remove = std::remove_if(capture.points.begin(), capture.points.end(), [&](const V3& point){
          ++examined;
          const Key bucket = key_of(point, bucket_width);
          const Reference* nearest = nullptr;
          double nearest2 = INFINITY;
          int64_t nearest_dt = INT64_MAX;
          for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
            {
              auto found = index.find({bucket.first + dx, bucket.second + dy});
              if (found == index.end())
                continue;
              for (const auto& reference:found->second)
              {
                const double d2 = (point.head<2>() - reference.point->head<2>()).squaredNorm();
                const double xy_tolerance = float_epsilon * std::max({1.,
                  point.head<2>().cwiseAbs().maxCoeff(), reference.point->head<2>().cwiseAbs().maxCoeff()});
                const double radius = cfg.oak_lidar_xy_radius + xy_tolerance;
                const int64_t dt = capture.stamp >= reference.stamp?
                  capture.stamp - reference.stamp:reference.stamp - capture.stamp;
                if (d2 <= radius * radius&&
                    (d2 < nearest2||(d2 == nearest2&&dt < nearest_dt)))
                {
                  nearest = &reference;
                  nearest2 = d2;
                  nearest_dt = dt;
                }
              }
            }
          // A missing LiDAR reference is not evidence that an OAK point is bad.
          if (!nearest)
            return false;
          ++matched;
          const double difference = std::abs(point.z() - nearest->point->z());
          // Ingest stores float32 coordinates. Include the 12cm boundary despite
          // float32 rounding; this tolerance is far below a millimetre here.
          const double tolerance = float_epsilon *
            std::max({1., std::abs(point.z()), std::abs(nearest->point->z())});
          if (difference + tolerance < cfg.oak_lidar_z_reject)
            return false;
          ++rejected;
          changed = true;
          return true;
        });
        capture.points.erase(remove, capture.points.end());
      }
      auto& oak = cell.sources[1];
      oak.erase(std::remove_if(oak.begin(), oak.end(), [](const Capture& capture){
        return capture.points.empty();
      }), oak.end());
      if (changed)
      {
        dirty.insert(cell_key);
        // Rebuild a height previously influenced by removed OAK points before
        // any neighbouring refit can consume it. LiDAR samples are untouched.
        std::vector<double> medians;
        for (const auto& source:cell.sources)
        {
          std::vector<double> zs;
          for (const auto& capture:source)
            for (const auto& point:capture.points)
              zs.push_back(point.z());
          if (!zs.empty())
            medians.push_back(median(std::move(zs)));
        }
        if (medians.empty())
        {
          heights.erase(cell_key);
          empty_after_filter.insert(cell_key);
          // Retain only planes protected by the selected freeze policy, also
          // outside the update ROI. High-cost planes may be invalidated normally.
          auto estimate = estimates.find(cell_key);
          if (estimate != estimates.end()) {
            if (slope_frozen(estimate->second, heading))
              count(counters, "frozen_slopes_kept_after_oak_rejection");
            else
              estimate->second = Estimate{};
          }
        }
        else
          heights[cell_key] = {std::accumulate(medians.begin(), medians.end(), 0.) / medians.size(), latest};
      }
      if (cell.sources[0].empty()&&cell.sources[1].empty())
        it = samples.erase(it);
      else
        ++it;
    }
    insertion.erase(std::remove_if(insertion.begin(), insertion.end(), [this](Key key){
      return !samples.count(key);
    }), insertion.end());
    count(counters, "oak_lidar_height_rejected_points", rejected);
    counters["oak_lidar_height_gate_last_snapshot"] = {
      {"examined_oak_points", examined}, {"matched_oak_points", matched},
      {"unmatched_kept_oak_points", examined - matched}, {"rejected_oak_points", rejected},
      {"xy_radius_m", cfg.oak_lidar_xy_radius}, {"reject_abs_z_difference_m", cfg.oak_lidar_z_reject}};
    return empty_after_filter;
  }
  std::vector<Ground::Rep> Ground::prepare()const
  {
    std::vector<Rep> out;
    struct Bin {
      V3 sum = V3::Zero();
      int n = 0;
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
          }
      }
      for (auto& [k, b]:bins)
        out.push_back({b.sum / double(b.n), k, b.n});
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
    enum class Failure { none, support, quality };
    struct Attempt {
      std::optional<Fit> fit;
      Failure failure;
    };
    auto attempt = [&](int half_width) -> Attempt
    {
      std::set<Key> candidates;
      for (int dx = -half_width; dx <= half_width; ++dx)
        for (int dy = -half_width; dy <= half_width; ++dy)
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
            return {{}, Failure::quality};
          if (connected.insert(k).second)
            pending.push_back(k);
        }
      }
      connected.erase(target);
      if (connected.size() < size_t(cfg.min_heights))
        return {{}, Failure::support};
      Points p;
      for (auto k:connected)
      {
        auto xy = center(k, cfg.resolution);
        p.push_back({xy.x(), xy.y(), heights.at(k).first});
      }
      auto xy = center(target, cfg.resolution);
      if (!spread(p, cfg)||!contains_target(p, xy))
        return {{}, Failure::support};
      auto fitted = fit_tls(p, {}, cfg, false);
      if (!fitted||fitted->residual > cfg.rms||std::abs(fitted->normal.z()) < 1e-8)
        return {{}, Failure::quality};
      V3 v(xy.x(), xy.y(), h->second.first);
      if (std::abs(fitted->normal.dot(v) + fitted->offset) / std::abs(fitted->normal.z()) > cfg.target_error)
        return {{}, Failure::quality};
      return {std::move(fitted), Failure::none};
    };
    auto local = attempt(1);
    if (local.fit||local.failure != Failure::support)
      return local.fit;
    return attempt(2).fit;
  }
  Points Ground::update_height(Key k)
  {
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
    if (!medians.empty())
      heights[k] = {std::accumulate(medians.begin(), medians.end(), 0.) / medians.size(), latest};
    return central;
  }
  void Ground::prune(const MapWindow& w)
  {
    if (!w.active()) return;
    // Cell cap (2026-11-14): the measured cells inside the window, least recently observed first.
    std::set<Key> known;
    if (w.max_cells && samples.size() + estimates.size() + heights.size() > w.max_cells)
    {
      for (auto& kv:samples) if (w.contains(kv.first)) known.insert(kv.first);
      for (auto& kv:estimates) if (w.contains(kv.first)) known.insert(kv.first);
      for (auto& kv:heights) if (w.contains(kv.first)) known.insert(kv.first);
    }
    const std::set<Key> evicted = cells_over_cap(known, w, [&](Key k) {
      auto it = last_observed.find(k);
      if (it != last_observed.end()) return it->second;
      auto e = estimates.find(k);
      return e != estimates.end() ? e->second.last_observed : int64_t(0);
    });
    if (!evicted.empty()) count(counters, "cap_evicted_cells", evicted.size());
    auto outside = [&](Key k){ return !w.contains(k) || evicted.count(k) > 0; };
    for (auto it = samples.begin(); it != samples.end();) it = outside(it->first)?samples.erase(it):std::next(it);
    insertion.erase(std::remove_if(insertion.begin(), insertion.end(), outside), insertion.end());
    for (auto* keys:{&observed, &dirty, &previous_active})
      for (auto it = keys->begin(); it != keys->end();) it = outside(*it)?keys->erase(it):std::next(it);
    for (auto it = estimates.begin(); it != estimates.end();) it = outside(it->first)?estimates.erase(it):std::next(it);
    for (auto it = heights.begin(); it != heights.end();) it = outside(it->first)?heights.erase(it):std::next(it);
    for (auto* times:{&last_observed, &calculated})
      for (auto it = times->begin(); it != times->end();) it = outside(it->first)?times->erase(it):std::next(it);
  }
  SlopeSnapshot Ground::snapshot()
  {
    using Clock = std::chrono::steady_clock;
    auto elapsed = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double,std::milli>(b-a).count();
    };
    const auto snapshot_started = Clock::now();
    double prepare_ms=0, direct_ms=0, ordered_ms=0;
    size_t direct_tasks=0, threads_used=1;
    Eigen::Vector2d heading = (roi.pose.q * forward_axis.normalized()).head<2>();
    const double heading_norm = heading.norm();
    if (heading_norm > 0)
      heading.normalize();
    const auto empty_after_filter = filter_oak_against_lidar(heading);
    const auto filter_finished = Clock::now();
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
      size_t frozen_skipped = 0;
      const auto prepare_started = Clock::now();
      std::vector<Key> pending_keys;
      for (auto key:changed) {
        auto old=estimates.find(key);
        if (old == estimates.end() || !slope_frozen(old->second,heading))
          pending_keys.push_back(key);
      }
      std::vector<Rep> reps;
      std::map<Key, std::vector<size_t>> bins;
      if (!pending_keys.empty()) {
        reps=prepare();
        for (size_t i=0;i<reps.size();++i) bins[reps[i].bin].push_back(i);
      }
      struct DirectResult {
        int raw=0;
        size_t bin_count=0;
        std::optional<Fit> fit;
      };
      std::vector<DirectResult> direct_results(pending_keys.size());
      direct_tasks=pending_keys.size();
      const auto prepare_finished = Clock::now();
      const auto& readonly_bins=bins;
      auto direct_cell = [&](size_t index) {
        const Key k=pending_keys[index];
        auto xy = center(k, cfg.resolution);
        int rr = int(std::ceil((cfg.radius + 1e-9) / cfg.bin)) + 1;
        Key bc{int64_t(std::floor(xy.x() / cfg.bin)), int64_t(std::floor(xy.y() / cfg.bin))};
        std::vector<size_t> neighbors;
        for (int dx = -rr; dx <= rr; ++dx)
          for (int dy = -rr; dy <= rr; ++dy)
          {
            auto it = readonly_bins.find({bc.first + dx, bc.second + dy});
            if (it != readonly_bins.end())
              for (auto i:it->second)
                if ((reps[i].mean.head<2>() - xy).squaredNorm() <= std::pow(cfg.radius + 1e-9, 2))
                  neighbors.push_back(i);
          }
        std::sort(neighbors.begin(), neighbors.end());
        int raw = 0;
        std::map<Key, int> hist;
        Points p;
        for (auto i:neighbors)
        {
          raw += reps[i].count;
          ++hist[reps[i].bin];
          p.push_back(reps[i].mean);
        }
        std::optional<Fit> fit;
        if (raw >= cfg.min_raw&&hist.size() >= size_t(cfg.min_bins))
        {
          std::vector<double> w;
          for (auto i:neighbors)
            w.push_back(1. / hist[reps[i].bin]);
          fit = fit_tls(p, w, cfg, true);
        }
        direct_results[index]={raw,hist.size(),std::move(fit)};
      };
      if (cfg.slope_compute_threads > 1 &&
          pending_keys.size() >= size_t(cfg.slope_parallel_min_cells)) {
        if (!cell_executor || cell_executor->threads() != size_t(cfg.slope_compute_threads))
          cell_executor=std::make_shared<CellExecutor>(size_t(cfg.slope_compute_threads));
        threads_used=cell_executor->threads();
        cell_executor->run(pending_keys.size(),direct_cell);
      } else {
        for (size_t i=0;i<pending_keys.size();++i) direct_cell(i);
      }
      const auto direct_finished = Clock::now();
      size_t direct_index=0;
      for (auto k:changed)
      {
        auto accepted = estimates.find(k);
        if (accepted != estimates.end()&&slope_frozen(accepted->second, heading))
        {
          update_height(k);
          if (accepted->second.startup_seeded&&heights.count(k))
            accepted->second.height = heights.at(k).first;
          ++frozen_skipped;
          continue;
        }
        if (cfg.freeze_slope_after_first_valid&&cfg.retry_slope_score_at_least_100&&
            accepted != estimates.end()&&accepted->second.valid)
          count(counters, "high_cost_slope_recalculation_attempts");
        auto& direct_result=direct_results.at(direct_index++);
        int raw=direct_result.raw;
        const auto bin_count=direct_result.bin_count;
        auto& fit=direct_result.fit;
        if (fit) count(counters,"direct_tls_success");
        Points central = update_height(k);
        if (!fit)
        {
          count(counters, "height_refit_attempts");
          fit = refit(k);
          if (fit)
            count(counters, "height_refit_success");
        }
        auto old = estimates.find(k);
        bool reused = false;
        if (!fit&&old != estimates.end()&&old->second.valid&&!empty_after_filter.count(k))
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
        // 2026-11-14 keep_slope_on_failure: a failed recalculation never replaces a valid
        // plane (the same exclusion as above: cells whose OAK points the LiDAR gate removed).
        if (!reused&&!fit&&cfg.keep_slope_on_failure&&old != estimates.end()&&old->second.valid&&!empty_after_filter.count(k))
        {
          old->second.reused = true;
          reused = true;
          count(counters, "previous_slope_kept_on_failure");
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
            e.confidence = std::min({1., double(bin_count) / cfg.raw_reference, fit->qspace});
          }
          estimates[k] = e;
        }
        calculated[k] = roi.pose.stamp;
      }
      const auto ordered_finished = Clock::now();
      prepare_ms=elapsed(prepare_started,prepare_finished);
      direct_ms=elapsed(prepare_finished,direct_finished);
      ordered_ms=elapsed(direct_finished,ordered_finished);
      count(counters, "calculated_cells", changed.size() - frozen_skipped);
      if (frozen_skipped)
        count(counters, "frozen_slope_recalculations_skipped", frozen_skipped);
    }
    if (roi.valid)
    {
      previous_active = std::move(active);
      dirty.clear();
    }
    require(heading_norm >= 1e-8||estimates.empty(), "Invalid heading");
    const auto output_started = Clock::now();
    SlopeSnapshot result;
    int64_t capture = 0;
    result.rows.reserve(estimates.size());
    for (auto& [k, e]:estimates)
    {
      const auto [angle, tilt] = slope_angles(e, heading, cfg.gravity_slope_angle);
      int64_t observed = last_observed[k];
      capture = std::max(capture, observed);
      SlopeRow row;
      row.ix = k.first; row.iy = k.second;
      // Former JSON: num() -> null for nonfinite; build_cost read null as NaN.
      row.angle = angle; row.height = float(e.height); row.confidence = float(e.confidence);
      row.reused = e.reused; row.calculated_s = double(calculated[k]) / 1e9;
      row.observed = observed; row.tilt = tilt;
      result.rows.push_back(row);
    }
    std::sort(result.rows.begin(), result.rows.end(), [](const SlopeRow& a, const SlopeRow& b){
      return std::make_pair(a.iy, a.ix) < std::make_pair(b.iy, b.ix);
    });
    if (initial_flat_stamp > 0) {
      result.startup_frozen_cells.assign(initial_flat_slopes.begin(), initial_flat_slopes.end());
      result.startup_frozen_stamp = initial_flat_stamp;
    }
    result.capture = capture;
    result.source_capture = source_capture;
    result.counters = counters;
    const auto snapshot_finished=Clock::now();
    slope_timing={{"requested_threads",cfg.slope_compute_threads},
      {"direct_threads_available",threads_used},{"direct_cells",direct_tasks},
      {"filter_ms",elapsed(snapshot_started,filter_finished)},
      {"prepare_ms",prepare_ms},{"direct_tls_ms",direct_ms},
      {"ordered_update_ms",ordered_ms},{"output_ms",elapsed(output_started,snapshot_finished)},
      {"snapshot_ms",elapsed(snapshot_started,snapshot_finished)},
      {"parallel_scope","read_only_neighbor_search_and_direct_tls"}};
    return result;
  }
}

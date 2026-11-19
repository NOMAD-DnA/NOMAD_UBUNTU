#include "nomad_costmap/detail.hpp"
#include <algorithm>
#include <numeric>

namespace nomad {
  bool SemanticGrid::accepts(Key key, int id, int64_t geometry, bool reprojected) const
  {
    if (!cfg.person_ray_restore || id < 0) return true;
    const auto it = cells.find(key);
    if (it == cells.end()) return true;
    const auto& cell = it->second;
    // Replayed body geometry must not recreate the person or relabel its old
    // position as background after a later actual LiDAR ray cleared it.
    if (cell.person_clear > 0 && geometry <= cell.person_clear) return false;
    if (unresolved_people.count(key)) {
      if (reprojected) return false;
      if (id != cfg.person_class_id && geometry <= cell.person_geometry) return false;
    }
    return true;
  }

  std::vector<Key> SemanticGrid::clear_person_by_lidar(const Frame& f, const M4& imu_lidar)
  {
    std::vector<Key> cleared;
    if (!cfg.person_ray_restore || f.source != 0 || unresolved_people.empty()) return cleared;
    require(f.points.size() == f.stamps.size(), "Invalid LiDAR clearing timestamps");
    require(f.ray_origins.empty() || f.ray_origins.size() == f.points.size(),
            "Invalid per-return LiDAR origins");
    const V3 fallback = f.pose.p + f.pose.q * imu_lidar.block<3,1>(0,3);
    int64_t capture = f.stamp;
    for (auto stamp:f.stamps) capture = std::max(capture,stamp);
    // Query sparse occupied person cells only. Slab intersection is strictly
    // in XY: one ray through a cell before its measured endpoint is enough.
    // There is deliberately no Z test, multi-ray vote or ground-plane test.
    for (size_t i=0;i<f.points.size() && !unresolved_people.empty();++i) {
      const auto& origin = f.ray_origins.empty()?fallback:f.ray_origins[i];
      const auto& hit = f.points[i];
      if (!origin.allFinite() || !hit.allFinite() || f.stamps[i] <= 0) continue;
      const Eigen::Vector2d a = origin.head<2>(), delta = hit.head<2>()-a;
      if (delta.squaredNorm() <= 1e-20) continue;
      const Key endpoint = key_of(hit,cfg.resolution);
      for (auto it=unresolved_people.begin();it!=unresolved_people.end();) {
        const Key key = it->first;
        if (key == endpoint || f.stamps[i] <= it->second) { ++it; continue; }
        double enter = 0., leave = 1.;
        bool intersects = true;
        for (int axis=0;axis<2;++axis) {
          const double lower = double(axis?key.second:key.first)*cfg.resolution;
          const double upper = lower+cfg.resolution;
          if (std::abs(delta[axis]) <= 1e-15) {
            if (a[axis] < lower || a[axis] >= upper) { intersects=false; break; }
          } else {
            double near = (lower-a[axis])/delta[axis], far = (upper-a[axis])/delta[axis];
            if (near > far) std::swap(near,far);
            enter = std::max(enter,near);
            leave = std::min(leave,far);
            if (leave <= enter) { intersects=false; break; }
          }
        }
        if (!intersects || leave >= 1.-1e-12 || leave <= 0.) { ++it; continue; }
        auto& cell = cells.at(key);
        // Clear through this scan's latest actual acquisition time so other
        // old returns from the same scan cannot immediately undo the restore.
        cell.person_clear = std::max(cell.person_clear,capture);
        cell.person_clear_source = "lidar_xy_ray_pass";
        cleared.push_back(key);
        it = unresolved_people.erase(it);
        ++revision;
      }
    }
    return cleared;
  }

  void SemanticGrid::seed_startup_person_cells(const std::set<Key>& keys)
  {
    if (!cfg.startup_person_cost_overlay) return;
    require(!keys.empty(), "Startup person cost footprint is empty");
    require(startup_person_cells.empty() || startup_person_cells == keys,
            "Startup person cost footprint changed");
    startup_person_cells = keys;
  }

  void SemanticGrid::add(const Frame& f, const std::vector<int>& ids, const std::vector<int64_t>& labels, int64_t camera, bool reprojected)
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
      if (!accepts(k,ids[i],f.stamps[i],reprojected)) continue;
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
      const auto& spec = cfg.class_info(id);
      if (spec.ignore)
        continue;
      if (cfg.person_ray_restore) {
        if (id == cfg.person_class_id && t.second >= cell.person_semantic) {
          cell.person_geometry = std::max(cell.person_geometry,t.first);
          cell.person_semantic = t.second;
          unresolved_people[{x,y}] = std::max(cell.person_geometry,cell.person_semantic);
        } else if (!spec.dynamic && unresolved_people.count({x,y}) &&
                   !(cfg.startup_person_cost_overlay && startup_person_cells.count({x,y})) &&
                   t.first > cell.person_geometry && t.second >= cell.person_semantic) {
          unresolved_people.erase({x,y});
          cell.person_clear = std::max(cell.person_clear,t.first);
          cell.person_clear_source = "new_nonperson_geometry";
        }
      }
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
      if (t.second > e.last || (t.second == e.last && !reprojected))
        e.reprojected = reprojected;
      e.last = std::max(e.last, t.second);
      if (spec.dynamic&&e.last > before)
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
  void SemanticGrid::prune(const MapWindow& w)
  {
    if (!w.active()) return;
    for (auto it = cells.begin(); it != cells.end();) it = w.contains(it->first)?std::next(it):cells.erase(it);
    // Cell cap (2026-11-14): least recently observed (geometry or camera) first.
    std::vector<Key> keys;
    keys.reserve(cells.size());
    for (auto& kv:cells) keys.push_back(kv.first);
    for (auto& k:cells_over_cap(keys, w, [&](Key k) { auto& c = cells.at(k); return std::max(c.geometry, c.camera); }))
      cells.erase(k);
    for (auto it = unresolved_people.begin(); it != unresolved_people.end();)
      it = w.contains(it->first)?std::next(it):unresolved_people.erase(it);
  }
  void LocalGround::prune(const MapWindow& w)
  {
    if (!w.active()) return;
    grid.prune(w);
    for (auto it = semantic_anchors.begin(); it != semantic_anchors.end();)
      it = w.contains(it->first)?std::next(it):semantic_anchors.erase(it);
    std::vector<Key> keys;
    keys.reserve(semantic_anchors.size());
    for (auto& kv:semantic_anchors) keys.push_back(kv.first);
    for (auto& k:cells_over_cap(keys, w, [&](Key k) {
           int64_t t = 0;
           for (auto& a:semantic_anchors.at(k)) if (a) t = std::max(t, a->geometry_stamp);
           return t; }))
      semantic_anchors.erase(k);
  }
  SemanticSnapshot SemanticGrid::snapshot(int64_t now)const
  {
    SemanticSnapshot result;
    result.stamp = now;
    auto& rows = result.rows;
    rows.reserve(cells.size());
    auto priority = [](const std::string& kind){
                      return kind == "ground"?0:kind == "other"?1:kind == "obstacle"?2:3;
                    };
    int64_t retain = nearest_even(cfg.semantic.at("dynamic_retention_s").get<double>() * 1e9);
    for (auto& [k, cell]:cells)
    {
      int selected = -1, best = -1, best_direct = -1;
      int64_t stamp = 0, last_dynamic = 0;
      bool expired = false;
      int64_t newer = 0;
      int base_id = -1, base_priority = -1, base_direct = -1;
      (void)base_id;  // base class id was only in the dropped JSON row
      int64_t base_stamp = 0;
      for (int id:cell.class_order)
      {
        const auto& e=cell.classes.at(id);
        const std::string& kind = cfg.class_info(id).kind;
        if (cfg.person_ray_restore && kind != "dynamic") {
          const int pr = priority(kind), direct = e.reprojected?0:1;
          if (e.last > base_stamp || (e.last == base_stamp &&
              (direct > base_direct || (direct == base_direct && pr > base_priority)))) {
            base_id = id; base_stamp = e.last; base_priority = pr; base_direct = direct;
          }
        }
        if (cfg.person_ray_restore && id == cfg.person_class_id &&
            cell.person_geometry > 0 && cell.person_geometry <= cell.person_clear) continue;
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
        int direct = e.reprojected?0:1;
        // A newer semantic capture replaces the old class, including an old
        // obstacle. Current real geometry wins a same-capture replay tie;
        // otherwise keep the existing class-priority rule.
        if (e.last > stamp||(e.last == stamp&&
            (direct > best_direct||(direct == best_direct&&pr > best))))
        {
          selected = id;
          best = pr;
          best_direct = direct;
          stamp = e.last;
        }
      }
      if (expired&&newer <= last_dynamic)
      {
        selected = -1;
        stamp = 0;
      }
      const bool startup_person_pending = cfg.startup_person_cost_overlay &&
        startup_person_cells.count(k) && unresolved_people.count(k);
      if (startup_person_pending) {
        // A timeout or new background label cannot establish free space here.
        // Hold occupancy using its original capture time until actual ray clearance.
        selected = cfg.person_class_id;
        stamp = cell.person_semantic;
      }
      SemanticRow row;
      row.ix = k.first; row.iy = k.second;
      if (selected >= 0)
      {
        row.class_id = selected;
        row.cost = cfg.class_info(selected).cost;
      }
      row.last_geometry_ns = cell.geometry;
      row.last_semantic_ns = stamp;
      row.camera_observed_ns = cell.camera;
      row.camera_capture_ns = cell.captures;
      if (cfg.person_ray_restore) {
        row.has_person = true;
        row.base_last_semantic_ns = base_stamp;
        row.person_unresolved = unresolved_people.count(k)!=0;
        row.person_last_geometry_ns = cell.person_geometry;
        row.person_clear_source = cell.person_clear_source;
        row.person_clear_observed_ns = cell.person_clear;
      }
      rows.push_back(std::move(row));
    }
    return result;
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
      if (label.safe[ix]&&id >= 0&&id <= 255&&cfg.classes[size_t(id)].known&&!cfg.classes[size_t(id)].ignore)
        ids[i] = id;
    }
    return ids;
  }
  void LocalGround::submit(Frame f, const Points* camera)
  {
    int s = f.source;
    require((s == 0||s == 1)&&f.stamp > last[s], "reordered local geometry");
    last[s] = f.stamp;
    const auto cleared = grid.clear_person_by_lidar(f,mounts.sensor[0]);
    if (!cleared.empty()) {
      for (auto key:cleared) semantic_anchors.erase(key);
      ++sequence;
    }
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
    selected.ray_origins.clear();
    double halo = cfg.semantic.at("local_ground").at("support_halo_m");
    for (size_t i = 0; i < f.points.size(); ++i)
      if (roi.contains(f.points[i].head<2>(), halo))
      {
        selected.points.push_back(f.points[i]);
        selected.stamps.push_back(f.stamps[i]);
        const Key key = key_of(f.points[i], cfg.resolution);
        if (cfg.person_ray_restore && grid.unresolved_people.count(key)) continue;
        const auto middle = center(key, cfg.resolution);
        const size_t slot = size_t(s)*4 +
          (f.points[i].x() >= middle.x()?1:0) +
          (f.points[i].y() >= middle.y()?2:0);
        auto& anchor = semantic_anchors[key][slot];
        // Keep an observed point, not a fabricated flat cell center. Prefer
        // the newest capture; tied captures keep the lower real surface point.
        if (!anchor || f.stamps[i] > anchor->geometry_stamp ||
            (f.stamps[i] == anchor->geometry_stamp &&
             f.points[i].z() < anchor->point.z()))
          anchor = SemanticAnchor{f.points[i], f.stamps[i], s};
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
    struct Selected {
      const Label* label;
      Pose camera_pose;
      Camera camera;
    };
    const int64_t gap = nearest_even(cfg.semantic.at("max_pose_gap_ms").get<double>() * 1e6);
    auto select = [&](const Label& label, Selected& result) {
      if (label.stamp > now||now - label.stamp > window)
        return false;
      try{
        auto p = interpolate(rows, label.stamp, gap);
        result.label = &label;
        result.camera_pose = p;
        result.camera.k = k;
        result.camera.width = width;
        result.camera.height = height;
        result.camera.world = M4::Identity();
        result.camera.world.block<3, 3>(0, 0) = p.q.toRotationMatrix() * mount.block<3, 3>(0, 0);
        result.camera.world.block<3, 1>(0, 3) = p.p + p.q * mount.block<3, 1>(0, 3);
        return true;
      }
      catch (const std::exception&){}
      return false;
    };
    // Legacy: the newest label with a valid camera pose only.
    std::vector<Selected> chosen;
    for (auto it = labels.rbegin(); it != labels.rend(); ++it)
    {
      Selected candidate;
      if (select(*it, candidate)) { chosen.push_back(candidate); break; }
    }
    if (chosen.empty())
      return out;
    if (cfg.label_accumulation)
    {
      // Sliding window (2026-11-13): every label newer than the last projected one,
      // oldest first, at most label_accumulation_max (the newest ones). The final
      // pass is always the legacy newest label, so a tick without new labels still
      // projects newly received geometry exactly as before.
      std::vector<Selected> pending;
      for (const auto& label:labels)
      {
        if (label.stamp <= last_label || label.stamp >= chosen.back().label->stamp)
          continue;
        Selected candidate;
        if (select(label, candidate)) pending.push_back(candidate);
      }
      const size_t room = size_t(cfg.label_accumulation_max) - 1;
      if (pending.size() > room)
      {
        labels_skipped += pending.size() - room;
        pending.erase(pending.begin(), pending.end() - room);
      }
      pending.push_back(chosen.back());
      chosen.swap(pending);
    }
    const Pose newest_pose = rows.back();
    bool did = false;
    for (size_t index = 0; index < chosen.size(); ++index)
    {
      const bool last_pass = index + 1 == chosen.size();
      const auto& item = chosen[index];
      if (item.label->stamp > last_label) { last_label = item.label->stamp; ++labels_projected; }
      project_label(*item.label, item.camera_pose, item.camera, now, window, last_pass, last_pass, did, out, newest_pose);
    }
    if (did)
      ++sequence;
    return out;
  }
  void LocalGround::project_label(const Label& selected, const Pose& camera_pose, const Camera& camera, int64_t now, int64_t window,
                                  bool emit, bool refresh_anchors, bool& did, std::vector<Frame>& out, const Pose& newest_pose)
  {
    int64_t lt = selected.stamp;
    std::set<Key> current_geometry_cells;
    std::set<int> grounds = cfg.semantic.at("ground_class_ids").get<std::set<int> >();
    int64_t max_dt = nearest_even(cfg.semantic.at("max_point_label_dt_ms").get<double>() * 1e6), dynamic = nearest_even(cfg.semantic.at("dynamic_retention_s").get<double>() * 1e9);
    for (auto& source:frames)
      for (auto& f:source)
      {
        if (f.attempt >= lt)
          continue;
        auto ids = project(f.frame.points, selected, camera, cfg);
        std::vector<uint8_t> mask(ids.size());
        for (size_t i = 0; i < ids.size(); ++i)
        {
          if (f.frame.stamps[i] < lt - window||f.frame.stamps[i] > lt + max_dt)
            ids[i] = -1;
          if (cfg.person_ray_restore) {
            const Key key = key_of(f.frame.points[i],cfg.resolution);
            if (!grid.accepts(key,ids[i],f.frame.stamps[i])) ids[i] = -1;
            if (f.ids[i] >= 0 && !grid.accepts(key,f.ids[i],f.frame.stamps[i])) {
              f.ids[i] = -1;
              f.label_stamps[i] = 0;
            }
          }
          if (ids[i] >= 0&&lt >= f.label_stamps[i])
          {
            f.ids[i] = ids[i];
            f.label_stamps[i] = lt;
            current_geometry_cells.insert(key_of(f.frame.points[i], cfg.resolution));
          }
          if (f.ids[i] >= 0&&cfg.class_info(f.ids[i]).dynamic&&now - f.label_stamps[i] > dynamic)
            f.ids[i] = -1;
          mask[i] = grounds.count(f.ids[i]);
        }
        f.attempt = lt;
        grid.add(f.frame, f.ids, f.label_stamps, lt);
        if (cfg.person_ray_restore)
          for (const auto& person:grid.unresolved_people) semantic_anchors.erase(person.first);
        last_stamp = std::max(last_stamp, f.frame.stamp);
        if (emit&&(!f.emitted||*f.emitted != mask))
        {
          Frame ground = f.frame;
          ground.points.clear();
          ground.stamps.clear();
          ground.pose = newest_pose;
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
    // Semantic memory must not depend on a new ground point or plane fitting.
    // Reproject only once per newer mask, at most 1Hz, using its own pose.
    // Geometry timestamps stay original and these frames are NEVER emitted to
    // Ground; a semantic-only refresh cannot invent slope or height evidence.
    if (refresh_anchors&&lt > anchor_attempt)
    {
      // Rate-limit by current pose time, not mask-to-mask jitter. A mask that
      // arrives 0.99s after the previous one can be retried on the next tick.
      if (anchor_refresh < 0 || now - anchor_refresh >= 1000000000LL)
      {
        anchor_attempt = lt;
        anchor_refresh = now;
        ROI roi;
        roi.mounts = mounts;
        roi.radius = cfg.semantic.at("local_ground").at("radius_m");
        roi.set_pose(camera_pose);
        const double halo = cfg.semantic.at("local_ground").at("support_halo_m");
        const double radius = roi.radius + halo;
        const double xmin = std::min(roi.origins[0].x(), roi.origins[1].x()) - radius;
        const double xmax = std::max(roi.origins[0].x(), roi.origins[1].x()) + radius;
        const double ymin = std::min(roi.origins[0].y(), roi.origins[1].y()) - radius;
        const double ymax = std::max(roi.origins[0].y(), roi.origins[1].y()) + radius;
        const Key minimum = key_of(V3(xmin, ymin, 0.), cfg.resolution);
        const Key maximum = key_of(V3(xmax, ymax, 0.), cfg.resolution);
        std::array<Frame, 2> cached;
        for (int source=0;source<2;++source) {
          cached[source].source = source;
          cached[source].pose = camera_pose;
        }
        // Query just the current ROI's X/Y cell range, not the whole map.
        for (int64_t x=minimum.first;x<=maximum.first;++x)
        {
          auto it = semantic_anchors.lower_bound({x, minimum.second});
          for (;it!=semantic_anchors.end()&&it->first.first==x&&
                 it->first.second<=maximum.second;++it)
          {
            if (current_geometry_cells.count(it->first))
              continue;
            if (cfg.person_ray_restore && grid.unresolved_people.count(it->first)) continue;
            const auto cell = grid.cells.find(it->first);
            bool current_direct = false;
            if (cell != grid.cells.end())
              for (const auto& evidence:cell->second.classes)
                current_direct |= evidence.second.last >= lt && !evidence.second.reprojected;
            if (current_direct)
              continue;
            for (const auto& anchor:it->second)
              if (anchor && anchor->geometry_stamp > 0 &&
                  grid.accepts(it->first,0,anchor->geometry_stamp,true) &&
                  anchor->geometry_stamp <= lt + max_dt &&
                  roi.contains(anchor->point.head<2>(), halo))
              {
                auto& frame = cached[anchor->source];
                frame.points.push_back(anchor->point);
                frame.stamps.push_back(anchor->geometry_stamp);
                frame.stamp = std::max(frame.stamp, anchor->geometry_stamp);
              }
          }
        }
        for (const auto& frame:cached)
        {
          if (frame.points.empty())
            continue;
          const auto ids = project(frame.points, selected, camera, cfg);
          Frame accepted = frame;
          accepted.points.clear();
          accepted.stamps.clear();
          std::vector<int> accepted_ids;
          for (size_t i=0;i<ids.size();++i)
            if (ids[i] >= 0 && grid.accepts(key_of(frame.points[i],cfg.resolution),
                                          ids[i],frame.stamps[i],true))
            {
              accepted.points.push_back(frame.points[i]);
              accepted.stamps.push_back(frame.stamps[i]);
              accepted_ids.push_back(ids[i]);
            }
          if (!accepted_ids.empty())
          {
            grid.add(accepted, accepted_ids,
                     std::vector<int64_t>(accepted_ids.size(), lt), lt, true);
            if (cfg.person_ray_restore)
              for (const auto& person:grid.unresolved_people) semantic_anchors.erase(person.first);
            did = true;
          }
        }
      }
    }
  }
}

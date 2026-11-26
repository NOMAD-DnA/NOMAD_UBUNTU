#include "nomad_costmap/detail.hpp"
#include "nomad_costmap/moving_front_free_region.hpp"
#include <algorithm>

namespace nomad {
  CostSnapshot build_cost(const SlopeSnapshot& live, const SemanticSnapshot& semantic, const Config& cfg, StartupFreeRegion* startup, const MovingFrontFreeRegion* moving, const MapWindow* window)
  {
    // 2026-11-13: same rules as the JSON version; rows are structs.
    std::map<Key, const SlopeRow*> slopes;
    std::map<Key, const SemanticRow*> sems;
    std::set<Key> initialized_slopes(live.startup_frozen_cells.begin(), live.startup_frozen_cells.end());
    for (auto& r:live.rows)
      slopes[{r.ix, r.iy}] = &r;
    for (auto& r:semantic.rows)
      sems[{r.ix, r.iy}] = &r;
    if (startup) {
      // Even incomplete real observations supersede this one-time assumption.
      // Retirement is permanent for this map session, including after expiry.
      for (const auto& [k,r]:slopes)
        if (r->observed > 0) startup->retire_slope(k);
      for (const auto& [k,r]:sems) {
        // A person overlays the user-declared terrain prior, rather than
        // permanently destroying it. Other observations retain old retirement.
        if (cfg.person_ray_restore && r->has_person &&
            r->person_last_geometry_ns > 0 &&
            r->base_last_semantic_ns <= 0) continue;
        if (r->last_geometry_ns > 0 ||
            r->last_semantic_ns > 0 ||
            r->camera_observed_ns > 0) startup->retire_semantic(k);
      }
    }
    std::set<Key> keys;
    for (auto& v:slopes)
      keys.insert(v.first);
    for (auto& v:sems)
      keys.insert(v.first);
    if (startup)
      keys.insert(startup->cells().begin(),startup->cells().end());
    const bool moving_active = moving && moving->enabled() && moving->initialized();
    if (moving_active) keys.insert(moving->cells().begin(),moving->cells().end());
    const bool driven_active = moving && moving->driven_enabled() && !moving->driven().empty();
    if (driven_active) keys.insert(moving->driven().begin(),moving->driven().end());
    require(!keys.empty(), "No measured cells for cost output");
    int64_t xmin = INT64_MAX, xmax = INT64_MIN, ymin = INT64_MAX, ymax = INT64_MIN;
    if (window && window->valid)
    {
      // Fixed vehicle-centred map (map_length_m) instead of the measured bounding box.
      xmin = window->xmin; ymin = window->ymin;
      xmax = xmin + window->width - 1; ymax = ymin + window->height - 1;
    }
    else
    for (auto k:keys)
    {
      xmin = std::min(xmin, k.first);
      xmax = std::max(xmax, k.first);
      ymin = std::min(ymin, k.second);
      ymax = std::max(ymax, k.second);
    }
    auto inside = [&](Key k){ return k.first >= xmin && k.first <= xmax && k.second >= ymin && k.second <= ymax; };
    int64_t w = xmax - xmin + 1, h = ymax - ymin + 1;
    require(w > 0&&h > 0&&w <= 4000000&&h <= 4000000&&w * h <= 4000000, "Cost grid exceeds limit");
    size_t n = size_t(w * h);
    std::vector<double> angles(n, nan), tilts(n, nan), scores(n, nan), terrain(n, nan);
    std::vector<int64_t> st(n), semt(n), camt(n);
    std::vector<std::vector<int64_t> > captures(n);
    int64_t capture = live.capture; // Actual processed scan time, never fabricated cell observations.
    for (auto& [k, r]:slopes)
    {
      if (!inside(k)) continue;
      size_t i = size_t((k.second - ymin) * w + k.first - xmin);
      st[i] = r->observed;
      if (st[i] > 0)
      {
        angles[i] = r->angle;
        tilts[i] = r->tilt;
        capture = std::max(capture, st[i]);
      }
    }
    for (auto& [k, r]:sems)
    {
      if (!inside(k)) continue;
      size_t i = size_t((k.second - ymin) * w + k.first - xmin);
      semt[i] = r->last_semantic_ns;
      camt[i] = std::max(int64_t(0), r->camera_observed_ns);
      if (r->cost >= 0&&semt[i] > 0)
        terrain[i] = float(r->cost);
      else
        semt[i] = 0;
      captures[i] = r->camera_capture_ns;
      capture = std::max(capture, camt[i]);
      if (std::isfinite(terrain[i]))
        capture = std::max(capture, semt[i]);
      if (cfg.person_ray_restore)
        capture = std::max(capture,r->person_clear_observed_ns);
    }
    require(capture > 0, "Positive capture required");
    CostSnapshot result;
    result.capture = capture;
    result.resolution = cfg.resolution;
    result.origin_x = xmin*cfg.resolution;
    result.origin_y = ymin*cfg.resolution;
    result.width = w;
    result.height = h;
    for (auto* v:{&result.slope_deg, &result.slope_magnitude_deg, &result.slope_score, &result.semantic_score, &result.slope_cost, &result.terrain_cost})
      v->reserve(n);
    for (auto* v:{&result.global, &result.local, &result.support})
      v->reserve(n);
    const float fnan = std::numeric_limits<float>::quiet_NaN();
    // Former num(float(x)) -> JSON null for nonfinite -> NaN on publication.
    auto f32 = [&](float x){ return std::isfinite(x)?x:fnan; };
    for (size_t i = 0; i < n; ++i)
    {
      double a = angles[i], t = tilts[i];
      float score = slope_score_of(a, t, cfg.slope_score_per_deg, cfg.slope_block_deg);
      if (st[i] <= 0)
        score = std::numeric_limits<float>::quiet_NaN();
      const Key key{xmin+int64_t(i)%w,ymin+int64_t(i)/w};
      const bool unobserved_slope_prior = startup && startup->slope_cells().count(key);
      const bool slope_prior = unobserved_slope_prior || initialized_slopes.count(key);
      const auto semantic_row = sems.find(key);
      const bool unresolved_person = cfg.person_ray_restore && semantic_row != sems.end() &&
        semantic_row->second->has_person && semantic_row->second->person_unresolved;
      const bool initial_person_scope = cfg.startup_person_cost_overlay &&
        ((startup && startup->original_cells().count(key)) || initialized_slopes.count(key));
      const bool person_cost_overlay = initial_person_scope && unresolved_person;
      const bool semantic_prior = startup && startup->semantic_cells().count(key) && !unresolved_person &&
        (!cfg.startup_person_cost_overlay || !std::isfinite(terrain[i]));
      if (slope_prior) { a = t = 0.; score = 0.; }
      if (semantic_prior) terrain[i] = 0.;
      const bool restored_zero = initial_person_scope && slope_prior && semantic_prior &&
        semantic_row != sems.end() &&
        semantic_row->second->has_person &&
        semantic_row->second->person_clear_source == "lidar_xy_ray_pass";
      // Preserve the frozen zero-degree plane. A person makes the exported
      // slope cost unknown (NaN); semantic cost100 keeps both final maps blocked.
      if (person_cost_overlay) {
        score = std::numeric_limits<float>::quiet_NaN();
        terrain[i] = 100.;
      }
      // Output-only user override for all nonperson blocking classes.
      // People retain the original occupancy, expiry and actual-ray release rules.
      // Preserve source terrain, actual angles, timestamps and camera support.
      const bool selected_person = semantic_row != sems.end() &&
        semantic_row->second->class_id >= 0 && semantic_row->second->class_id == cfg.person_class_id;
      // Driven path (cells the body covered): drivable by definition, all costs 0, permanent.
      const bool driven_override = driven_active && moving->driven().count(key) &&
        !unresolved_person && !selected_person;
      const bool moving_override = driven_override || (moving_active && moving->cells().count(key) &&
        !unresolved_person && !selected_person);
      if (moving_override) {
        // Front region: measured slope is kept (steep still blocks); unmeasured slope (blind zone) is 0.
        if (driven_override || !moving->keep_measured_slope() || !std::isfinite(score)) score = 0.f;
        terrain[i] = 0.f;
      }
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
      const bool recent = cfg.retain_last_observed_costs ||
        ((unobserved_slope_prior||restored_zero||double(capture - st[i]) / 1e9 <= 30.) &&
         (semantic_prior||double(capture - semt[i]) / 1e9 <= 30.));
      int lc = known && recent ? occ : -1;
      if (blocked)
        lc = 100;
      if (moving_override) lc = 0;
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
      result.slope_deg.push_back(f32(float(a)));
      result.slope_magnitude_deg.push_back(f32(float(t)));
      result.slope_score.push_back(f32(score));
      result.semantic_score.push_back(f32(float(terrain[i])));
      result.slope_cost.push_back(f32(float(score / 100.f)));
      result.terrain_cost.push_back(f32(float(float(terrain[i]) / 100.f)));
      result.global.push_back(int8_t(occ));
      result.local.push_back(int8_t(lc));
      result.support.push_back(int8_t(sup));
    }
    return result;
  }
}

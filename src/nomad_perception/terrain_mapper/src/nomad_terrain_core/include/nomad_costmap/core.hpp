#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace nomad {
  class CellExecutor;
  class MovingFrontFreeRegion;
  using Json = nlohmann::json;
  using V3 = Eigen::Vector3d;
  using M3 = Eigen::Matrix3d;
  using M4 = Eigen::Matrix4d;
  using Points = std::vector<V3>;
  using Key = std::pair<int64_t, int64_t>;
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
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
  };
  struct Mounts {
    std::array<M4, 2> sensor{M4::Identity(), M4::Identity()};
  };
  struct Config {
    Json semantic;
    // Vehicle forward expressed in the public base_link frame, matching Pi4.
    V3 slope_forward_axis_base_link = V3::UnitX();
    double resolution = .12, radius = .18, window = 3., forward_radius = 3., max_size = 160.;
    double bin = .03, minor = .01, variance_ratio = .02, jump = .12, rms = .01, target_error = .02, tls_gap = 1e-8, xy_rank = 1e-6;
    bool oak_lidar_height_gate = true;
    bool freeze_slope_after_first_valid = false;
    // With freezing enabled, retry planes whose current 0..100 slope score is >=100.
    bool retry_slope_score_at_least_100 = false;
    int slope_compute_threads = 1, slope_parallel_min_cells = 32;
    bool startup_free_region_enabled = false;
    bool startup_freeze_initial_slope = false;
    // Initial flat cells use a person-cost overlay until an actual LiDAR ray clears it.
    bool startup_person_cost_overlay = false;
    bool person_ray_restore = false;
    int person_class_id = 1;
    double oak_lidar_xy_radius = .06, oak_lidar_z_reject = .12;
    int min_raw = 12, min_bins = 6, min_heights = 6, raw_reference = 30, per_cell = 512, global_points = 1000000;
    double space_fraction = .1;
    // 2026-11-13 additions. Defaults reproduce the legacy behaviour exactly.
    // slope.json "slope_angle_reference": "heading" (legacy signed slope along the
    // vehicle heading) or "gravity" (angle between the fitted plane normal and gravity).
    bool gravity_slope_angle = false;
    // slope.json "startup_seed_keep_frozen": the user-declared initial flat cells
    // keep their zero slope (legacy). false: real geometry recalculates them.
    bool startup_seed_frozen = true;
    // slope.json "map_length_m": square vehicle-centred map side. 0 = legacy
    // (no pruning, cost grid = bounding box of every measured cell).
    double map_length = 0.;
    // semantic.real.json local_ground.label_accumulation: project every unprocessed
    // label image in time order (sliding window) instead of the newest one only.
    bool label_accumulation = false;
    int label_accumulation_max = 4;
    // 2026-11-14 additions (absent keys keep the 11-13 behaviour).
    // slope.json "keep_slope_on_failure": a cell whose recalculation fails keeps its
    // previous valid plane instead of being overwritten by an undetermined estimate.
    bool keep_slope_on_failure = false;
    // slope.json "cost_area": "map_window" (whole square) or "known_cells" (bounding
    // box of measured cells inside the window, the legacy grid extent).
    bool cost_known_cells = false;
    // slope.json "max_map_cells" (0 = off) and "map_keep_radius_m": every store keeps
    // at most this many cells; the least recently observed cells outside the keep
    // radius around the vehicle are dropped first.
    size_t max_map_cells = 0;
    double map_keep_radius = 6.;
    // 2026-11-16 additions (absent keys keep the former constants 4 and 20).
    // slope.json/terrain.json "slope_score_per_deg" (k): slope score = k * tilt for
    // uphill (heading) or any tilt (gravity). "slope_block_deg" (B): score 100 from
    // this tilt. When B is absent it follows k as 80/k (k=4 -> 20, k=2 -> 40, k=8 -> 10),
    // so the score just below the block angle stays 80. k*B must be <= 100.
    double slope_score_per_deg = 4.;
    double slope_block_deg = 20.;
    // Class table cached from semantic["classes"] once at load (the old code looked
    // the JSON object up by string for every projected point).
    struct ClassInfo {
      bool known = false, ignore = false, dynamic = false;
      std::string kind;
      int cost = -1;                      // -1 = null
    };
    std::array<ClassInfo, 256> classes{};
    const ClassInfo& class_info(int id) const {
      if (id < 0 || id > 255 || !classes[size_t(id)].known) throw std::out_of_range("Unknown semantic class id");
      return classes[size_t(id)];
    }
    static Config load(const Json&, const Json&);
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
  V3 slope_forward_axis_in_imu(const Config&, const M4& base_from_imu);
  M4 rigid(const Json&);
  Json matrix_json(const M4&);
  Json points_json(const Points&);
  Points points_from_json(const Json&);
  Pose pose_from_json(const Json&);
  Json pose_json(const Pose&);
  Pose interpolate(const std::vector<Pose>&, int64_t, int64_t max_gap = 200000000);
  Points transform(const Points&, const M4&);
  double median(std::vector<double>);
  int64_t nearest_even(double);
  struct ROI {
    Mounts mounts;
    Pose pose;
    bool valid = false;
    std::array<Eigen::Vector2d, 2> origins, forward;
    double radius = 3.;
    void set_pose(const Pose&);
    bool contains(const Eigen::Vector2d&, double margin = 0.)const;
  };
  struct Fit {
    V3 normal = V3::Constant(nan);
    double offset = 0., tilt = 0., residual = 0., qspace = 0.;
  };
  std::optional<Fit> fit_tls(const Points&, const std::vector<double>&, const Config&, bool weighted = true);
  Points frame_voxels(const Points& camera, const Points& world);
  bool spread(const Points&, const Config&);
  struct Estimate {
    bool valid = false, reused = false, startup_seeded = false;
    double height = nan, confidence = 0.;
    Fit fit;
    int raw_count = 0;
    int64_t last_observed = 0;
  };
  struct Capture {
    int64_t stamp;
    Points points;
  };
  struct SampleCell {
    std::array<std::deque<Capture>, 2> sources;
  };
  class Ground {
public:
    Config cfg;
    ROI roi;
    std::map<Key, SampleCell> samples;
    std::vector<Key> insertion;
    std::set<Key> observed, dirty, previous_active;
    std::map<Key, Estimate> estimates;
    std::map<Key, std::pair<double, int64_t> > heights;
    std::map<Key, int64_t> last_observed, calculated;
    std::array<int64_t, 2> last_stamp{-1, -1}, last_bucket{-1, -1}, source_capture{0, 0};
    int64_t latest = -1;
    uint64_t revisions = 0;
    Json counters = Json::object();
    // Runtime diagnostics only; excluded from legacy numerical snapshots.
    Json slope_timing = Json::object();
    std::set<Key> initial_flat_slopes;
    int64_t initial_flat_stamp = 0;
    V3 forward_axis = V3::UnitX();
    Ground(const Config&, const Mounts&);
    void seed_initial_flat_slopes(const std::set<Key>&, int64_t);
    void ingest(const Frame&, bool revision = false);
    SlopeSnapshot snapshot();
    // Drop every cell outside the map window (all per-cell state).
    void prune(const MapWindow&);
private:
    std::shared_ptr<CellExecutor> cell_executor;
    struct Rep {
      V3 mean;
      Key bin;
      int count;
    };
    bool slope_frozen(const Estimate&, const Eigen::Vector2d&)const;
    std::set<Key> filter_oak_against_lidar(const Eigen::Vector2d&);
    Points update_height(Key);
    std::vector<Rep> prepare()const;
    std::optional<Fit> refit(Key)const;
  };
  struct Evidence {
    int64_t last = 0;
    std::array<uint64_t, 2> votes{0, 0};
    // A current real-geometry observation wins a same-capture tie with replay.
    bool reprojected = false;
  };
  struct SemanticCell {
    int64_t geometry = 0, camera = 0;
    std::vector<int64_t> captures;
    std::map<int, Evidence> classes;
    std::vector<int> class_order;
    int64_t person_geometry = 0, person_semantic = 0, person_clear = 0;
    std::string person_clear_source;
  };
  class SemanticGrid {
public:
    Config cfg;
    std::map<Key, SemanticCell> cells;
    // Terrain evidence stays in classes; only this person occupancy is cleared.
    std::map<Key, int64_t> unresolved_people;
    std::set<Key> startup_person_cells;
    uint64_t revision = 0;
    using Expiry = std::tuple<int64_t, Key, int, int64_t>;
    std::priority_queue<Expiry, std::vector<Expiry>, std::greater<Expiry> > expirations;
    explicit SemanticGrid(const Config& c) : cfg(c)
    {
    }
    void add(const Frame&, const std::vector<int>&, const std::vector<int64_t>&, int64_t camera, bool reprojected = false);
    void seed_startup_person_cells(const std::set<Key>&);
    bool accepts(Key, int id, int64_t geometry, bool reprojected = false) const;
    std::vector<Key> clear_person_by_lidar(const Frame&, const M4& imu_lidar);
    uint64_t version(int64_t now);
    SemanticSnapshot snapshot(int64_t now)const;
    void prune(const MapWindow&);
  };
  struct Camera {
    M4 world = M4::Identity();
    M3 k = M3::Identity();
    int width = 1280, height = 720;
  };
  struct Label {
    int64_t stamp = 0;
    int width = 640, height = 360;
    std::vector<uint8_t> ids, safe;
  };
  void label_safe(Label&, int margin);
  std::vector<int> project(const Points&, const Label&, const Camera&, const Config&);
  struct Geometry {
    Frame frame;
    std::vector<int> ids;
    std::vector<int64_t> label_stamps;
    int64_t attempt = -1;
    std::optional<std::vector<uint8_t> > emitted;
  };
  struct SemanticAnchor {
    V3 point = V3::Zero();
    int64_t geometry_stamp = 0;
    int source = 0;
  };
  class LocalGround {
public:
    Config cfg;
    Mounts mounts;
    SemanticGrid grid;
    std::array<std::deque<Geometry>, 2> frames;
    std::array<int64_t, 2> last{-1, -1}, bucket{-1, -1};
    // Actual observed points only: two sources x four XY subcells, max 8/cell.
    // This cache is semantic evidence, never a source of slope/height frames.
    std::map<Key, std::array<std::optional<SemanticAnchor>, 8> > semantic_anchors;
    int64_t anchor_attempt = -1, anchor_refresh = -1;
    uint64_t sequence = 0;
    int64_t last_stamp = 0;
    LocalGround(const Config& c, const Mounts& m) : cfg(c), mounts(m), grid(c)
    {
    }
    // Newest label stamp already projected (label_accumulation only).
    int64_t last_label = -1;
    uint64_t labels_projected = 0, labels_skipped = 0;
    void submit(Frame, const Points* camera = nullptr);
    std::vector<Frame> process(const std::vector<Pose>&, const std::deque<Label>&, const M4& mount, const M3& k, int width, int height);
    void prune(const MapWindow&);
private:
    // One legacy process() pass for one selected label (body unchanged).
    void project_label(const Label& selected, const Pose& camera_pose, const Camera& camera, int64_t now, int64_t window,
                       bool emit, bool refresh_anchors, bool& did, std::vector<Frame>& out, const Pose& newest_pose);
public:
  };
  CostSnapshot build_cost(const SlopeSnapshot& slope, const SemanticSnapshot& semantic, const Config&, StartupFreeRegion* startup = nullptr, const MovingFrontFreeRegion* moving = nullptr, const MapWindow* window = nullptr);
  std::array<M4, 2> factory_mounts(const Json& factory, const Json& config);
  Mounts sensor_mounts(const M4& imu_b, const M4& b_lidar, const M4& imu_a);
  // Former JSON scan/cloud/pair messages between the receiver and mapper processes.
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
  ScanPair scan_pair(const ScanInput& scan, const CloudInput& cloud, const M4& camera_lidar);
  Frame deskew(const ScanPair& pair, const std::vector<Pose>&, const M4& imu_lidar, double offset_s = 0.);
  // Same validation/normalisation as pose_from_json(pose_json(p)), without JSON.
  Pose checked_pose(const Pose&);
}

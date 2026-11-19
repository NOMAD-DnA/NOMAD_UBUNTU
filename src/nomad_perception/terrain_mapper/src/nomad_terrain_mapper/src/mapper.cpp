#include <nomad_costmap/oak_height_override.hpp>
#include <nomad_costmap/oak_horizontal_crop.hpp>
// NOMAD legacy terrain mapper in ONE process (2026-11-13).
//
// Same map logic as nomad_costmap_cpp mapper.cpp + receivers.cpp + monitor.cpp:
// the VIO receiver, point receiver and slope worker used to be child processes
// that exchanged CBOR-encoded JSON over sockets, and the monitor wrote
// health.json that the mapper read back. Here:
//   - VIO / point / monitor input checks run in ROS callbacks of this node;
//   - the slope worker (legacy Ground) is a thread fed with native structs;
//   - map snapshots are structs (no JSON rows, no terrain/live.json writes);
//   - with map_length_m > 0 every per-cell state outside the vehicle-centred
//     map window is dropped, so the work no longer grows with the drive.
// Output topics, QoS and values are those of the legacy mapper.
#include "nomad_costmap/ros_util.hpp"
#include "nomad_costmap/input_health.hpp"
#include "nomad_costmap/camera_model.hpp"
#include "nomad_costmap/executor_event.hpp"
#include "nomad_costmap/moving_front_free_region.hpp"
#include <algorithm>
#include <cfenv>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

namespace nomad {
namespace {
// ----- slope worker (former "--mode slope" child) ---------------------------
struct SlopeTask {
  Frame frame;
  bool revision = false;
  Mounts mounts;
  V3 forward_axis = V3::UnitX();
  int64_t enqueued_ns = 0;
  bool has_seed = false;
  std::set<Key> seed_cells;
  int64_t seed_stamp = 0;
  MapWindow window;
};
struct SlopeResult {
  SlopeSnapshot live;
  uint64_t sequence = 0, updates = 0, discarded = 0, overflows = 0, acknowledged = 0;
  std::array<int64_t, 2> accepted_capture{0, 0}, accepted_enqueue{0, 0};
  int64_t worker_completed_ns = 0, mapper_received_ns = 0;
  double worker_compute_ms = 0.;
  Json slope_phases = Json::object();
};
// Legacy SlopeWorkQueue: overflow at 200 discards the backlog, batches of 32,
// the initial flat-slope seed is never lost.
struct SlopeWorkQueue {
  std::deque<SlopeTask> work;
  bool has_seed = false;
  std::set<Key> seed_cells;
  int64_t seed_stamp = 0;
  uint64_t discarded = 0, overflows = 0;
  void push(SlopeTask value) {
    if (value.has_seed) {
      if (!has_seed) { has_seed = true; seed_cells = value.seed_cells; seed_stamp = value.seed_stamp; }
      else if (seed_cells != value.seed_cells || seed_stamp != value.seed_stamp)
        throw std::runtime_error("Initial slope seed changed during mapping");
    }
    if (work.size() >= 200) { discarded += work.size(); work.clear(); ++overflows; }
    work.push_back(std::move(value));
  }
  std::deque<SlopeTask> take() {
    std::deque<SlopeTask> batch;
    for (size_t i=0;i<32&&!work.empty();++i) {
      batch.push_back(std::move(work.front())); work.pop_front();
    }
    if (!batch.empty() && has_seed && !batch.front().has_seed) {
      batch.front().has_seed = true; batch.front().seed_cells = seed_cells; batch.front().seed_stamp = seed_stamp;
    }
    return batch;
  }
};
// Legacy QueueRecovery with the slope timing JSON replaced by SlopeResult.
struct QueueRecovery {
  bool active = false;
  uint64_t epoch = 0, completed = 0, dropped = 0, deferred_replaced = 0;
  int64_t capture_after_ns = 0, started_monotonic_ns = 0;
  std::string cause;
  void begin(int64_t capture_now, int64_t wall_now, const std::string& why, uint64_t count = 0) {
    active = true; ++epoch; dropped += count;
    capture_after_ns = capture_now; started_monotonic_ns = wall_now; cause = why;
  }
  bool permits(int64_t map_capture, const SlopeResult& timing) const {
    if (!active) return true;
    if (map_capture <= capture_after_ns) return false;
    for (size_t s=0;s<2;++s)
      if (timing.accepted_capture[s] <= capture_after_ns ||
          timing.accepted_enqueue[s] < started_monotonic_ns) return false;
    return true;
  }
  bool finish(uint64_t expected_epoch, int64_t capture, const SlopeResult& timing) {
    if (!active) return true;
    if (epoch != expected_epoch || !permits(capture,timing)) return false;
    active = false; ++completed; return true;
  }
};
inline size_t push_recent_limit(size_t size, size_t limit) {
  if (!limit) throw std::invalid_argument("Zero observation queue capacity");
  return size >= limit ? size : 0;
}
// Geometry waiting for poses (former JSON "lidar"/"oak" observations).
struct GeometryItem {
  int64_t stamp_ns = 0, received_monotonic_ns = 0, received_ros_ns = 0;
  ScanPair pair;     // LiDAR
  Points optical;    // OAK optical XYZ
};

class TerrainMapper : public rclcpp::Node {
  Config cfg;
  StartupFreeRegion startup_free_region;
  MovingFrontFreeRegion front_free_region;
  uint64_t last_cost_front_version = 0;
  double oak_max_horizontal_distance_m = 2.0;
  uint64_t oak_crop_kept = 0, oak_crop_dropped = 0;
  M4 camera_lidar, imu_b = M4::Identity(), imu_a = M4::Identity(), zed_mount = M4::Identity();
  Mounts mounts;
  bool factory_valid = false, info_valid = false, initialized = false, clock_fault = false;
  bool startup_slope_seed_sent = false;
  RectifiedCameraModel rectified_model;
  int64_t last_now = 0, still_since = 0, last_capture = 0, last_map_wall = 0;
  std::string fatal, reason = "Waiting for Pi5 initialization";
  bool navigation_valid = false;
  int64_t last_status_steady = 0, last_status_report = 0;
  int last_status_state = -1;
  // ---- former VIO receiver child
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  StatusSample localization;
  std::optional<M4> fixed;
  std::optional<Pose> last_vio;
  int64_t vio_last_now = 0, pose_wall = 0;
  uint64_t vio_rejected = 0;
  int vio_fast = 0;
  std::string vio_error, vio_reason = "Waiting for Pi5 localization";
  // ---- former point receiver child
  struct PairSlot { bool has_scan = false, has_cloud = false; ScanInput scan; CloudInput cloud; };
  std::map<int64_t, PairSlot> pairs;
  std::deque<int64_t> insertion;
  std::array<int64_t, 2> point_capture{0, 0}, point_received{0, 0};
  std::map<std::string, uint64_t> point_counts;
  // ---- former monitor child
  InputHealth health;
  InputHealth::Result health_result;
  // ---- mapper
  std::vector<Pose> poses;
  std::array<std::deque<GeometryItem>, 2> pending;
  std::array<int64_t, 2> last_geometry{0, 0};
  std::deque<Label> labels;
  M3 camera_k = M3::Identity();
  int image_width = 1280, image_height = 720;
  std::unique_ptr<LocalGround> semantic;
  std::set<std::pair<int, int64_t> > submitted;
  std::shared_ptr<const SlopeResult> live_result;
  uint64_t live_version = 0, last_cost_version = 0;
  uint64_t semantic_version = UINT64_MAX, cached_semantic_version = UINT64_MAX;
  SemanticSnapshot cached_semantics;
  struct SemanticKeyRow {
    int64_t ix, iy; int cost; int64_t last_semantic, camera_observed;
    std::vector<int64_t> captures; int class_id; bool unresolved;
    bool operator==(const SemanticKeyRow& o) const {
      return ix==o.ix&&iy==o.iy&&cost==o.cost&&last_semantic==o.last_semantic&&camera_observed==o.camera_observed&&
        captures==o.captures&&class_id==o.class_id&&unresolved==o.unresolved;
    }
    bool operator!=(const SemanticKeyRow& o) const { return !(*this == o); }
  };
  std::vector<SemanticKeyRow> last_semantics, cached_key;
  bool have_last_semantics = false;
  MapWindow window, last_cost_window;
  int64_t last_prune_wall = 0;
  uint64_t lidar_done = 0, oak_done = 0, geometry_dropped = 0;
  // ---- slope worker thread
  std::mutex inbox_mutex;            // worker queue + result + recovery
  std::condition_variable worker_cv;
  SlopeWorkQueue work_queue;
  bool worker_stop = false;
  std::string worker_fault;
  std::optional<SlopeResult> slope_result;
  uint64_t slope_sent = 0, slope_acknowledged = 0;
  QueueRecovery recovery;
  uint64_t geometry_recovery_epoch = 0, worker_discarded_seen = 0;
  bool slope_throttled = false;
  std::array<std::optional<Frame>,2> deferred_ground;
  std::thread worker;
  // ---- ROS
  std::shared_ptr<ExecutorEvent> input_event, result_event;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::array<rclcpp::Publisher<Grid>::SharedPtr, 3> grids;
  rclcpp::Publisher<Layers>::SharedPtr layers, diagnostic_layers;
  rclcpp::Publisher<Status>::SharedPtr status;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 4> images;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  rclcpp::TimerBase::SharedPtr input_timer, status_timer;

  void overload_locked(const std::string& cause, uint64_t dropped = 0)
  {
    recovery.begin(this->now().nanoseconds(),steady_ns(),cause,dropped);
  }
  // ======================= slope worker =======================
  void worker_main()
  {
    std::unique_ptr<Ground> ground;
    uint64_t sequence = 0, updates = 0;
    std::array<int64_t,2> accepted_capture{0,0}, accepted_enqueue{0,0};
    try {
      for (;;)
      {
        std::deque<SlopeTask> batch;
        {
          std::unique_lock<std::mutex> lock(inbox_mutex);
          worker_cv.wait(lock, [&]{ return worker_stop || !work_queue.work.empty(); });
          if (worker_stop) return;
          batch = work_queue.take();
        }
        int64_t started = steady_ns();
        MapWindow latest_window;
        for (auto& j:batch)
        {
          if (!ground)
          {
            ground = std::make_unique<Ground>(cfg, j.mounts);
            ground->forward_axis = j.forward_axis;
            if (cfg.startup_freeze_initial_slope) {
              if (!j.has_seed) throw std::runtime_error("Missing initial flat-slope seed");
              ground->seed_initial_flat_slopes(j.seed_cells,j.seed_stamp);
            }
          }
          const auto& f = j.frame;
          ground->ingest(f, j.revision);
          ++updates;
          if (f.stamp >= accepted_capture.at(f.source)) {
            accepted_capture.at(f.source) = f.stamp;
            accepted_enqueue.at(f.source) = j.enqueued_ns;
          }
          if (j.window.active()) latest_window = j.window;
        }
        ground->prune(latest_window);
        SlopeResult result;
        result.live = ground->snapshot();
        int64_t completed = steady_ns();
        result.sequence = ++sequence;
        result.updates = updates;
        result.worker_compute_ms = (completed - started) / 1e6;
        result.worker_completed_ns = completed;
        result.slope_phases = ground->slope_timing;
        result.accepted_capture = accepted_capture;
        result.accepted_enqueue = accepted_enqueue;
        {
          std::lock_guard<std::mutex> lock(inbox_mutex);
          result.discarded = work_queue.discarded;
          result.overflows = work_queue.overflows;
          result.acknowledged = updates + work_queue.discarded;
          receive_slope_locked(std::move(result));
        }
        result_event->notify();
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      worker_fault = std::string("Slope worker stopped: ") + e.what();
      result_event->notify();
    }
  }
  // Former Mapper::receive() for kind == "slope" (inbox_mutex held).
  void receive_slope_locked(SlopeResult j)
  {
    j.mapper_received_ns = steady_ns();
    if (j.acknowledged > slope_sent || j.acknowledged < slope_acknowledged) {
      fatal = "Invalid slope observation acknowledgement";
      return;
    }
    slope_acknowledged = j.acknowledged;
    if (j.discarded > worker_discarded_seen) {
      overload_locked("Slope worker backlog",j.discarded-worker_discarded_seen);
      worker_discarded_seen = j.discarded;
    }
    slope_result = std::move(j); // Only completed snapshots coalesce normally.
  }
  // ======================= former VIO receiver =======================
  void on_localization(const Status& m)
  {
    localization = status_sample(m, steady_ns());
    health.observe_status("localization", m);
  }
  void on_odometry(const nav_msgs::msg::Odometry& m)
  {
    {
      InputHealth::Sample s;
      if (m.child_frame_id != "base_link") s.error = "Expected odom -> base_link pose";
      health.observe("pose", m.header, s);
    }
    if (!vio_error.empty())
      return;
    try{
      int64_t ns = stamp(m.header), now = this->now().nanoseconds(), wall = steady_ns();
      if (!status_ready(localization, now, wall))
        throw std::runtime_error("Pi5 localization not READY");
      if (m.header.frame_id != "odom"||m.child_frame_id != "base_link"||ns <= 0||(last_vio&&ns <= last_vio->stamp)||now - ns < -50000000||now - ns > 600000000)
        throw std::runtime_error("Wrong frame, stale/future or reordered Pi5 pose");
      auto tf = tf_buffer.lookupTransform("base_link", "oak_imu_frame", tf2::TimePointZero);
      if (tf.header.frame_id != "base_link"||tf.child_frame_id != "oak_imu_frame"||stamp(tf.header) != 0)
        throw std::runtime_error("Expected static base_link <- oak_imu_frame");
      M4 mount = tf_matrix(tf.transform);
      if (fixed&&(mount - *fixed).cwiseAbs().maxCoeff() > 1e-9)
      {
        vio_error = "Fixed IMU transform changed; restart with empty map";
        return;
      }
      const auto& p = m.pose.pose.position;
      const auto& q = m.pose.pose.orientation;
      Pose body_in;
      body_in.stamp = ns;
      body_in.p = V3(p.x, p.y, p.z);
      body_in.q = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      Pose body = checked_pose(body_in);
      Pose raw = matrix_pose(ns, pose_matrix(body) * mount);
      if (last_vio)
      {
        double speed = (raw.p - last_vio->p).norm() / ((ns - last_vio->stamp) / 1e9);
        vio_fast = speed > 10?vio_fast + 1:0;
        if (vio_fast >= 5)
        {
          vio_error = "VIO divergence: speed exceeded 10m/s for 5 samples";
          return;
        }
      }
      fixed = mount;
      last_vio = raw;
      pose_wall = wall;
      vio_reason = "Pi5 capture-time pose and static IMU transform available";
      // Former pose IPC row -> mapper pose history (checked_pose = pose_from_json).
      const Pose row = checked_pose(raw);
      if (!poses.empty()&&row.stamp <= poses.back().stamp)
        fatal = "Reordered VIO IPC history";
      else
      {
        poses.push_back(row);
        if (poses.size()>10000) poses.erase(poses.begin(),poses.end()-10000);
      }
    }
    catch (const std::exception& e){++vio_rejected;
      vio_reason = e.what();
    }
    input_event->notify();
  }
  // Former VIO heartbeat "ready" + "reason".
  bool vio_ready(std::string& why)
  {
    int64_t now = this->now().nanoseconds(), wall = steady_ns();
    if (vio_last_now&&now < vio_last_now - 50000000)
      vio_error = "Pi5 receiver ROS clock moved backwards";
    vio_last_now = now;
    bool ready = vio_error.empty()&&status_ready(localization, now, wall)&&fixed&&last_vio&&now - last_vio->stamp >= -50000000&&now - last_vio->stamp <= 600000000&&wall - pose_wall <= 1000000000;
    why = vio_error.empty()?vio_reason:vio_error;
    return ready;
  }
  // ======================= former point receiver =======================
  void increment(const char* key) { ++point_counts[key]; }
  void push_geometry(int source, GeometryItem item)
  {
    auto& queue = pending[source];
    const size_t limit = source?8:80;
    const size_t dropped = push_recent_limit(queue.size(), limit);
    if (dropped) queue.clear();
    queue.push_back(std::move(item));
    if (dropped) {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      overload_locked(std::string("Point receiver backlog: ") + (source?"oak":"lidar"),dropped);
    }
    input_event->notify();
  }
  void join(int64_t ns)
  {
    auto& item = pairs[ns];
    if (item.has_scan&&item.has_cloud)
    {
      try{
        GeometryItem g;
        g.pair = scan_pair(item.scan, item.cloud, camera_lidar);
        g.stamp_ns = ns;
        g.received_monotonic_ns = steady_ns();
        g.received_ros_ns = item.cloud.received_ros_ns;
        point_capture[0] = ns;
        point_received[0] = g.received_monotonic_ns;
        increment("paired_lidar");
        push_geometry(0, std::move(g));
      }
      catch (const std::exception&){
        increment("invalid_lidar");
      }
      pairs.erase(ns);
      insertion.erase(std::remove(insertion.begin(), insertion.end(), ns), insertion.end());
    }
    while (pairs.size() > 30)
    {
      pairs.erase(insertion.front());
      insertion.pop_front();
      increment("unmatched_pairs_expired");
    }
  }
  void on_scan(const sensor_msgs::msg::LaserScan& m)
  {
    health.observe("scan", m.header);
    if (m.header.frame_id != "g2_laser"||stamp(m.header) <= 0)
      return;
    increment("scan");
    const int64_t ns = stamp(m.header);
    if (!pairs.count(ns)) insertion.push_back(ns);
    auto& slot = pairs[ns];
    slot.has_scan = true;
    slot.scan.stamp_ns = ns;
    slot.scan.ranges.resize(m.ranges.size());
    for (size_t i = 0; i < m.ranges.size(); ++i)
      slot.scan.ranges[i] = std::isfinite(m.ranges[i])?double(m.ranges[i]):nan;
    slot.scan.angle_min = m.angle_min;
    slot.scan.angle_increment = m.angle_increment;
    slot.scan.range_min = m.range_min;
    slot.scan.range_max = m.range_max;
    join(ns);
  }
  void on_cloud(const sensor_msgs::msg::PointCloud& m)
  {
    health.observe("cloud", m.header);
    if (m.header.frame_id != "g2_laser"||stamp(m.header) <= 0)
      return;
    increment("cloud");
    const int64_t ns = stamp(m.header);
    if (!pairs.count(ns)) insertion.push_back(ns);
    auto& slot = pairs[ns];
    slot.has_cloud = true;
    slot.cloud.stamp_ns = ns;
    slot.cloud.points.clear();
    slot.cloud.points.reserve(m.points.size());
    for (auto& v:m.points)
      slot.cloud.points.emplace_back(v.x, v.y, v.z);
    slot.cloud.offsets.clear();
    for (auto& c:m.channels)
      if (c.name == "stamps")
        slot.cloud.offsets.assign(c.values.begin(), c.values.end());
    slot.cloud.received_ros_ns = this->now().nanoseconds();
    join(ns);
  }
  void on_oak(const sensor_msgs::msg::PointCloud2& m)
  {
    {
      InputHealth::Sample s;
      bool xyz = true;
      for (auto name:{"x", "y", "z"})
        xyz &= std::any_of(m.fields.begin(), m.fields.end(), [&](auto& f){ return f.name == name; });
      if (!xyz||m.width * m.height == 0)
        s.error = "Missing OAK XYZ fields or empty cloud";
      health.observe("oak_points", m.header, s);
    }
    try{
      GeometryItem g;
      g.optical = cloud_points(m, "oak_cam_a_optical_frame");
      g.stamp_ns = stamp(m.header);
      g.received_monotonic_ns = steady_ns();
      g.received_ros_ns = this->now().nanoseconds();
      point_capture[1] = g.stamp_ns;
      point_received[1] = g.received_monotonic_ns;
      increment("oak");
      push_geometry(1, std::move(g));
    }
    catch (const std::exception&){increment("invalid_oak");
    }
  }
  // ======================= mapper (legacy) =======================
  void on_factory(const std_msgs::msg::String& m)
  {
    try{
      auto transforms = factory_mounts(Json::parse(m.data), cfg.semantic);
      if (factory_valid&&((transforms[0] - imu_b).cwiseAbs().maxCoeff() > 1e-9||(transforms[1] - imu_a).cwiseAbs().maxCoeff() > 1e-9))
      {
        fatal = "Factory calibration changed during mapping";
        return;
      }
      imu_b = transforms[0];
      imu_a = transforms[1];
      mounts = sensor_mounts(imu_b, camera_lidar, imu_a);
      zed_mount = imu_b * camera_lidar * rigid(cfg.semantic.at("T_zed_optical_from_lidar")).inverse();
      if (!semantic)
        semantic = std::make_unique<LocalGround>(cfg, mounts);
      factory_valid = true;
    }
    catch (const std::exception& e){
      reason = "OAK calibration rejected: " + std::string(e.what());
    }
  }
  void on_info(const sensor_msgs::msg::CameraInfo& m)
  {
    {
      InputHealth::Sample s;
      if (m.width != 1280||m.height != 720||m.k[0] <= 0||m.k[4] <= 0)
        s.error = "Invalid camera model";
      health.observe("rgb_info", m.header, s);
    }
    try{
      camera_k=rectified_model.accept({{"frame",m.header.frame_id},{"width",m.width},{"height",m.height},{"p",m.p},{"r",m.r},{"d",m.d},{"distortion_model",m.distortion_model}},cfg.semantic);
      image_width = m.width;
      image_height = m.height;
      info_valid = true;
    }
    catch (const std::exception& e){
      info_valid = false;
      reason = e.what();
    }
  }
  void on_labels(const sensor_msgs::msg::Image& m)
  {
    {
      InputHealth::Sample s;
      if (m.width != 640||m.height != 360||m.encoding != "mono8")
        s.error = "Wrong image dimensions or encoding";
      health.observe("mask", m.header, s);
    }
    auto shape = cfg.semantic.at("label_image");
    if (m.header.frame_id != cfg.semantic.at("zed_optical_frame")||stamp(m.header) <= 0||m.width != shape.at("width")||m.height != shape.at("height")||m.encoding != "mono8"||m.step < m.width||m.data.size() < size_t(m.step) * m.height)
    {
      reason = "Invalid semantic image contract";
      return;
    }
    if (!labels.empty()&&stamp(m.header) < labels.back().stamp)
      return;
    Label label;
    label.stamp = stamp(m.header);
    label.width = m.width;
    label.height = m.height;
    label.ids.reserve(size_t(m.width) * m.height);
    for (size_t y = 0; y < m.height; ++y)
      label.ids.insert(label.ids.end(), m.data.begin() + y * m.step, m.data.begin() + y * m.step + m.width);
    for(auto id:label.ids)if(!cfg.classes[id].known){reason="Unexpected semantic class ID; complete image rejected";return;}
    label_safe(label, cfg.semantic.at("boundary_margin_px"));
    labels.push_back(std::move(label));
    while (labels.size() > 96)
      labels.pop_front();
  }
  std::pair<bool, std::string> ready()
  {
    int64_t now = this->now().nanoseconds(), wall = steady_ns();
    std::string f, wf;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      f = fatal;
      wf = worker_fault;
    }
    if (!f.empty())
      return {false, f};
    if (clock_fault)
      return {false, "ROS clock moved backwards; restart with empty map"};
    if (!wf.empty())
      return {false, wf};
    std::string why;
    if (!vio_ready(why))
      return {false, why};
    int64_t ns = point_capture[0], rx = point_received[0];
    if (ns <= 0||now - ns < -50000000||now - ns > 1000000000||wall - rx > 1000000000)
      return {false, "LiDAR paired capture or receipt expired"};
    if (!factory_valid||!info_valid)
      return {false, reason};
    if (!initialized)
      return {false, "Waiting for 2s of stationary VIO initialization"};
    const int64_t age = wall - health_result.updated_wall;
    if (health_result.updated_wall <= 0||age < 0||age > 1000000000)
      return {false, "Waiting for native input monitor"};
    if (!health_result.inputs_ready)
      return {false, health_result.input_reason};
    return {true, "Validated live input checks"};
  }
  void status_tick()
  {
    health_result = health.evaluate(*this, tf_buffer);
    int64_t now = this->now().nanoseconds();
    if (last_now&&now < last_now - 50000000)
      clock_fault = true;
    last_now = now;
    auto r = ready();
    if (!r.first)
    {
      navigation_valid = false;
      reason = r.second;
    }
    bool recovering;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      recovering = recovery.active;
      if (recovering&&r.first) reason = "Recovering queue overload: " + recovery.cause;
    }
    bool ok = !recovering&&navigation_valid&&r.first&&last_capture > 0&&now - last_capture >= -50000000&&now - last_capture <= 2000000000&&steady_ns() - last_map_wall <= 3000000000;
    Status m;
    set_stamp(m.header, now);
    m.valid_for.sec = 1;
    m.valid_for.nanosec = 0;
    m.state = ok?Status::READY:last_capture?Status::STOPPED:Status::WAITING;
    m.detail = ok?reason:"Not ready: " + reason + "; capture/clock/wall freshness required";
    const auto publish_ns = this->now().nanoseconds();
    if (publish_ns < now - 50000000) {
      clock_fault = true;
      m.state = Status::STOPPED;
      m.detail = "ROS clock moved backwards during status validation";
    }
    set_stamp(m.header, publish_ns);
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      if (recovery.active) {
        navigation_valid=false;
        m.state=last_capture?Status::STOPPED:Status::WAITING;
        m.detail="Recovering queue overload: " + recovery.cause + "; input status: " + r.second;
      }
    }
    status->publish(m);
    const auto published_steady = steady_ns();
    if (last_status_state != m.state || published_steady-last_status_report>=5000000000LL) {
      RCLCPP_INFO(get_logger(), "perception_status state=%u capture_age_ms=%.3f map_receipt_age_ms=%.3f labels_projected=%llu labels_skipped=%llu reason=%s",
                  unsigned(m.state),
                  last_capture ? (publish_ns-last_capture)/1e6 : -1.0,
                  last_map_wall ? (published_steady-last_map_wall)/1e6 : -1.0,
                  static_cast<unsigned long long>(semantic?semantic->labels_projected:0),
                  static_cast<unsigned long long>(semantic?semantic->labels_skipped:0),
                  m.detail.c_str());
      last_status_report=published_steady;
      last_status_state=m.state;
    }
    last_status_steady=published_steady;
  }
  bool stationary()
  {
    if (poses.size() < 10)
      return false;
    const auto& last = poses.back();
    auto it = std::lower_bound(poses.begin(), poses.end(), last.stamp - 2000000000, [](const Pose& p, int64_t n){
      return p.stamp < n;
    });
    if (poses.end() - it < 10||last.stamp - it->stamp < 1800000000)
      return false;
    double translation = 0, rotation = 0;
    for (; it != poses.end(); ++it)
    {
      translation = std::max(translation, (last.p - it->p).norm());
      rotation = std::max(rotation, Eigen::AngleAxisd(last.q.conjugate() * it->q).angle() * 180 / 3.14159265358979323846);
    }
    return translation <= .03&&rotation <= 2.;
  }
  void enqueue_ground(Frame frame)
  {
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      if (recovery.active && (frame.stamp <= recovery.capture_after_ns ||
          this->now().nanoseconds()-frame.stamp > (frame.source?1500000000LL:1000000000LL))) {
        ++recovery.dropped;
        return;
      }
      if (slope_throttled || slope_sent - slope_acknowledged >= 200) {
        if (!slope_throttled) {
          overload_locked("Slope submission backlog (200 outstanding)");
          slope_throttled = true;
        }
        auto& held = deferred_ground.at(frame.source);
        if (!held || frame.stamp >= held->stamp) {
          if (held) ++recovery.deferred_replaced;
          held = std::move(frame);
        } else ++recovery.dropped;
        return;
      }
      ++slope_sent;
    }
    auto identity = std::make_pair(frame.source, frame.stamp);
    bool revision = !submitted.insert(identity).second;
    SlopeTask task;
    const V3 axis = slope_forward_axis_in_imu(cfg, *fixed);
    task.revision = revision;
    task.mounts = mounts;
    task.forward_axis = axis;
    task.enqueued_ns = steady_ns();
    task.window = window;
    if (cfg.startup_freeze_initial_slope&&!startup_slope_seed_sent) {
      if (!startup_free_region.initialized()) throw std::runtime_error("Initial flat slopes are not anchored");
      task.has_seed = true;
      task.seed_cells = startup_free_region.slope_cells();
      task.seed_stamp = startup_free_region.metadata().at("anchor_pose")[0].get<int64_t>();
    }
    task.frame = std::move(frame);
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      work_queue.push(std::move(task));
    }
    worker_cv.notify_one();
    startup_slope_seed_sent = true;
  }
  void flush_deferred_ground()
  {
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      if (!slope_throttled || slope_sent != slope_acknowledged) return;
      slope_throttled = false;
    }
    for (auto& held:deferred_ground) if (held) {
      Frame frame = std::move(*held); held.reset();
      enqueue_ground(std::move(frame));
    }
  }
  void drop_recovered_geometry()
  {
    std::lock_guard<std::mutex> lock(inbox_mutex);
    if (geometry_recovery_epoch != recovery.epoch) {
      for (auto& queue:pending) {
        auto old=queue.size();
        queue.erase(std::remove_if(queue.begin(),queue.end(),[&](const GeometryItem& j){
          return j.stamp_ns <= recovery.capture_after_ns;
        }),queue.end());
        recovery.dropped += old-queue.size();
      }
      geometry_recovery_epoch=recovery.epoch;
    }
  }
  bool process_geometry(int source)
  {
    auto& queue = pending[source];
    bool changed = false;
    while (!queue.empty())
    {
      drain_completion();
      auto& item = queue.front();
      int64_t ns = item.stamp_ns;
      {
        std::lock_guard<std::mutex> lock(inbox_mutex);
        if (recovery.active && (ns <= recovery.capture_after_ns ||
            this->now().nanoseconds()-ns > (source?1500000000LL:1000000000LL))) {
          queue.pop_front(); ++geometry_dropped; ++recovery.dropped; continue;
        }
      }
      int64_t end = ns;
      if (source == 0)
      {
        const auto& offsets = item.pair.point_time_offsets_s;
        if (!offsets.empty())
          end += nearest_even(offsets.back() * 1e9);
      }
      if (poses.empty()||end > poses.back().stamp)
      {
        if (steady_ns() - item.received_monotonic_ns < 1500000000)
          break;
        queue.pop_front();
        ++geometry_dropped;
        continue;
      }
      if (ns <= last_geometry[source])
      {
        queue.pop_front();
        ++geometry_dropped;
        continue;
      }
      try{
        Frame frame;
        Points optical;
        if (source == 0)
          frame = deskew(item.pair, poses, imu_b * camera_lidar);
        else{
          optical = std::move(item.optical);
          auto pose = interpolate(poses, ns);
          frame.stamp = ns;
          frame.source = 1;
          frame.pose = poses.back();
          frame.points = transform(optical, pose_matrix(pose) * imu_a);
          // poses store world<-IMU. Recover the public vehicle origin at this capture,
          // including the fixed IMU offset; do not use the latest processing-time pose.
          if (!fixed) throw std::runtime_error("Missing vehicle/IMU mount for OAK crop");
          oak_crop_dropped += crop_oak_horizontal(optical, frame.points,
              pose_matrix(pose) * fixed->inverse(), cfg.slope_forward_axis_base_link,
              oak_max_horizontal_distance_m);
          oak_crop_kept += frame.points.size();
          frame.stamps.assign(frame.points.size(), ns);
        }
        semantic->submit(std::move(frame), source?&optical:nullptr);
        last_geometry[source] = ns;
        source?++oak_done:++lidar_done;
        changed = true;
      }
      catch (const std::exception& e){
        ++geometry_dropped;
        std::cerr << "Geometry capture " << ns << " rejected: " << e.what() << '\n';
      }
      queue.pop_front();
    }
    return changed;
  }
  Grid make_grid(const CostSnapshot& snapshot, const std::vector<int8_t>& values)
  {
    Grid m;
    set_stamp(m.header, snapshot.capture);
    m.info.resolution = snapshot.resolution;
    m.info.width = snapshot.width;
    m.info.height = snapshot.height;
    m.info.origin.position.x = snapshot.origin_x;
    m.info.origin.position.y = snapshot.origin_y;
    m.info.origin.orientation.w = 1.;
    m.data = values;
    return m;
  }
  void publish_cost()
  {
    if (!live_result||!semantic)
      return;
    auto now = this->now().nanoseconds();
    if (front_free_region.enabled()) {
      if (poses.empty() || !fixed || now-poses.back().stamp < -50000000LL ||
          now-poses.back().stamp > 500000000LL ||
          !front_free_region.update_from_imu(poses.back(),*fixed)) {
        navigation_valid=false; reason="Moving front free region requires a fresh valid vehicle pose"; return;
      }
    }
    const bool front_changed = front_free_region.version() != last_cost_front_version;
    const bool window_changed = window.valid && (window.xmin != last_cost_window.xmin || window.ymin != last_cost_window.ymin || !last_cost_window.valid);
    uint64_t version = semantic->grid.version(now);
    if (live_version == last_cost_version&&version == semantic_version&&!front_changed&&!window_changed)
      return;
    if (cached_semantic_version != version)
    {
      cached_semantics = semantic->grid.snapshot(now);
      cached_semantic_version = version;
      // The legacy change key over every semantic row, rebuilt only when the grid changed.
      cached_key.clear();
      cached_key.reserve(cached_semantics.rows.size());
      for (auto& row:cached_semantics.rows)
        cached_key.push_back({row.ix, row.iy, row.cost, row.last_semantic_ns, row.camera_observed_ns, row.camera_capture_ns, row.class_id, row.has_person && row.person_unresolved});
    }
    const bool semantics_changed = !have_last_semantics || cached_key != last_semantics;
    if (live_version == last_cost_version&&!semantics_changed&&!front_changed&&!window_changed)
    {
      semantic_version = version;
      return;
    }
    const bool moving_cost_changed = (front_free_region.enabled() && (front_changed || semantics_changed)) || window_changed;
    auto r = ready();
    if (!r.first)
    {
      navigation_valid = false;
      reason = r.second;
      return;
    }
    // cost_area known_cells (2026-11-14): no forced window, so the grid is the bounding box of the
    // measured cells, which the prune already keeps inside the window (legacy extent rule).
    auto snapshot = build_cost(live_result->live, cached_semantics, cfg, &startup_free_region, &front_free_region,
                               cfg.cost_known_cells ? nullptr : &window);
    uint64_t recovery_epoch;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      recovery_epoch = recovery.epoch;
      if (recovery.active && (slope_throttled ||
          !recovery.permits(snapshot.capture,*live_result))) {
        navigation_valid=false;
        reason="Recovering queue overload: waiting for new LiDAR/OAK and map result";
        return;
      }
    }
    last_semantics = cached_key;
    have_last_semantics = true;
    semantic_version = version;
    last_cost_version = live_version;
    last_cost_window = window;
    Grid global = make_grid(snapshot, snapshot.global), local = make_grid(snapshot, snapshot.local), support = make_grid(snapshot, snapshot.support);
    Layers bundle;
    bundle.grid = global;
    bundle.slope_cost = snapshot.slope_cost;
    bundle.terrain_cost = snapshot.terrain_cost;
    std::array<const std::vector<float>*, 4> values{&snapshot.slope_score, &snapshot.semantic_score, &snapshot.slope_deg, &snapshot.slope_magnitude_deg};
    for (int i = 0; i < 4; ++i)
    {
      sensor_msgs::msg::Image m;
      m.header = global.header;
      m.height = global.info.height;
      m.width = global.info.width;
      m.encoding = "32FC1";
      m.step = m.width * 4;
      m.data.resize(size_t(m.step) * m.height);
      std::memcpy(m.data.data(), values[i]->data(), m.data.size());
      images[i]->publish(m);
    }
    diagnostic_layers->publish(bundle);
    int64_t capture = snapshot.capture;
    if (capture < last_capture || (capture == last_capture && !moving_cost_changed))
    {
      status_tick();
      return;
    }
    if (now - capture < -50000000||now - capture > 2000000000)
    {
      navigation_valid = false;
      reason = "Stale or future cost snapshot";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      if (recovery.epoch != recovery_epoch || (recovery.active &&
          (slope_throttled || !recovery.permits(capture,*live_result)))) {
        navigation_valid=false; return;
      }
      const auto publication_now=this->now().nanoseconds();
      if (publication_now-capture < -50000000 || publication_now-capture > 2000000000) {
        navigation_valid=false;
        reason="Stale or future cost snapshot at publication";
        return;
      }
      grids[2]->publish(support);
      layers->publish(bundle);
      grids[0]->publish(global);
      grids[1]->publish(local);
      last_capture = capture;
      last_cost_front_version = front_free_region.version();
      last_map_wall = steady_ns();
      recovery.finish(recovery_epoch,capture,*live_result);
      navigation_valid = true;
      reason = "Validated perception snapshot";
    }
    status_tick();
  }
  void drain_completion()
  {
    std::optional<SlopeResult> completed;
    int64_t dispatch;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      completed = std::move(slope_result);
      slope_result.reset();
      dispatch = steady_ns();
    }
    if (!completed) return;
    auto result = std::make_shared<SlopeResult>(std::move(*completed));
    live_result = result;
    ++live_version;
    const int64_t previous = last_capture, started = steady_ns();
    publish_cost();
    // Small diagnostic record (the legacy one also carried full JSON dumps).
    diagnostic_msgs::msg::DiagnosticArray message;
    set_stamp(message.header, result->live.capture);
    diagnostic_msgs::msg::DiagnosticStatus record;
    record.name = "semantic_validation/timing";
    record.hardware_id = "native_cpp_single_process";
    bool active;
    std::map<std::string, std::string> values;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      active = recovery.active;
      values["slope_outstanding"] = std::to_string(slope_sent-slope_acknowledged);
      values["queue_recovery_active"] = active?"true":"false";
      values["queue_recovery_cause"] = recovery.cause;
      values["queue_recovery_dropped"] = std::to_string(recovery.dropped);
    }
    record.level = active?1:0;
    record.message = active?"Recovering queue overload":"Capture provenance and event dispatch";
    values["sequence"] = std::to_string(result->sequence);
    values["updates"] = std::to_string(result->updates);
    values["worker_compute_ms"] = std::to_string(result->worker_compute_ms);
    values["worker_to_mapper_receive_ms"] = std::to_string((result->mapper_received_ns-result->worker_completed_ns)/1e6);
    values["mapper_event_wait_ms"] = std::to_string((dispatch-result->mapper_received_ns)/1e6);
    values["mapper_cost_output_ms"] = std::to_string((steady_ns()-started)/1e6);
    values["mapper_published_new_capture"] = last_capture > previous?"true":"false";
    values["slope_rows"] = std::to_string(result->live.rows.size());
    values["semantic_rows"] = std::to_string(cached_semantics.rows.size());
    values["lidar_processed"] = std::to_string(lidar_done);
    values["oak_crop_kept"] = std::to_string(oak_crop_kept);
    values["oak_crop_dropped"] = std::to_string(oak_crop_dropped);
    values["oak_max_horizontal_distance_m"] = std::to_string(oak_max_horizontal_distance_m);
    values["oak_processed"] = std::to_string(oak_done);
    values["geometry_dropped"] = std::to_string(geometry_dropped);
    values["labels_projected"] = std::to_string(semantic?semantic->labels_projected:0);
    values["labels_skipped"] = std::to_string(semantic?semantic->labels_skipped:0);
    values["slope_phases"] = result->slope_phases.dump();
    values["counters"] = result->live.counters.dump();
    for (auto& [k, v]:values)
    {
      diagnostic_msgs::msg::KeyValue value;
      value.key = k;
      value.value = v;
      record.values.push_back(std::move(value));
    }
    message.status.push_back(std::move(record));
    diagnostics->publish(message);
  }
  void completion_tick()
  {
    try { drain_completion(); flush_deferred_ground(); }
    catch (const std::exception& error) {
      navigation_valid = false;
      reason = error.what();
      std::cerr << "Completion event: " << reason << '\n';
    }
  }
  // Vehicle-centred map window from the newest IMU pose and the static mount.
  void update_window()
  {
    if ((!(cfg.map_length > 0.) && !cfg.max_map_cells) || poses.empty() || !fixed) return;
    const M3 rotation = poses.back().q.toRotationMatrix()*fixed->block<3,3>(0,0).transpose();
    const V3 origin = poses.back().p-rotation*fixed->block<3,1>(0,3);
    window = make_window(cfg, origin.head<2>());
    const int64_t wall = steady_ns();
    if (semantic && wall - last_prune_wall >= 1000000000LL) {
      last_prune_wall = wall;
      semantic->prune(window);
      front_free_region.prune(window);
      // Frames older than every retention window can never be revisions again.
      const int64_t cutoff = poses.back().stamp - 10000000000LL;
      for (auto it = submitted.begin(); it != submitted.end();)
        it = it->second < cutoff?submitted.erase(it):std::next(it);
    }
  }
  void input_tick()
  {
    try{
      drain_completion();
      flush_deferred_ground();
      drop_recovered_geometry();
      std::string why;
      bool vio_good = vio_ready(why);
      if (!initialized)
      {
        for (auto& q:pending)
          q.clear();
        if (vio_good&&factory_valid&&stationary())
        {
          if (!still_since)
            still_since = steady_ns();
          if (steady_ns() - still_since >= 2000000000)
          {
            if (cfg.startup_free_region_enabled &&
                !startup_free_region.initialize_from_imu(poses.back(),*fixed)) {
              reason = "Waiting for valid initial base_link pose for startup free region";
              return;
            }
            if (cfg.startup_person_cost_overlay)
              semantic->grid.seed_startup_person_cells(startup_free_region.original_cells());
            initialized = true;
            std::cout << "{\"event\":\"mapping_initialized\",\"stamp_ns\":" << poses.back().stamp << "}" << std::endl;
          }
        }
        else
          still_since = 0;
        return;
      }
      if (!vio_good)
      {
        for (auto& q:pending)
          q.clear();
        navigation_valid = false;
        return;
      }
      update_window();
      process_geometry(0);
      drain_completion();
      process_geometry(1);
      drain_completion();
      if (info_valid&&semantic)
      {
        for (auto& frame:semantic->process(poses, labels, zed_mount, camera_k, image_width, image_height))
          enqueue_ground(std::move(frame));
      }
      drain_completion();
      publish_cost();
    }
    catch (const std::exception& e){
      navigation_valid = false;
      reason = e.what();
      std::cerr << "Map tick: " << reason << '\n';
    }
  }
public:
  TerrainMapper(const Json& semantic_cfg, const Json& slope_cfg, const Json& calibration, double oak_limit_m)
    : Node("nomad_costmap_cpp"), cfg(Config::load(semantic_cfg, slope_cfg)), startup_free_region(cfg), front_free_region(cfg),
      tf_buffer(get_clock()), tf_listener(tf_buffer)
  {
    if (calibration.value("source", "") != "real_sensor"||calibration.value("camera_frame", "") != "oak_cam_b_optical_frame"||calibration.value("lidar_frame", "") != "g2_laser"||calibration.value("convention", "") != "p_camera_optical = R_camera_lidar * p_lidar + t_camera_lidar_m")
      throw std::runtime_error("LiDAR calibration provenance/frames mismatch");
    if (!std::isfinite(oak_limit_m) || oak_limit_m < 0.1 || oak_limit_m > 30.0)
      throw std::runtime_error("oak_max_horizontal_distance_m must be finite and within 0.1..30m");
    oak_max_horizontal_distance_m = oak_limit_m;
    RCLCPP_INFO(get_logger(), "OAK horizontal accumulation limit=%.3fm vehicle_origin=capture_time forward=vehicle XY front_half_circle=true lidar_unchanged=true", oak_max_horizontal_distance_m);
    RCLCPP_INFO(get_logger(), "OAK/LiDAR height gate enabled=%d xy_radius_m=%.4f reject_abs_z_difference_cm=%.4f", int(cfg.oak_lidar_height_gate), cfg.oak_lidar_xy_radius, cfg.oak_lidar_z_reject*100.);
    camera_lidar = calibration_matrix(calibration);
    auto context = get_node_base_interface()->get_context();
    input_event = std::make_shared<ExecutorEvent>(context, [this] { input_tick(); });
    result_event = std::make_shared<ExecutorEvent>(context, [this] { completion_tick(); });
    get_node_waitables_interface()->add_waitable(input_event, nullptr);
    get_node_waitables_interface()->add_waitable(result_event, nullptr);
    grids[0] = create_publisher<Grid>("/nomad/navigation_costmap", map_qos());
    grids[1] = create_publisher<Grid>("/nomad/navigation_local_costmap", map_qos());
    grids[2] = create_publisher<Grid>("/nomad/camera_observation_support", map_qos());
    layers = create_publisher<Layers>("/nomad/planning_cost_layers", map_qos());
    diagnostic_layers = create_publisher<Layers>("/nomad/semantic_validation/cost_layers", map_qos());
    status = create_publisher<Status>("/nomad/perception/status", rclcpp::QoS(1).reliable());
    diagnostics = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/nomad/semantic_validation/capture_timing", map_qos());
    std::array<std::string, 4> names{"slope_score", "semantic_score", "slope_deg", "slope_magnitude_deg"};
    for (int i = 0; i < 4; ++i)
      images[i] = create_publisher<sensor_msgs::msg::Image>("/nomad/semantic_validation/" + names[i], map_qos());
    subscriptions.push_back(create_subscription<std_msgs::msg::String>("/oak/calibration/factory", map_qos(), [this](std_msgs::msg::String::SharedPtr m){
      on_factory(*m);
      input_event->notify();
    }));
    subscriptions.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(cfg.semantic.at("topics").at("camera_info"), sensor_qos(), [this](sensor_msgs::msg::CameraInfo::SharedPtr m){
      on_info(*m);
      input_event->notify();
    }));
    subscriptions.push_back(create_subscription<sensor_msgs::msg::Image>(cfg.semantic.at("topics").at("labels"), rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::Image::SharedPtr m){
      on_labels(*m);
      input_event->notify();
    }));
    subscriptions.push_back(create_subscription<Status>("/nomad/localization/status", status_qos(), [this](Status::SharedPtr m){
      on_localization(*m);
    }));
    subscriptions.push_back(create_subscription<nav_msgs::msg::Odometry>("/nomad/localization/odometry", status_qos(), [this](nav_msgs::msg::Odometry::SharedPtr m){
      on_odometry(*m);
    }));
    subscriptions.push_back(create_subscription<sensor_msgs::msg::LaserScan>("/scan", sensor_qos(), [this](sensor_msgs::msg::LaserScan::SharedPtr m){
      on_scan(*m);
    }));
    subscriptions.push_back(create_subscription<sensor_msgs::msg::PointCloud>("/point_cloud", sensor_qos(), [this](sensor_msgs::msg::PointCloud::SharedPtr m){
      on_cloud(*m);
    }));
    subscriptions.push_back(create_subscription<sensor_msgs::msg::PointCloud2>("/oak/depth/points", sensor_qos(), [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
      on_oak(*m);
    }));
    // Housekeeping only. Input and completion delivery trigger the events.
    input_timer = create_wall_timer(std::chrono::milliseconds(100), [this]{
      input_event->notify();
    });
    status_timer = create_wall_timer(std::chrono::milliseconds(100), [this]{
      status_tick();
    });
    worker = std::thread([this]{ worker_main(); });
    RCLCPP_INFO(get_logger(), "terrain_mapper_started single_process=true map_length_m=%.2f slope_angle=%s startup_seed_frozen=%d freeze_after_first_valid=%d label_accumulation=%d max_labels=%d keep_slope_on_failure=%d cost_area=%s max_map_cells=%zu keep_radius_m=%.1f slope_score_per_deg=%.3f slope_block_deg=%.3f",
                cfg.map_length, cfg.gravity_slope_angle?"gravity":"heading", int(cfg.startup_seed_frozen),
                int(cfg.freeze_slope_after_first_valid), int(cfg.label_accumulation), cfg.label_accumulation_max,
                int(cfg.keep_slope_on_failure), cfg.cost_known_cells?"known_cells":"map_window", cfg.max_map_cells, cfg.map_keep_radius,
                cfg.slope_score_per_deg, cfg.slope_block_deg);
  }
  ~TerrainMapper() override
  {
    input_timer.reset();
    status_timer.reset();
    {
      std::lock_guard<std::mutex> lock(inbox_mutex);
      worker_stop = true;
    }
    worker_cv.notify_all();
    if (worker.joinable()) worker.join();
    get_node_waitables_interface()->remove_waitable(result_event, nullptr);
    get_node_waitables_interface()->remove_waitable(input_event, nullptr);
  }
};
}  // namespace
}  // namespace nomad

int main(int argc, char** argv)
{
  std::fesetround(FE_TONEAREST);
  try{
    std::map<std::string, std::string> args;
    for (int i = 1; i + 1 < argc; ++i)
      if (std::string(argv[i]).rfind("--", 0) == 0 && std::string(argv[i]) != "--ros-args") { args[argv[i]] = argv[i + 1]; ++i; }
    auto get = [&](const char* key) {
      auto it = args.find(key);
      if (it == args.end()) throw std::runtime_error(std::string("Missing ") + key);
      return it->second;
    };
    auto semantic = nomad::read_json(get("--semantic-config"));
    auto slope = nomad::read_json(get("--slope-config"));
    double oak_limit_m = 2.0;
    // terrain.json (GUI-edited, 2026-11-13): only the new options plus the slope
    // freeze switches are overridden; the legacy runtime files stay unchanged.
    if (args.count("--terrain-config")) {
      const auto terrain = nomad::read_json(args.at("--terrain-config"));
      nomad::apply_oak_height_override(terrain, slope);
      static const std::set<std::string> slope_keys = {"map_length_m", "slope_angle_reference", "startup_seed_keep_frozen",
        "freeze_slope_after_first_valid", "retry_slope_score_at_least_100",
        "keep_slope_on_failure", "cost_area", "max_map_cells", "map_keep_radius_m",
        "slope_score_per_deg", "slope_block_deg"};
      for (auto it = terrain.begin(); it != terrain.end(); ++it) {
        if (it.key().rfind("_", 0) == 0) continue;   // comments
        if (it.key() == "oak_max_horizontal_distance_m") oak_limit_m = it.value().get<double>();
        else if (it.key() == "oak_lidar_max_abs_z_difference_cm") continue; // applied above
        else if (slope_keys.count(it.key())) slope[it.key()] = it.value();
        else if (it.key() == "label_accumulation") semantic["local_ground"]["label_accumulation"] = it.value();
        else throw std::runtime_error("Unknown terrain.json key: " + it.key());
      }
    }
    const auto calibration = nomad::read_json(get("--lidar-calibration"));
    int ros_argc = 1;
    rclcpp::init(ros_argc, argv);
    auto mapper = std::make_shared<nomad::TerrainMapper>(semantic, slope, calibration, oak_limit_m);
    rclcpp::spin(mapper);
    mapper.reset();
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e){
    std::cerr << "Native costmap failed: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 2;
  }
}

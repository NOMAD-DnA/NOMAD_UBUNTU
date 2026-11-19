#pragma once
// In-process port of the legacy nomad_monitor input checks (monitor.cpp).
// The old monitor was a separate process that wrote health.json every 100 ms
// and the mapper read that file back. Same specs, ages and reasons; no file.
// Only the input part is kept: the mapper used inputs_ready/input_reason only.
#include "nomad_costmap/ros_util.hpp"
#include <deque>
#include <map>

namespace nomad {
class InputHealth {
public:
  struct Spec {
    const char* key;
    const char* topic;
    double age;
    const char* frame;
    bool input;
  };
  struct Sample {
    int64_t capture = 0, receipt = 0, valid = 0;
    std::string error, detail;
    int state = -1;
  };
  struct Result {
    bool inputs_ready = false;
    std::string input_reason = "Waiting for native input monitor";
    int64_t updated_wall = 0;
  };
  static const std::vector<Spec>& specs() {
    static const std::vector<Spec> s = {
      {"oak_points", "/oak/depth/points", 1.5, "oak_cam_a_optical_frame", true},
      {"scan", "/scan", .8, "g2_laser", true}, {"cloud", "/point_cloud", .8, "g2_laser", true},
      {"pose", "/nomad/localization/odometry", .6, "odom", true},
      {"localization", "/nomad/localization/status", 1., "odom", true},
      {"mask", "/perception/semantic/class_ids", 2., "zed2_23472639_left_rectified_optical_frame", true},
      {"rgb_info", "/zed/zed_node/left/camera_info", 2., "zed2_23472639_left_rectified_optical_frame", true},
      // Outputs: only the duplicate-publisher check uses them.
      {"global_map", "/nomad/navigation_costmap", 2., "odom", false},
      {"local_map", "/nomad/navigation_local_costmap", 2., "odom", false},
      {"layers", "/nomad/planning_cost_layers", 2., "odom", false},
      {"support", "/nomad/camera_observation_support", 2., "odom", false},
      {"perception", "/nomad/perception/status", 1., "odom", false}};
    return s;
  }
  void observe(const std::string& key, const std_msgs::msg::Header& h)
  {
    observe(key, h, Sample());
  }
  void observe(const std::string& key, const std_msgs::msg::Header& h, Sample value)
  {
    const Spec& s = spec(key);
    value.capture = stamp(h);
    value.receipt = steady_ns();
    if (value.capture <= 0||h.frame_id != s.frame)
      value.error = "Wrong frame or nonpositive capture stamp";
    auto previous = samples_.find(key);
    if (previous != samples_.end()&&value.capture < previous->second.capture)
      clock_fault_ = key + " capture clock moved backwards";
    samples_[key] = value;
    if ((key == "scan"||key == "cloud")&&value.error.empty())
    {
      auto& row = history_[key];
      if (row.empty()||row.back() != value.capture)
        row.push_back(value.capture);
      while (row.size() > 96)
        row.pop_front();
    }
  }
  void observe_status(const char* key, const Status& m)
  {
    Sample s;
    s.valid = int64_t(m.valid_for.sec) * 1000000000 + m.valid_for.nanosec;
    s.state = m.state;
    s.detail = m.detail;
    observe(key, m.header, s);
  }
  // Same rules as Monitor::tick() for inputs_ready / input_reason.
  Result evaluate(rclcpp::Node& node, tf2_ros::Buffer& buffer)
  {
    int64_t now = node.now().nanoseconds(), wall = steady_ns();
    if (last_now_&&now < last_now_ - 50000000)
      clock_fault_ = "ROS clock moved backwards";
    last_now_ = now;
    std::map<std::string, std::pair<bool, std::string> > checks;
    std::vector<std::string> duplicates;
    for (auto& s:specs())
    {
      bool ok = false;
      std::string reason = "No data";
      auto it = samples_.find(s.key);
      if (it != samples_.end())
      {
        auto& m = it->second;
        int64_t elapsed = now - m.capture;
        ok = m.error.empty()&&elapsed >= -50000000&&elapsed <= int64_t(s.age * 1e9)&&wall - m.receipt >= 0&&wall - m.receipt <= int64_t(s.age * 1e9);
        reason = !m.error.empty()?m.error:ok?"Fresh capture":"Capture or receipt expired";
        if (m.state >= 0)
        {
          ok = ok&&m.state == Status::READY&&m.valid > 0&&m.valid <= 1000000000&&elapsed <= m.valid;
          if (!ok)
            reason = m.detail.empty()?"Module is not READY or its status expired":m.detail;
        }
      }
      checks[s.key] = {ok, reason};
      if (node.count_publishers(s.topic) > 1)
        duplicates.push_back(s.topic);
    }
    bool matched = false;
    for (auto a:history_["scan"])
      if (now - a >= -50000000&&now - a <= 800000000)
        for (auto b:history_["cloud"])
          if (a == b&&now - b >= -50000000&&now - b <= 800000000)
            matched = true;
    checks["lidar_match"] = {matched, matched?"Capture pair available":"No capture pair in tolerance"};
    bool tf_ok = false;
    std::string tf_reason = "Waiting for calibrated TF";
    try{
      auto fixed = buffer.lookupTransform("base_link", "oak_imu_frame", tf2::TimePointZero);
      auto moving = buffer.lookupTransform("odom", "base_link", tf2::TimePointZero);
      int64_t age = now - stamp(moving.header);
      tf_ok = stamp(fixed.header) == 0&&age >= -50000000&&age <= 600000000;
      tf_reason = stamp(fixed.header) == 0?"Static IMU mount and fresh odom TF":"IMU mount TF must be static";
    }
    catch (const std::exception& e){
      tf_reason = "TF: " + std::string(e.what());
    }
    checks["tf"] = {tf_ok, tf_reason};
    std::vector<std::string> input_keys;
    for (auto& s:specs())
      if (s.input)
        input_keys.push_back(s.key);
    input_keys.insert(input_keys.end(), {"lidar_match", "tf"});
    Result r;
    r.inputs_ready = true;
    r.input_reason = "Live input checks passed";
    for (auto& key:input_keys)
      if (!checks.at(key).first)
      {
        if (r.inputs_ready)
          r.input_reason = key + ": " + checks.at(key).second;
        r.inputs_ready = false;
      }
    r.inputs_ready = r.inputs_ready&&clock_fault_.empty()&&duplicates.empty();
    if (!clock_fault_.empty() && r.input_reason == "Live input checks passed") r.input_reason = clock_fault_;
    if (!duplicates.empty() && r.input_reason == "Live input checks passed") {
      r.input_reason = "Duplicate publisher:";
      for (auto& s:duplicates) r.input_reason += " " + s;
    }
    r.updated_wall = wall;
    return r;
  }
private:
  const Spec& spec(const std::string& key) const
  {
    for (auto& s:specs())
      if (s.key == key)
        return s;
    throw std::runtime_error("Unknown monitor field");
  }
  std::map<std::string, Sample> samples_;
  std::map<std::string, std::deque<int64_t> > history_;
  std::string clock_fault_;
  int64_t last_now_ = 0;
};
}

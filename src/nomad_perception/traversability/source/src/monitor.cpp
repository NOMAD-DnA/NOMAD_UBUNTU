#include "nomad_costmap/ros.hpp"
#include <iostream>

namespace nomad {
  struct Spec {
    const char* key;
    const char* topic;
    double age;
    const char* frame;
    bool input;
  };
  const std::vector<Spec> specs = {
    {"oak_points", "/oak/depth/points", 1.5, "oak_cam_a_optical_frame", true},
    {"scan", "/scan", .8, "g2_laser", true}, {"cloud", "/point_cloud", .8, "g2_laser", true},
    {"pose", "/nomad/localization/odometry", .6, "odom", true},
    {"localization", "/nomad/localization/status", 1., "odom", true},
    {"mask", "/perception/semantic/class_ids", 2., "zed2_23472639_left_rectified_optical_frame", true},
    {"rgb_info", "/zed/zed_node/left/camera_info", 2., "zed2_23472639_left_rectified_optical_frame", true},
    {"global_map", "/nomad/navigation_costmap", 2., "odom", false},
    {"local_map", "/nomad/navigation_local_costmap", 2., "odom", false},
    {"layers", "/nomad/planning_cost_layers", 2., "odom", false},
    {"support", "/nomad/camera_observation_support", 2., "odom", false},
    {"perception", "/nomad/perception/status", 1., "odom", false}};
  struct Sample {
    int64_t capture = 0, receipt = 0, valid = 0;
    std::string error, detail;
    int state = -1;
    Json signature;
  };
  static Json signature(const Grid& g)
  {
    auto& p = g.info.origin.position;
    auto& q = g.info.origin.orientation;
    return {stamp(g.header), g.header.frame_id, g.info.width, g.info.height, g.info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w};
  }
  class Monitor : public rclcpp::Node {
    Arguments args;
    std::map<std::string, Sample> samples;
    std::map<std::string, std::deque<int64_t> > history;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    rclcpp::TimerBase::SharedPtr timer;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    std::string clock_fault;
    int64_t last_now = 0;
    const Spec& spec(const std::string& key)
    {
      for (auto& s:specs)
        if (s.key == key)
          return s;
      throw std::runtime_error("Unknown monitor field");
    }
    void observe(const std::string& key, const std_msgs::msg::Header& h, Sample value = {})
    {
      auto s = spec(key);
      value.capture = stamp(h);
      value.receipt = steady_ns();
      if (value.capture <= 0||h.frame_id != s.frame)
        value.error = "Wrong frame or nonpositive capture stamp";
      auto previous = samples.find(key);
      if (previous != samples.end()&&value.capture < previous->second.capture)
        clock_fault = key + " capture clock moved backwards";
      samples[key] = value;
      if ((key == "scan"||key == "cloud")&&value.error.empty())
      {
        auto& row = history[key];
        if (row.empty()||row.back() != value.capture)
          row.push_back(value.capture);
        while (row.size() > 96)
          row.pop_front();
      }
    }
    void status_observe(const char* key, const Status& m)
    {
      Sample s;
      s.valid = int64_t(m.valid_for.sec) * 1000000000 + m.valid_for.nanosec;
      s.state = m.state;
      s.detail = m.detail;
      observe(key, m.header, s);
    }
    void tick()
    {
      int64_t now = this->now().nanoseconds(), wall = steady_ns();
      if (last_now&&now < last_now - 50000000)
        clock_fault = "ROS clock moved backwards";
      last_now = now;
      Json checks = Json::object();
      std::vector<std::string> duplicates;
      for (auto& s:specs)
      {
        bool ok = false;
        std::string reason = "No data";
        Json age = nullptr;
        auto it = samples.find(s.key);
        if (it != samples.end())
        {
          auto& m = it->second;
          int64_t elapsed = now - m.capture;
          age = elapsed / 1e9;
          ok = m.error.empty()&&elapsed >= -50000000&&elapsed <= int64_t(s.age * 1e9)&&wall - m.receipt >= 0&&wall - m.receipt <= int64_t(s.age * 1e9);
          reason = !m.error.empty()?m.error:ok?"Fresh capture":"Capture or receipt expired";
          if (m.state >= 0)
          {
            ok = ok&&m.state == Status::READY&&m.valid > 0&&m.valid <= 1000000000&&elapsed <= m.valid;
            if (!ok)
              reason = m.detail.empty()?"Module is not READY or its status expired":m.detail;
          }
        }
        checks[s.key] = {{"ok", ok}, {"reason", reason}, {"capture_age_s", age}, {"topic", s.topic}};
        if (count_publishers(s.topic) > 1)
          duplicates.push_back(s.topic);
      }
      bool matched = false;
      for (auto a:history["scan"])
        if (now - a >= -50000000&&now - a <= 800000000)
          for (auto b:history["cloud"])
            if (a == b&&now - b >= -50000000&&now - b <= 800000000)
              matched = true;
      checks["lidar_match"] = {{"ok", matched}, {"reason", matched?"Capture pair available":"No capture pair in tolerance"}};
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
      checks["tf"] = {{"ok", tf_ok}, {"reason", tf_reason}};
      std::vector<std::string> input_keys;
      for (auto& s:specs)
        if (s.input)
          input_keys.push_back(s.key);
      input_keys.insert(input_keys.end(), {"lidar_match", "tf"});
      bool inputs = true;
      std::string input_reason = "Live input checks passed";
      for (auto& key:input_keys)
        if (!checks.at(key).at("ok").get<bool>())
        {
          if (inputs)
            input_reason = key + ": " + checks.at(key).at("reason").get<std::string>();
          inputs = false;
        }
      inputs = inputs&&clock_fault.empty()&&duplicates.empty();
      bool coherent = true;
      Json first;
      for (auto key:{"global_map", "local_map", "layers", "support"})
      {
        if (!checks.at(key).at("ok").get<bool>())
        {
          coherent = false;
          break;
        }
        auto sig = samples.at(key).signature;
        if (first.is_null())
          first = sig;
        else if (sig != first)
          coherent = false;
      }
      checks["map_bundle"] = {{"ok", coherent}, {"reason", coherent?"Same map generation":"Waiting for coherent Planning map bundle"}};
      bool ready = inputs&&coherent;
      std::string reason = input_reason;
      if (inputs)
      {
        reason = "All live checks passed";
        for (auto& s:specs)
          if (!checks.at(s.key).at("ok").get<bool>())
          {
            ready = false;
            if (reason == "All live checks passed")
              reason = std::string(s.key) + ": " + checks.at(s.key).at("reason").get<std::string>();
          }
        if (!coherent&&reason == "All live checks passed")
          reason = checks.at("map_bundle").at("reason");
      }
      if (!clock_fault.empty())
        reason = clock_fault;
      if (!duplicates.empty())
      {
        reason = "Duplicate publisher:";
        for (auto& s:duplicates)
          reason += " " + s;
      }
      Json report = {{"run_id", args.get("--run-id")}, {"release_id", args.get("--release-id", "nomad-costmap-cpp-20261105-1")}, {"updated_monotonic_ns", wall}, {"updated_wall_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()}, {"inputs_ready", inputs}, {"input_reason", input_reason}, {"ready", ready}, {"reason", reason}, {"fatal", !clock_fault.empty()||!duplicates.empty()}, {"checks", checks}, {"implementation", "C++17"}};
      atomic_json(args.get("--output"), report);
    }
public:
    explicit Monitor(const Arguments& a) : Node("nomad_jetson_run_monitor_cpp"), args(a), buffer(get_clock()), listener(buffer)
    {
      subscriptions.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(spec("oak_points").topic, sensor_qos(), [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
        Sample s;
        bool xyz = true;
        for (auto name:{"x", "y", "z"})
          xyz &= std::any_of(m->fields.begin(), m->fields.end(), [&](auto& f){
            return f.name == name;
          });
        if (!xyz||m->width * m->height == 0)
          s.error = "Missing OAK XYZ fields or empty cloud";
        observe("oak_points", m->header, s);
      }));
      subscriptions.push_back(create_subscription<sensor_msgs::msg::LaserScan>(spec("scan").topic, sensor_qos(), [this](sensor_msgs::msg::LaserScan::SharedPtr m){
        observe("scan", m->header);
      }));
      subscriptions.push_back(create_subscription<sensor_msgs::msg::PointCloud>(spec("cloud").topic, sensor_qos(), [this](sensor_msgs::msg::PointCloud::SharedPtr m){
        observe("cloud", m->header);
      }));
      subscriptions.push_back(create_subscription<nav_msgs::msg::Odometry>(spec("pose").topic, status_qos(), [this](nav_msgs::msg::Odometry::SharedPtr m){
        Sample s;
        if (m->child_frame_id != "base_link")
          s.error = "Expected odom -> base_link pose";
        observe("pose", m->header, s);
      }));
      subscriptions.push_back(create_subscription<Status>(spec("localization").topic, status_qos(), [this](Status::SharedPtr m){
        status_observe("localization", *m);
      }));
      subscriptions.push_back(create_subscription<sensor_msgs::msg::Image>(spec("mask").topic, sensor_qos(), [this](sensor_msgs::msg::Image::SharedPtr m){
        Sample s;
        if (m->width != 640||m->height != 360||m->encoding != "mono8")
          s.error = "Wrong image dimensions or encoding";
        observe("mask", m->header, s);
      }));
      subscriptions.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(spec("rgb_info").topic, sensor_qos(), [this](sensor_msgs::msg::CameraInfo::SharedPtr m){
        Sample s;
        if (m->width != 1280||m->height != 720||m->k[0] <= 0||m->k[4] <= 0)
          s.error = "Invalid camera model";
        observe("rgb_info", m->header, s);
      }));
      for (auto key:{"global_map", "local_map", "support"})
        subscriptions.push_back(create_subscription<Grid>(spec(key).topic, map_qos(), [this, key](Grid::SharedPtr m){
          Sample s;
          s.signature = signature(*m);
          observe(key, m->header, s);
        }));
      subscriptions.push_back(create_subscription<Layers>(spec("layers").topic, map_qos(), [this](Layers::SharedPtr m){
        Sample s;
        s.signature = signature(m->grid);
        observe("layers", m->grid.header, s);
      }));
      subscriptions.push_back(create_subscription<Status>(spec("perception").topic, status_qos(), [this](Status::SharedPtr m){
        status_observe("perception", *m);
      }));
      timer = create_wall_timer(std::chrono::milliseconds(100), [this]{
        tick();
      });
    }
  };
}
int main(int argc, char** argv)
{
  try{
    nomad::Arguments args(argc, argv);
    int n = 1;
    rclcpp::init(n, argv);
    auto node = std::make_shared<nomad::Monitor>(args);
    rclcpp::spin(node);
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e){
    std::cerr << "Native monitor failed: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 2;
  }
}

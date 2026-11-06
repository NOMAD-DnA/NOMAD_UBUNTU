#include "nomad_costmap/ros.hpp"
#include <unistd.h>
#include <iostream>

namespace nomad {
  int run_vio(const Arguments& args)
  {
    auto node = std::make_shared<rclcpp::Node>("nomad_vio_receiver_" + std::to_string(getpid()));
    tf2_ros::Buffer buffer(node->get_clock());
    tf2_ros::TransformListener listener(buffer);
    Channel pipe(std::stoi(args.get("--ipc-fd")));
    pipe.start([](Json){
    });
    Json localization = Json::object();
    std::optional<M4> fixed;
    std::optional<Pose> last;
    int64_t last_now = 0, pose_wall = 0;
    uint64_t rejected = 0;
    int fast = 0;
    std::string error, reason = "Waiting for Pi5 localization";
    auto status = node->create_subscription<Status>("/nomad/localization/status", status_qos(), [&](Status::SharedPtr m){
      localization = status_json(*m, steady_ns());
    });
    auto pose = node->create_subscription<nav_msgs::msg::Odometry>("/nomad/localization/odometry", status_qos(), [&](nav_msgs::msg::Odometry::SharedPtr m){
      if (!error.empty())
        return;
      try{
        int64_t ns = stamp(m->header), now = node->now().nanoseconds(), wall = steady_ns();
        if (!status_ready(localization, now, wall))
          throw std::runtime_error("Pi5 localization not READY");
        if (m->header.frame_id != "odom"||m->child_frame_id != "base_link"||ns <= 0||(last&&ns <= last->stamp)||now - ns < -50000000||now - ns > 600000000)
          throw std::runtime_error("Wrong frame, stale/future or reordered Pi5 pose");
        auto tf = buffer.lookupTransform("base_link", "oak_imu_frame", tf2::TimePointZero);
        if (tf.header.frame_id != "base_link"||tf.child_frame_id != "oak_imu_frame"||stamp(tf.header) != 0)
          throw std::runtime_error("Expected static base_link <- oak_imu_frame");
        M4 mount = tf_matrix(tf.transform);
        if (fixed&&(mount - *fixed).cwiseAbs().maxCoeff() > 1e-9)
        {
          error = "Fixed IMU transform changed; restart with empty map";
          return;
        }
        const auto& p = m->pose.pose.position;
        const auto& q = m->pose.pose.orientation;
        Pose body = pose_from_json(Json{ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w});
        Pose raw = matrix_pose(ns, pose_matrix(body) * mount);
        if (last)
        {
          double speed = (raw.p - last->p).norm() / ((ns - last->stamp) / 1e9);
          fast = speed > 10?fast + 1:0;
          if (fast >= 5)
          {
            error = "VIO divergence: speed exceeded 10m/s for 5 samples";
            return;
          }
        }
        fixed = mount;
        last = raw;
        pose_wall = wall;
        reason = "Pi5 capture-time pose and static IMU transform available";
        pipe.send({{"kind", "pose"}, {"row", pose_json(raw)}, {"received_monotonic_ns", wall}});
      }
      catch (const std::exception& e){++rejected;
                                      reason = e.what();
      }
    });
    auto heartbeat = node->create_wall_timer(std::chrono::milliseconds(20), [&]{
      int64_t now = node->now().nanoseconds(), wall = steady_ns();
      if (last_now&&now < last_now - 50000000)
        error = "Pi5 receiver ROS clock moved backwards";
      last_now = now;
      bool ready = error.empty()&&status_ready(localization, now, wall)&&fixed&&last&&now - last->stamp >= -50000000&&now - last->stamp <= 600000000&&wall - pose_wall <= 1000000000;
      pipe.send({{"kind", "vio_state"}, {"heartbeat_ns", wall}, {"ready", ready}, {"reason", error.empty()?reason:error}, {"error", error}, {"rejected", rejected}, {"pose_stamp", last?last->stamp:0}, {"pose_wall_ns", pose_wall}, {"fixed", fixed?matrix_json(*fixed):Json(nullptr)}, {"status", localization}});
    });
    while (rclcpp::ok()&&pipe.fault().empty())
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pipe.fault().empty()?0:2;
  }

  int run_points(const Arguments& args)
  {
    auto node = std::make_shared<rclcpp::Node>("nomad_point_receiver_" + std::to_string(getpid()));
    M4 calibration = calibration_matrix(read_json(args.get("--lidar-calibration")));
    Channel pipe(std::stoi(args.get("--ipc-fd")));
    pipe.start([](Json){
    });
    std::map<int64_t, Json> pairs;
    std::deque<int64_t> insertion;
    std::array<int64_t, 2> capture{0, 0}, received{0, 0};
    Json counts = {{"scan", 0}, {"cloud", 0}, {"oak", 0}, {"paired_lidar", 0}, {"invalid_lidar", 0}, {"invalid_oak", 0}, {"unmatched_pairs_expired", 0}};
    auto increment = [&](const char* key){
                       counts[key] = counts.at(key).get<uint64_t>() + 1;
                     };
    auto join = [&](int64_t ns, const char* kind, Json value){
                  if (!pairs.count(ns))
                    insertion.push_back(ns);
                  pairs[ns][kind] = std::move(value);
                  auto& item = pairs[ns];
                  if (item.size() == 2)
                  {
                    try{
                      auto selected = scan_pair(item.at("scan"), item.at("cloud"), calibration);
                      int64_t wall = steady_ns();
                      selected["received_monotonic_ns"] = wall;
                      selected["received_ros_ns"] = item.at("cloud").at("received_ros_ns");
                      selected["kind"] = "lidar";
                      pipe.send(selected);
                      capture[0] = ns;
                      received[0] = wall;
                      increment("paired_lidar");
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
                };
    auto scan = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", sensor_qos(), [&](sensor_msgs::msg::LaserScan::SharedPtr m){
      if (m->header.frame_id != "g2_laser"||stamp(m->header) <= 0)
        return;
      increment("scan");
      Json ranges = Json::array();
      for (auto r:m->ranges)
        ranges.push_back(std::isfinite(r)?Json(r):Json(nullptr));
      join(stamp(m->header), "scan", {{"stamp_ns", stamp(m->header)}, {"ranges", ranges}, {"angle_min", m->angle_min}, {"angle_increment", m->angle_increment}, {"range_min", m->range_min}, {"range_max", m->range_max}});
    });
    auto cloud = node->create_subscription<sensor_msgs::msg::PointCloud>("/point_cloud", sensor_qos(), [&](sensor_msgs::msg::PointCloud::SharedPtr m){
      if (m->header.frame_id != "g2_laser"||stamp(m->header) <= 0)
        return;
      increment("cloud");
      Json p = Json::array(), offsets = Json::array();
      for (auto& v:m->points)
        p.push_back({v.x, v.y, v.z});
      for (auto& c:m->channels)
        if (c.name == "stamps")
          offsets = c.values;
      join(stamp(m->header), "cloud", {{"stamp_ns", stamp(m->header)}, {"points", p}, {"offsets", offsets}, {"received_ros_ns", node->now().nanoseconds()}});
    });
    auto oak = node->create_subscription<sensor_msgs::msg::PointCloud2>("/oak/depth/points", sensor_qos(), [&](sensor_msgs::msg::PointCloud2::SharedPtr m){
      try{auto p = cloud_points(*m, "oak_cam_a_optical_frame");
          int64_t ns = stamp(m->header), wall = steady_ns();
          pipe.send({{"kind", "oak"}, {"stamp_ns", ns}, {"points", points_json(p)}, {"received_monotonic_ns", wall}, {"received_ros_ns", node->now().nanoseconds()}});
          capture[1] = ns;
          received[1] = wall;
          increment("oak");
      }
      catch (const std::exception&){increment("invalid_oak");
      }
    });
    auto timer = node->create_wall_timer(std::chrono::milliseconds(20), [&]{
      pipe.send({{"kind", "points_state"}, {"heartbeat_ns", steady_ns()}, {"captures", capture}, {"receipts", received}, {"counts", counts}, {"dds_depth", 5}});
    });
    while (rclcpp::ok()&&pipe.fault().empty())
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pipe.fault().empty()?0:2;
  }

  int run_slope(const Arguments& args)
  {
    auto cfg = Config::load(read_json(args.get("--semantic-config")), read_json(args.get("--slope-config")));
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<Json> work;
    std::string error;
    Channel pipe(std::stoi(args.get("--ipc-fd")));
    pipe.start([&](Json j){
      std::lock_guard<std::mutex> lock(mutex);
      if (work.size() >= 200)
      {
        error = "Slope observation queue overflow (200)";
        changed.notify_all();
        return;
      }
      work.push_back(std::move(j));
      changed.notify_all();
    });
    std::unique_ptr<Ground> ground;
    uint64_t sequence = 0, updates = 0;
    Json source_timing = Json::object();
    while (rclcpp::ok()&&pipe.fault().empty())
    {
      std::deque<Json> batch;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait_for(lock, std::chrono::milliseconds(20), [&]{
          return !work.empty()||!error.empty();
        });
        if (!error.empty())
          throw std::runtime_error(error);
        for (size_t i = 0; i < 32&&!work.empty(); ++i)
        {
          batch.push_back(std::move(work.front()));
          work.pop_front();
        }
      }
      if (batch.empty())
        continue;
      int64_t started = steady_ns();
      for (auto& j:batch)
      {
        if (!ground)
        {
          Mounts mounts;
          for (int s = 0; s < 2; ++s)
            mounts.sensor[s] = rigid(j.at("mounts")[s]);
          ground = std::make_unique<Ground>(cfg, mounts);
          auto axis = j.at("forward_axis");
          ground->forward_axis = V3(axis[0], axis[1], axis[2]);
        }
        auto f = frame_from_json(j.at("frame"));
        ground->ingest(f, j.at("revision"));
        ++updates;
        std::string name = f.source == 0?"lidar":"oak";
        source_timing[name] = {{"capture_ns", f.stamp}, {"enqueued_monotonic_ns", j.at("enqueued_ns")}, {"worker_queue_delay_ms", (started - j.at("enqueued_ns").get<int64_t>()) / 1e6}};
      }
      auto live = ground->snapshot();
      int64_t completed = steady_ns();
      Json timing = {{"sequence", ++sequence}, {"snapshot_capture_ns", live.at("snapshot_capture_ns")}, {"source_capture_ns", live.at("source_capture_ns")}, {"source_timing", source_timing}, {"updates", updates}, {"worker_compute_ms", (completed - started) / 1e6}, {"worker_completed_monotonic_ns", completed}};
      pipe.send({{"kind", "slope"}, {"live", live}, {"timing", timing}});
    }
    return pipe.fault().empty()?0:2;
  }
}

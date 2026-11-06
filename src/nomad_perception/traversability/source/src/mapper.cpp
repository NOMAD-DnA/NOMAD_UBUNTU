#include "nomad_costmap/ros.hpp"
#include "nomad_costmap/camera_model.hpp"
#include "nomad_costmap/executor_event.hpp"
#include "nomad_costmap/snapshot_writer.hpp"
#include <algorithm>
#include <iostream>
#include <unistd.h>

namespace nomad {
  namespace {
    class Mapper : public rclcpp::Node {
      Arguments args;
      Config cfg;
      M4 camera_lidar, imu_b = M4::Identity(), imu_a = M4::Identity(), zed_mount = M4::Identity();
      Mounts mounts;
      bool factory_valid = false, info_valid = false, initialized = false, clock_fault = false;
      RectifiedCameraModel rectified_model;
      int64_t last_now = 0, still_since = 0, last_capture = 0, last_map_wall = 0;
      std::string fatal, reason = "Waiting for Pi5 initialization";
      bool navigation_valid = false;
      std::mutex inbox_mutex;
      std::deque<Pose> incoming_poses;
      std::array<std::deque<Json>, 2> incoming;
      Json vio_state = Json::object(), points_state = Json::object();
      std::optional<Json> slope_result;
      uint64_t slope_sent = 0, slope_acknowledged = 0;
      std::vector<Pose> poses;
      std::array<std::deque<Json>, 2> pending;
      std::array<int64_t, 2> last_geometry{0, 0};
      std::deque<Label> labels;
      M3 camera_k = M3::Identity();
      int image_width = 1280, image_height = 720;
      std::unique_ptr<LocalGround> semantic;
      std::set<std::pair<int, int64_t> > submitted;
      Child vio, points, slope;
      std::unique_ptr<PublisherLock> publisher_lock;
      std::shared_ptr<const Json> live_result;
      Json last_semantics = Json::object();
      std::shared_ptr<ExecutorEvent> input_event, result_event;
      std::unique_ptr<SnapshotWriter> snapshot_writer;
      uint64_t live_version = 0, last_cost_version = 0;
      uint64_t semantic_version = UINT64_MAX, cached_semantic_version = UINT64_MAX;
      Json cached_semantics = Json::object();
      std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
      std::array<rclcpp::Publisher<Grid>::SharedPtr, 3> grids;
      rclcpp::Publisher<Layers>::SharedPtr layers, diagnostic_layers;
      rclcpp::Publisher<Status>::SharedPtr status;
      std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 4> images;
      rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
      rclcpp::TimerBase::SharedPtr input_timer, status_timer;
      std::filesystem::path output;
      uint64_t lidar_done = 0, oak_done = 0, geometry_dropped = 0;

      void receive(Json j)
      {
        std::lock_guard<std::mutex> lock(inbox_mutex);
        std::string kind = j.at("kind");
        if (kind == "pose")
        {
          if (incoming_poses.size() >= 10000)
          {
            fatal = "VIO history overrun (10000)";
            return;
          }
          incoming_poses.push_back(pose_from_json(j.at("row")));
        }
        else if (kind == "vio_state")
          vio_state = std::move(j);
        else if (kind == "points_state")
          points_state = std::move(j);
        else if (kind == "lidar"||kind == "oak")
        {
          int s = kind == "lidar"?0:1;
          size_t limit = s?8:80;
          if (incoming[s].size() >= limit)
          {
            fatal = "Point receiver FIFO overflow (" + std::to_string(limit) + ")";
            return;
          }
          incoming[s].push_back(std::move(j));
        }
        else if (kind == "slope")
        {
          j["timing"]["mapper_received_monotonic_ns"] = steady_ns();
          slope_acknowledged = j.at("timing").at("updates");
          slope_result = std::move(j); // Completed snapshots may coalesce; observations never do.
        }
        else
          fatal = "Unknown native IPC message";
        if (kind == "slope") result_event->notify();
        else input_event->notify();
      }
      void fail(const std::string& message)
      {
        std::lock_guard<std::mutex> lock(inbox_mutex);
        fatal = message;
      }
      void on_factory(const std_msgs::msg::String& m)
      {
        try{
          auto transforms = factory_mounts(Json::parse(m.data), cfg.semantic);
          if (factory_valid&&((transforms[0] - imu_b).cwiseAbs().maxCoeff() > 1e-9||(transforms[1] - imu_a).cwiseAbs().maxCoeff() > 1e-9))
          {
            fail("Factory calibration changed during mapping");
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
        std::array<bool,256> allowed{};
        for(auto it=cfg.semantic.at("classes").begin();it!=cfg.semantic.at("classes").end();++it)allowed[std::stoi(it.key())]=true;
        for(auto id:label.ids)if(!allowed[id]){reason="Unexpected semantic class ID; complete image rejected";return;}
        label_safe(label, cfg.semantic.at("boundary_margin_px"));
        labels.push_back(std::move(label));
        while (labels.size() > 96)
          labels.pop_front();
      }
      std::pair<bool, std::string> ready()
      {
        int64_t now = this->now().nanoseconds(), wall = steady_ns();
        Json v, p;
        std::string f;
        {
          std::lock_guard<std::mutex> lock(inbox_mutex);
          v = vio_state;
          p = points_state;
          f = fatal;
        }
        if (!f.empty())
          return {false, f}
        ;
        if (clock_fault)
          return {false, "ROS clock moved backwards; restart with empty map"}
        ;
        if (!vio.alive()||!points.alive()||!slope.alive())
          return {false, "Native receiver or slope worker stopped"}
        ;
        for (auto* child:{&vio, &points, &slope})
          if (!child->channel->fault().empty())
            return {false, child->channel->fault()}
        ;
        if (v.empty()||wall - v.value("heartbeat_ns", int64_t(0)) > 1000000000||!v.value("ready", false))
          return {false, v.value("reason", "Waiting for VIO receiver")}
        ;
        if (p.empty()||wall - p.value("heartbeat_ns", int64_t(0)) > 1000000000)
          return {false, "Point receiver heartbeat expired"}
        ;
        int64_t ns = p.at("captures")[0], rx = p.at("receipts")[0];
        if (ns <= 0||now - ns < -50000000||now - ns > 1000000000||wall - rx > 1000000000)
          return {false, "LiDAR paired capture or receipt expired"}
        ;
        if (!factory_valid||!info_valid)
          return {false, reason}
        ;
        if (!initialized)
          return {false, "Waiting for 5s of stationary VIO initialization"}
        ;
        try{
          auto health = read_json(args.get("--health-file"));
          int64_t age = wall - health.at("updated_monotonic_ns").get<int64_t>();
          if (health.at("run_id") != args.get("--run-id")||age < 0||age > 1000000000)
            return {false, "Runtime input report expired or belongs to another run"}
          ;
          if (!health.value("inputs_ready", false))
            return {false, health.value("input_reason", "Runtime inputs not ready")}
          ;
        }
        catch (const std::exception&){
          return {false, "Waiting for native input monitor"};
        }
        return {true, "Validated live input checks"};
      }
      void status_tick()
      {
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
        bool ok = navigation_valid&&r.first&&last_capture > 0&&now - last_capture >= -50000000&&now - last_capture <= 2000000000&&steady_ns() - last_map_wall <= 3000000000;
        Status m;
        set_stamp(m.header, now);
        m.valid_for.nanosec = 300000000;
        m.state = ok?Status::READY:last_capture?Status::STOPPED:Status::WAITING;
        m.detail = ok?reason:"Not ready: " + reason + "; capture/clock/wall freshness required";
        status->publish(m);
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
          if (slope_sent - slope_acknowledged >= 200)
          {
            fatal = "Slope observation queue overflow (200 outstanding)";
            throw std::runtime_error(fatal);
          }
          ++slope_sent;
        }
        auto identity = std::make_pair(frame.source, frame.stamp);
        bool revision = !submitted.insert(identity).second;
        Json fixed;
        {
          std::lock_guard<std::mutex> lock(inbox_mutex);
          fixed = vio_state.at("fixed");
        }
        V3 axis = rigid(fixed).block<3, 3>(0, 0).transpose() * V3::UnitX();
        slope.channel->send({{"frame", frame_json(frame)}, {"revision", revision}, {"mounts", {matrix_json(mounts.sensor[0]), matrix_json(mounts.sensor[1])}}, {"forward_axis", {axis.x(), axis.y(), axis.z()}}, {"enqueued_ns", steady_ns()}});
      }
      bool process_geometry(int source)
      {
        auto& queue = pending[source];
        bool changed = false;
        while (!queue.empty())
        {
          drain_completion();
          auto& item = queue.front();
          int64_t ns = item.at("stamp_ns");
          int64_t end = ns;
          if (source == 0)
          {
            const auto& offsets = item.at("point_time_offsets_s");
            if (!offsets.empty())
              end += nearest_even(offsets.back().get<double>() * 1e9);
          }
          if (poses.empty()||end > poses.back().stamp)
          {
            if (steady_ns() - item.at("received_monotonic_ns").get<int64_t>() < 1500000000)
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
              frame = deskew(item, poses, imu_b * camera_lidar);
            else{
              optical = points_from_json(item.at("points"));
              auto pose = interpolate(poses, ns);
              frame.stamp = ns;
              frame.source = 1;
              frame.pose = poses.back();
              frame.points = transform(optical, pose_matrix(pose) * imu_a);
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
      Grid make_grid(const Json& snapshot, const char* values)
      {
        Grid m;
        set_stamp(m.header, snapshot.at("capture_ns"));
        m.info.resolution = snapshot.at("resolution_m");
        m.info.width = snapshot.at("width");
        m.info.height = snapshot.at("height");
        m.info.origin.position.x = snapshot.at("origin_xy")[0];
        m.info.origin.position.y = snapshot.at("origin_xy")[1];
        m.info.origin.orientation.w = 1.;
        m.data = snapshot.at(values).get<std::vector<int8_t> >();
        return m;
      }
      void publish_cost()
      {
        if (!live_result||!semantic)
          return;
        const Json& live = live_result->at("live");
        auto now = this->now().nanoseconds();
        uint64_t version = semantic->grid.version(now);
        if (live_version == last_cost_version&&version == semantic_version)
          return;
        if (cached_semantic_version != version)
        {
          cached_semantics = semantic->grid.snapshot(now);
          cached_semantic_version = version;
        }
        Json key = Json::array();
        for (auto& row:cached_semantics.at("cells"))
          key.push_back({row.at("ix"), row.at("iy"), row.at("cost"), row.at("last_semantic_ns"), row.at("camera_observed_ns"), row.at("camera_capture_ns")});
        if (live_version == last_cost_version&&key == last_semantics)
        {
          semantic_version = version;
          return;
        }
        auto r = ready();
        if (!r.first)
        {
          navigation_valid = false;
          reason = r.second;
          return;
        }
        auto snapshot = build_cost(live, cached_semantics, cfg);
        last_semantics = std::move(key);
        semantic_version = version;
        last_cost_version = live_version;
        Grid global = make_grid(snapshot, "global_costmap"), local = make_grid(snapshot, "local_costmap"), support = make_grid(snapshot, "camera_support");
        Layers bundle;
        bundle.grid = global;
        for (auto& v:snapshot.at("slope_cost"))
          bundle.slope_cost.push_back(v.is_null()?std::numeric_limits<float>::quiet_NaN():v.get<float>());
        for (auto& v:snapshot.at("terrain_cost"))
          bundle.terrain_cost.push_back(v.is_null()?std::numeric_limits<float>::quiet_NaN():v.get<float>());
        std::array<const char*, 4> names{"slope_score", "semantic_score", "slope_deg", "slope_magnitude_deg"};
        for (int i = 0; i < 4; ++i)
        {
          sensor_msgs::msg::Image m;
          m.header = global.header;
          m.height = global.info.height;
          m.width = global.info.width;
          m.encoding = "32FC1";
          m.step = m.width * 4;
          m.data.resize(size_t(m.step) * m.height);
          size_t k = 0;
          for (auto& value:snapshot.at(names[i]))
          {
            float v = value.is_null()?std::numeric_limits<float>::quiet_NaN():value.get<float>();
            std::memcpy(m.data.data() + 4 * k++, &v, 4);
          }
          images[i]->publish(m);
        }
        diagnostic_layers->publish(bundle);
        int64_t capture = snapshot.at("capture_ns");
        if (capture <= last_capture)
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
        grids[2]->publish(support);
        layers->publish(bundle);
        grids[0]->publish(global);
        grids[1]->publish(local);
        last_capture = capture;
        last_map_wall = steady_ns();
        navigation_valid = true;
        reason = "Validated perception snapshot";
        status_tick();
      }
      void drain_completion()
      {
        std::optional<Json> completed;
        int64_t dispatch;
        {
          std::lock_guard<std::mutex> lock(inbox_mutex);
          completed = std::move(slope_result);
          slope_result.reset();
          dispatch = steady_ns();
        }
        if (!completed) return;
        auto result = std::make_shared<Json>(std::move(*completed));
        auto& timing = result->at("timing");
        timing["mapper_dispatch_monotonic_ns"] = dispatch;
        timing["mapper_event_wait_ms"] = (dispatch - timing.at("mapper_received_monotonic_ns").get<int64_t>()) / 1e6;
        timing["worker_to_mapper_receive_ms"] = (timing.at("mapper_received_monotonic_ns").get<int64_t>() - timing.at("worker_completed_monotonic_ns").get<int64_t>()) / 1e6;
        timing["receiver_counts"] = {{"lidar_processed", lidar_done}, {"oak_processed", oak_done}, {"geometry_dropped", geometry_dropped}};
        live_result = result;
        ++live_version;
        const int64_t previous = last_capture, started = steady_ns();
        publish_cost();
        timing["mapper_cost_output_ms"] = (steady_ns() - started) / 1e6;
        timing["mapper_published_new_capture"] = last_capture > previous;
        if (last_capture > previous)
          timing["worker_complete_to_map_publish_ms"] = (last_map_wall - timing.at("worker_completed_monotonic_ns").get<int64_t>()) / 1e6;
        if (snapshot_writer && !snapshot_writer->fault().empty())
          timing["snapshot_log_error"] = snapshot_writer->fault();
        diagnostic_msgs::msg::DiagnosticArray message;
        set_stamp(message.header, timing.at("snapshot_capture_ns"));
        diagnostic_msgs::msg::DiagnosticStatus record;
        record.name = "semantic_validation/timing";
        record.hardware_id = "native_cpp";
        record.level = timing.contains("snapshot_log_error") ? 1 : 0;
        record.message = "Capture provenance and event dispatch";
        for (auto it = timing.begin(); it != timing.end(); ++it)
        {
          diagnostic_msgs::msg::KeyValue value;
          value.key = it.key();
          value.value = it.value().dump();
          record.values.push_back(std::move(value));
        }
        message.status.push_back(std::move(record));
        diagnostics->publish(message);
        // No filesystem serialization or I/O runs on the map publication path.
        if (snapshot_writer) snapshot_writer->submit(std::move(result));
      }
      void completion_tick()
      {
        try { drain_completion(); }
        catch (const std::exception& error) {
          navigation_valid = false;
          reason = error.what();
          std::cerr << "Completion event: " << reason << '\n';
        }
      }
      void input_tick()
      {
        try{
          drain_completion();
          Json v;
          {
            std::lock_guard<std::mutex> lock(inbox_mutex);
            for (auto& pose:incoming_poses)
            {
              if (!poses.empty()&&pose.stamp <= poses.back().stamp)
              {
                fatal = "Reordered VIO IPC history";
                break;
              }
              poses.push_back(pose);
            }
            incoming_poses.clear();
            if (poses.size() > 10000)
              poses.erase(poses.begin(), poses.end() - 10000);
            v = vio_state;
            for (int source = 0; source < 2; ++source)
            {
              size_t limit = source?8:80;
              while (!incoming[source].empty())
              {
                if (initialized&&pending[source].size() >= limit)
                {
                  fatal = "Mapper geometry FIFO overflow (" + std::to_string(limit) + ")";
                  break;
                }
                pending[source].push_back(std::move(incoming[source].front()));
                incoming[source].pop_front();
                if (!initialized&&pending[source].size() > limit)
                  pending[source].pop_front();
              }
            }
          }
          bool vio_good = !v.empty()&&v.value("ready", false)&&steady_ns() - v.value("heartbeat_ns", int64_t(0)) <= 1000000000;
          if (!initialized)
          {
            for (auto& q:pending)
              q.clear();
            if (vio_good&&factory_valid&&stationary())
            {
              if (!still_since)
                still_since = steady_ns();
              if (steady_ns() - still_since >= 5000000000)
              {
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
      explicit Mapper(const Arguments& a) : Node("nomad_costmap_cpp"), args(a), cfg(Config::load(read_json(a.get("--semantic-config")), read_json(a.get("--slope-config"))))
      {
        auto calibration = read_json(args.get("--lidar-calibration"));
        bool virtual_mode = args.has("--virtual-environment");
        if (calibration.value("source", "") != (virtual_mode?"virtual_environment":"real_sensor")||calibration.value("camera_frame", "") != "oak_cam_b_optical_frame"||calibration.value("lidar_frame", "") != "g2_laser"||calibration.value("convention", "") != "p_camera_optical = R_camera_lidar * p_lidar + t_camera_lidar_m")
          throw std::runtime_error("LiDAR calibration provenance/frames mismatch");
        if (virtual_mode&&std::filesystem::path(args.get("--lidar-calibration")).filename() != "oak_lidar.virtual.json")
          throw std::runtime_error("Virtual calibration must be explicitly named");
        camera_lidar = calibration_matrix(calibration);
        output = args.get("--output");
        auto context = get_node_base_interface()->get_context();
        input_event = std::make_shared<ExecutorEvent>(context, [this] { input_tick(); });
        result_event = std::make_shared<ExecutorEvent>(context, [this] { completion_tick(); });
        get_node_waitables_interface()->add_waitable(input_event, nullptr);
        get_node_waitables_interface()->add_waitable(result_event, nullptr);
        if (!output.empty()) snapshot_writer = std::make_unique<SnapshotWriter>(output);

        publisher_lock = std::make_unique<PublisherLock>("/nomad/planning_cost_layers");
        grids[0] = create_publisher<Grid>("/nomad/navigation_costmap", map_qos());
        grids[1] = create_publisher<Grid>("/nomad/navigation_local_costmap", map_qos());
        grids[2] = create_publisher<Grid>("/nomad/camera_observation_support", map_qos());
        layers = create_publisher<Layers>("/nomad/planning_cost_layers", map_qos());
        diagnostic_layers = create_publisher<Layers>("/nomad/semantic_validation/cost_layers", map_qos());
        status = create_publisher<Status>("/nomad/perception/status", status_qos());
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
        auto callback = [this](Json j){
                          receive(std::move(j));
                        };
        std::string exe = executable_path();
        vio.launch({exe, "--mode", "vio"}, callback);
        points.launch({exe, "--mode", "points", "--lidar-calibration", args.get("--lidar-calibration")}, callback);
        slope.launch({exe, "--mode", "slope", "--semantic-config", args.get("--semantic-config"), "--slope-config", args.get("--slope-config")}, callback);
        // Housekeeping only. Input and completion delivery trigger ROS guards.
        input_timer = create_wall_timer(std::chrono::milliseconds(100), [this]{
          input_event->notify();
        });
        status_timer = create_wall_timer(std::chrono::milliseconds(100), [this]{
          status_tick();
        });
      }
      ~Mapper()
      {
        input_timer.reset();
        status_timer.reset();
        vio.stop();
        points.stop();
        slope.stop();
        get_node_waitables_interface()->remove_waitable(result_event, nullptr);
        get_node_waitables_interface()->remove_waitable(input_event, nullptr);
        if (snapshot_writer) snapshot_writer->stop();
      }
    };
  }
  int run_mapper(const Arguments& args)
  {
    auto mapper = std::make_shared<Mapper>(args);
    rclcpp::spin(mapper);
    return 0;
  }
}

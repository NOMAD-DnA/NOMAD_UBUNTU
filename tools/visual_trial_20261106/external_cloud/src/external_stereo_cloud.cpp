#include "cloud_geometry.hpp"

#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;
using external_cloud::require;

std::int64_t stamp(const Image& m) {
  return std::int64_t(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
}

struct Pair {
  Image::ConstSharedPtr left, right;
};

class Bridge final : public rclcpp::Node {
 public:
  Bridge() : Node("nomad_external_stereo_cloud") {
    const auto factory_path = declare_parameter<std::string>("factory_json", "");
    const auto vio_path = declare_parameter<std::string>("vio_calib_json", "");
    const auto device_id = declare_parameter<std::string>("device_id", "1944301091BA147E00");
    const auto left_topic = declare_parameter<std::string>("left_topic", "/oak/left/image_rect");
    const auto right_topic = declare_parameter<std::string>("right_topic", "/oak/right/image_rect");
    const auto output_topic = declare_parameter<std::string>("cloud_topic", "/oak/depth/points");
    const auto hz = declare_parameter<double>("output_hz", 1.0);
    first_row_ = declare_parameter<int>("first_source_row", 206);
    stride_ = declare_parameter<int>("pixel_stride", 8);
    min_depth_ = declare_parameter<double>("min_depth_m", 0.2);
    max_depth_ = declare_parameter<double>("max_depth_m", 10.0);
    max_age_s_ = declare_parameter<double>("max_capture_age_s", 1.5);
    const int threads = declare_parameter<int>("opencv_threads", 1);
    const int disparities = declare_parameter<int>("num_disparities", 192);
    const int block_size = declare_parameter<int>("block_size", 7);
    require(!factory_path.empty() && !vio_path.empty(), "factory_json and vio_calib_json are required");
    require(device_id == "1944301091BA147E00", "This saved factory calibration belongs to 1944301091BA147E00");
    require(std::isfinite(hz) && hz > 0 && hz <= 2 && first_row_ >= 0 && first_row_ < 480 &&
                stride_ > 0 && stride_ <= 64 && std::isfinite(min_depth_) &&
                std::isfinite(max_depth_) && min_depth_ > 0 && max_depth_ > min_depth_ &&
                std::isfinite(max_age_s_) && max_age_s_ > 0 && threads >= 1 && threads <= 4 &&
                disparities >= 16 && disparities <= 256 && disparities % 16 == 0 &&
                block_size >= 3 && block_size <= 15 && block_size % 2 == 1,
            "Invalid bounded cloud/SGBM parameters");
    period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
    const std::string factory_text = external_cloud::read_text(factory_path);
    const auto factory = external_cloud::parse_json(factory_text);
    const auto vio = external_cloud::parse_json(external_cloud::read_text(vio_path));
    geometry_ = std::make_unique<external_cloud::Geometry>(
        factory, external_cloud::read_vio_intrinsics(vio));
    cv::setNumThreads(threads);
    matcher_ = cv::StereoSGBM::create(0, disparities, block_size,
        8 * block_size * block_size, 32 * block_size * block_size,
        1, 31, 15, 50, 2, cv::StereoSGBM::MODE_SGBM);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::SensorDataQoS().keep_last(1));
    factory_publisher_ = create_publisher<std_msgs::msg::String>(
        "/oak/calibration/factory", rclcpp::QoS(1).reliable().transient_local());
    std_msgs::msg::String calibration;
    calibration.data = "{\"calibration_source\":\"factory\",\"device_id\":\"" + device_id +
        "\",\"factory_calibration\":" + factory_text + "}";
    factory_publisher_->publish(calibration);
    const auto qos = rclcpp::SensorDataQoS().keep_last(2);
    left_subscription_ = create_subscription<Image>(left_topic, qos,
        [this](Image::ConstSharedPtr m) { accept(std::move(m), true); });
    right_subscription_ = create_subscription<Image>(right_topic, qos,
        [this](Image::ConstSharedPtr m) { accept(std::move(m), false); });
    report_ = create_wall_timer(std::chrono::seconds(5), [this] {
      RCLCPP_INFO(get_logger(),
          "pairs=%llu selected=%llu clouds=%llu empty=%llu rejected=%llu "
          "stale=%llu overwritten=%llu last_points=%llu compute_ms=%.2f",
          static_cast<unsigned long long>(pairs_), static_cast<unsigned long long>(selected_),
          static_cast<unsigned long long>(published_.load()), static_cast<unsigned long long>(empty_.load()),
          static_cast<unsigned long long>(rejected_), static_cast<unsigned long long>(stale_.load()),
          static_cast<unsigned long long>(overwritten_), static_cast<unsigned long long>(last_points_.load()),
          compute_ms_.load());
      if (failed_.load()) throw std::runtime_error("External stereo worker failed");
    });
    worker_ = std::thread([this] { work(); });
    RCLCPP_INFO(get_logger(),
        "External OpenCV SGBM: %.2f Hz; ROI rows %d:480 stride %d; "
        "measured baseline %.9f m; saved factory serial %s; CAM_A conversion enabled. "
        "Depth is host stereo estimation, not device StereoDepth output.",
        hz, first_row_, stride_, geometry_->baseline_m, device_id.c_str());
  }

  ~Bridge() override {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    changed_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  void accept(Image::ConstSharedPtr m, bool left) {
    const char* expected = left ? "oak_left_camera_optical_frame" : "oak_right_camera_optical_frame";
    const auto ns = stamp(*m);
    if (ns <= 0 || m->header.frame_id != expected || m->encoding != "mono8" ||
        m->width != 640 || m->height != 480 || m->step < 640 ||
        m->data.size() != std::size_t(m->step) * 480 || m->is_bigendian) {
      ++rejected_;
      return;
    }
    if (ns <= last_selected_stamp_) return;
    auto& pair = pairs_by_stamp_[ns];
    (left ? pair.left : pair.right) = std::move(m);
    if (pair.left && pair.right) {
      ++pairs_;
      const auto now = Clock::now();
      if (!have_selected_ || now - last_selected_ >= period_) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_) ++overwritten_;
        latest_ = pair;
        last_selected_ = now;
        last_selected_stamp_ = ns;
        have_selected_ = true;
        ++selected_;
        changed_.notify_one();
      }
      pairs_by_stamp_.erase(pairs_by_stamp_.begin(), pairs_by_stamp_.upper_bound(ns));
    }
    while (pairs_by_stamp_.size() > 4) pairs_by_stamp_.erase(pairs_by_stamp_.begin());
  }

  void work() noexcept {
    try {
      while (rclcpp::ok()) {
        Pair pair;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          changed_.wait_for(lock, std::chrono::milliseconds(100),
                            [this] { return stopping_ || latest_.has_value(); });
          if (stopping_) return;
          if (!latest_) continue;
          pair = std::move(*latest_);
          latest_.reset();
        }
        const double age = (now().nanoseconds() - stamp(*pair.left)) / 1e9;
        if (age > max_age_s_ || age < -0.05) { ++stale_; continue; }
        const auto start = Clock::now();
        // Crop vertically before dense matching; preserve original pixel rows
        // for reconstruction. The horizontal disparity search stays full VGA.
        const int crop_row = std::max(0, first_row_ - matcher_->getBlockSize());
        cv::Mat left(480, 640, CV_8UC1,
                     const_cast<std::uint8_t*>(pair.left->data.data()), pair.left->step);
        cv::Mat right(480, 640, CV_8UC1,
                      const_cast<std::uint8_t*>(pair.right->data.data()), pair.right->step);
        cv::Mat disparity;
        const cv::Rect roi(0, crop_row, 640, 480 - crop_row);
        matcher_->compute(left(roi), right(roi), disparity);
        std::vector<cv::Vec3f> xyz;
        xyz.reserve(((480 - first_row_ + stride_ - 1) / stride_) * ((640 + stride_ - 1) / stride_));
        for (int v = first_row_; v < 480; v += stride_) {
          const auto* row = disparity.ptr<std::int16_t>(v - crop_row);
          for (int u = 0; u < 640; u += stride_) {
            if (row[u] <= 0) continue;
            cv::Vec3f point;
            if (geometry_->point_a(u, v, row[u] / 16.0, min_depth_, max_depth_, point)) xyz.push_back(point);
          }
        }
        compute_ms_.store(std::chrono::duration<double, std::milli>(Clock::now() - start).count());
        last_points_.store(xyz.size());
        if (xyz.empty()) { ++empty_; continue; }
        const double age_after = (now().nanoseconds() - stamp(*pair.left)) / 1e9;
        if (age_after > max_age_s_) { ++stale_; continue; }
        if (!rclcpp::ok()) return;
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = pair.left->header.stamp;
        cloud.header.frame_id = "oak_cam_a_optical_frame";
        cloud.height = 1;
        cloud.width = static_cast<std::uint32_t>(xyz.size());
        cloud.fields.resize(3);
        for (int i = 0; i < 3; ++i) {
          cloud.fields[i].name = std::array<const char*, 3>{"x", "y", "z"}[i];
          cloud.fields[i].offset = i * 4;
          cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
          cloud.fields[i].count = 1;
        }
        cloud.is_bigendian = false;
        cloud.point_step = 12;
        cloud.row_step = 12 * cloud.width;
        cloud.data.resize(cloud.row_step);
        for (std::size_t i = 0; i < xyz.size(); ++i)
          std::memcpy(cloud.data.data() + i * 12, xyz[i].val, 12);
        cloud.is_dense = true;
        publisher_->publish(std::move(cloud));
        ++published_;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "External stereo worker: %s", e.what());
      failed_.store(true);
    }
  }

  std::unique_ptr<external_cloud::Geometry> geometry_;
  cv::Ptr<cv::StereoSGBM> matcher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr factory_publisher_;
  rclcpp::Subscription<Image>::SharedPtr left_subscription_, right_subscription_;
  rclcpp::TimerBase::SharedPtr report_;
  std::map<std::int64_t, Pair> pairs_by_stamp_;
  std::optional<Pair> latest_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::thread worker_;
  bool stopping_{false}, have_selected_{false};
  Clock::duration period_{};
  Clock::time_point last_selected_{};
  std::int64_t last_selected_stamp_{0};
  int first_row_{206}, stride_{8};
  double min_depth_{0.2}, max_depth_{10.0}, max_age_s_{1.5};
  std::uint64_t pairs_{0}, selected_{0}, rejected_{0}, overwritten_{0};
  std::atomic<std::uint64_t> published_{0}, empty_{0}, stale_{0}, last_points_{0};
  std::atomic<double> compute_ms_{0};
  std::atomic<bool> failed_{false};
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto bridge = std::make_shared<Bridge>();
    rclcpp::spin(bridge);
    bridge.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "external_stereo_cloud: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
}

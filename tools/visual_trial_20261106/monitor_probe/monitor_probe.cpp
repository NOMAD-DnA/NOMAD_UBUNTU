// Read-only ROS observer. Owns no sensor, estimator, publisher or control path.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;
using Odometry = nav_msgs::msg::Odometry;
using Grid = nav_msgs::msg::OccupancyGrid;
using Status = nomad_interfaces::msg::ModuleStatus;

std::string quoted(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          static constexpr char hex[] = "0123456789abcdef";
          out << "\\u00" << hex[c >> 4] << hex[c & 15];
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  out << '"';
  return out.str();
}

double seconds(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

std::string base64(const std::vector<unsigned char>& data) {
  static constexpr char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((data.size() + 2) / 3) * 4);
  for (size_t i = 0; i < data.size(); i += 3) {
    const uint32_t bits = (static_cast<uint32_t>(data[i]) << 16) |
      (i + 1 < data.size() ? static_cast<uint32_t>(data[i + 1]) << 8 : 0) |
      (i + 2 < data.size() ? static_cast<uint32_t>(data[i + 2]) : 0);
    out.push_back(table[(bits >> 18) & 63]);
    out.push_back(table[(bits >> 12) & 63]);
    out.push_back(i + 1 < data.size() ? table[(bits >> 6) & 63] : '=');
    out.push_back(i + 2 < data.size() ? table[bits & 63] : '=');
  }
  return out;
}

cv::Size bounded(int width, int height, int max_width, int max_height) {
  if (width <= 0 || height <= 0 || width > 32768 || height > 32768) {
    throw std::runtime_error("Invalid image dimensions");
  }
  const double scale = std::min({1.0, static_cast<double>(max_width) / width,
                                static_cast<double>(max_height) / height});
  return {std::max(1, static_cast<int>(std::floor(width * scale))),
          std::max(1, static_cast<int>(std::floor(height * scale)))};
}

void image_shape(const Image& m, size_t bytes_per_pixel) {
  bounded(static_cast<int>(m.width), static_cast<int>(m.height), 640, 360);
  if (m.step < static_cast<size_t>(m.width) * bytes_per_pixel ||
      m.data.size() < static_cast<size_t>(m.step) * m.height) {
    throw std::runtime_error("Image stride or payload is truncated");
  }
}

cv::Mat resized(const cv::Mat& input, cv::Size shape, int interpolation) {
  if (input.size() == shape) return input;
  cv::Mat output;
  cv::resize(input, output, shape, 0, 0, interpolation);
  return output;
}

cv::Mat rgb_image(const Image& m) {
  int channels = 0;
  if (m.encoding == "rgb8" || m.encoding == "bgr8") channels = 3;
  else if (m.encoding == "rgba8" || m.encoding == "bgra8") channels = 4;
  else if (m.encoding == "mono8") channels = 1;
  else throw std::runtime_error("Unsupported RGB encoding: " + m.encoding);
  image_shape(m, channels);
  cv::Mat raw(static_cast<int>(m.height), static_cast<int>(m.width), CV_MAKETYPE(CV_8U, channels),
              const_cast<unsigned char*>(m.data.data()), m.step);
  cv::Mat small = resized(raw, bounded(raw.cols, raw.rows, 640, 360), cv::INTER_AREA);
  cv::Mat output;
  if (m.encoding == "rgb8") cv::cvtColor(small, output, cv::COLOR_RGB2GRAY);
  else if (m.encoding == "rgba8") cv::cvtColor(small, output, cv::COLOR_RGBA2GRAY);
  else if (m.encoding == "bgra8") cv::cvtColor(small, output, cv::COLOR_BGRA2GRAY);
  else if (m.encoding == "bgr8") cv::cvtColor(small, output, cv::COLOR_BGR2GRAY);
  else output = small;
  return output;
}

cv::Mat semantic_image(const Image& m) {
  if (m.encoding != "mono8") throw std::runtime_error("Semantic image must be mono8 field12");
  image_shape(m, 1);
  cv::Mat raw(static_cast<int>(m.height), static_cast<int>(m.width), CV_8UC1,
              const_cast<unsigned char*>(m.data.data()), m.step);
  cv::Mat small = resized(raw, bounded(raw.cols, raw.rows, 640, 360), cv::INTER_NEAREST);
  // BGR: ignore, person, vehicle, wood, soil, gravel, paved, rock,
  // leaves, vegetation, artificial, sky. Unknown IDs stay gray.
  static const std::array<cv::Vec3b, 12> palette{{
    {100, 100, 100}, {70, 70, 240}, {240, 110, 60}, {45, 85, 150},
    {70, 120, 180}, {190, 190, 190}, {125, 105, 90}, {210, 90, 170},
    {35, 165, 245}, {75, 175, 50}, {30, 150, 245}, {255, 175, 50}
  }};
  cv::Mat output(small.size(), CV_8UC3);
  for (int y = 0; y < small.rows; ++y) {
    const auto* ids = small.ptr<uint8_t>(y);
    auto* pixels = output.ptr<cv::Vec3b>(y);
    for (int x = 0; x < small.cols; ++x) {
      pixels[x] = ids[x] < palette.size() ? palette[ids[x]] : cv::Vec3b(100, 100, 100);
    }
  }
  return output;
}

cv::Vec3b heat_color(double value) {
  // Green (0), yellow (0.5), red (1). Inputs are finite and normalized.
  value = std::clamp(value, 0.0, 1.0);
  return {0, static_cast<uint8_t>(255 * std::min(1.0, 2 * (1 - value))),
          static_cast<uint8_t>(255 * std::min(1.0, 2 * value))};
}

cv::Mat slope_image(const Image& m) {
  if (m.encoding != "32FC1") throw std::runtime_error("Slope image must be 32FC1");
  image_shape(m, 4);
  const cv::Size shape = bounded(static_cast<int>(m.width), static_cast<int>(m.height), 640, 360);
  cv::Mat output(shape, CV_8UC3);
  const uint16_t endian_test = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t*>(&endian_test) == 0;
  const bool swap = static_cast<bool>(m.is_bigendian) != host_big_endian;
  for (int y = 0; y < output.rows; ++y) {
    // The mapper writes slope using OccupancyGrid's low-Y-first array order.
    // Match cost_image: JPEG row zero is the high-Y side of the grid.
    const size_t sy = m.height - 1 - std::min<size_t>(m.height - 1,
      static_cast<size_t>(y) * m.height / output.rows);
    auto* pixels = output.ptr<cv::Vec3b>(y);
    for (int x = 0; x < output.cols; ++x) {
      const size_t sx = std::min<size_t>(m.width - 1, static_cast<size_t>(x) * m.width / output.cols);
      const auto* bytes = m.data.data() + sy * m.step + sx * 4;
      std::array<uint8_t, 4> ordered{{bytes[0], bytes[1], bytes[2], bytes[3]}};
      if (swap) std::reverse(ordered.begin(), ordered.end());
      float value;
      std::memcpy(&value, ordered.data(), 4);
      pixels[x] = std::isfinite(value) ? heat_color(std::abs(value) / 30.0) : cv::Vec3b(100, 100, 100);
    }
  }
  return output;
}

cv::Mat cost_image(const Grid& m) {
  const cv::Size shape = bounded(static_cast<int>(m.info.width), static_cast<int>(m.info.height), 512, 512);
  if (m.data.size() < static_cast<size_t>(m.info.width) * m.info.height) {
    throw std::runtime_error("OccupancyGrid payload is truncated");
  }
  cv::Mat output(shape, CV_8UC3);
  for (int y = 0; y < output.rows; ++y) {
    // +Y points up in the displayed grid; ROS array row zero is the bottom.
    const size_t sy = m.info.height - 1 - std::min<size_t>(m.info.height - 1,
      static_cast<size_t>(y) * m.info.height / output.rows);
    auto* pixels = output.ptr<cv::Vec3b>(y);
    for (int x = 0; x < output.cols; ++x) {
      const size_t sx = std::min<size_t>(m.info.width - 1,
        static_cast<size_t>(x) * m.info.width / output.cols);
      const int value = m.data[sy * m.info.width + sx];
      pixels[x] = value >= 0 && value <= 100 ? heat_color(value / 100.0) : cv::Vec3b(100, 100, 100);
    }
  }
  return output;
}

const char* state_name(uint8_t state) {
  switch (state) {
    case Status::WAITING: return "WAITING";
    case Status::READY: return "READY";
    case Status::STOPPED: return "STOPPED";
    case Status::FAULT: return "FAULT";
    default: return "FAULT";
  }
}

bool same_capture(const Image& image, const Grid& grid) {
  return image.header.frame_id == grid.header.frame_id &&
    image.header.stamp.sec == grid.header.stamp.sec &&
    image.header.stamp.nanosec == grid.header.stamp.nanosec &&
    image.width == grid.info.width && image.height == grid.info.height;
}

std::string grid_metadata(const Grid& grid) {
  const auto& info = grid.info;
  const auto& p = info.origin.position;
  const auto& q = info.origin.orientation;
  const std::array<double, 8> values{{info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w}};
  const double q_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (grid.header.frame_id.empty() || info.width == 0 || info.height == 0 ||
      info.resolution <= 0 || q_norm <= 1e-12 ||
      !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) {
    throw std::runtime_error("OccupancyGrid geometry is invalid or has no frame_id");
  }
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"frame_id\":" << quoted(grid.header.frame_id)
      << ",\"resolution\":" << info.resolution
      << ",\"width\":" << info.width << ",\"height\":" << info.height
      << ",\"origin\":{\"x\":" << p.x << ",\"y\":" << p.y << ",\"z\":" << p.z
      << ",\"qx\":" << q.x << ",\"qy\":" << q.y << ",\"qz\":" << q.z << ",\"qw\":" << q.w
      << "},\"image_y\":\"high_to_low\"}";
  return out.str();
}

class Probe final : public rclcpp::Node {
 public:
  explicit Probe(const std::string& mode)
    : Node("nomad_external_monitor_" + mode), mode_(mode), rate_time_(Clock::now()) {
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    auto pose_qos = rclcpp::QoS(100).best_effort().durability_volatile();
    auto status_qos = rclcpp::QoS(10).reliable().durability_volatile();
    if (mode_ == "pi") {
      pose_subscription("vio", "/basalt/odom", pose_qos);
      pose_subscription("fusion", "/fusion/odometry", pose_qos);
      pose_subscription("public", "/nomad/localization/odometry", pose_qos);
      status_subscription("localization", "/nomad/localization/status", status_qos);
    } else {
      image_subscription("rgb", "/zed/zed_node/left/image_rect_color", sensor_qos);
      image_subscription("semantic", "/perception/semantic/class_ids", sensor_qos);
      // Mapper diagnostics and maps use reliable/transient-local QoS.
      auto map_qos = rclcpp::QoS(1).reliable().transient_local();
      counters_["slope"] = 0;
      subscriptions_.push_back(create_subscription<Image>("/nomad/semantic_validation/slope_deg", map_qos,
        [this](Image::ConstSharedPtr m) {
          ++counters_["slope"];
          pending_slopes_.push_back({m, Clock::now()});
          if (pending_slopes_.size() > 8) {
            emit_unreferenced_slope(pending_slopes_.front().image,
              "No exact OccupancyGrid pair before the bounded slope cache filled");
            pending_slopes_.pop_front();
          }
          flush_slopes(false);
        }));
      counters_["costmap"] = 0;
      subscriptions_.push_back(create_subscription<Grid>("/nomad/navigation_costmap", map_qos,
        [this](Grid::ConstSharedPtr m) {
          ++counters_["costmap"];
          recent_grids_.push_back(m);
          if (recent_grids_.size() > 8) recent_grids_.pop_front();
          if (image_due("costmap")) {
            try { emit_image("costmap", seconds(m->header.stamp), cost_image(*m), m.get()); }
            catch (const std::exception& e) { report_error("costmap", e.what()); }
          }
          flush_slopes(false);
        }));
      status_subscription("perception", "/nomad/perception/status", status_qos);
    }
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { flush_slopes(true); emit_periodic(); });
    std::cerr << "Read-only monitor mode=" << mode_ << "; image cap=1Hz; t=ROS capture seconds; no publishers\n";
  }

 private:
  std::string mode_;
  Clock::time_point rate_time_;
  std::map<std::string, uint64_t> counters_;
  std::map<std::string, Clock::time_point> last_image_;
  std::map<std::string, Clock::time_point> last_error_;
  Status::ConstSharedPtr latest_status_;
  Clock::time_point status_received_{};
  std::string status_source_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  struct PendingSlope {
    Image::ConstSharedPtr image;
    Clock::time_point received;
  };
  std::deque<Grid::ConstSharedPtr> recent_grids_;
  std::deque<PendingSlope> pending_slopes_;

  void emit(const std::string& line) {
    std::cout << line << '\n' << std::flush;
  }

  bool image_due(const std::string& source) {
    // Preview RGB is published every 0.3 s upstream; forward every frame without a second jitter-sensitive gate.
    if (source == "rgb") return true;
    const auto now = Clock::now();
    auto it = last_image_.find(source);
    if (it != last_image_.end() && now - it->second < std::chrono::seconds(1)) return false;
    last_image_[source] = now;
    return true;
  }

  void report_error(const std::string& source, const std::string& detail) {
    const auto now = Clock::now();
    auto it = last_error_.find(source);
    if (it != last_error_.end() && now - it->second < std::chrono::seconds(5)) return;
    last_error_[source] = now;
    std::cerr << source << ": " << detail << '\n';
  }

  void pose_subscription(const std::string& source, const std::string& topic, const rclcpp::QoS& qos) {
    counters_[source] = 0;
    subscriptions_.push_back(create_subscription<Odometry>(topic, qos,
      [this, source](Odometry::ConstSharedPtr m) {
        ++counters_[source];
        const auto& p = m->pose.pose.position;
        const auto& q = m->pose.pose.orientation;
        const std::array<double, 7> values{{p.x, p.y, p.z, q.x, q.y, q.z, q.w}};
        if (!std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) {
          report_error(source, "Nonfinite pose omitted from JSONL");
          return;
        }
        std::ostringstream out;
        out << std::setprecision(17) << "{\"kind\":\"pose\",\"source\":" << quoted(source)
            << ",\"t\":" << seconds(m->header.stamp) << ",\"x\":" << p.x << ",\"y\":" << p.y
            << ",\"z\":" << p.z << ",\"qx\":" << q.x << ",\"qy\":" << q.y
            << ",\"qz\":" << q.z << ",\"qw\":" << q.w
            << ",\"frame_id\":" << quoted(m->header.frame_id)
            << ",\"child_frame_id\":" << quoted(m->child_frame_id) << '}';
        emit(out.str());
      }));
  }

  void image_subscription(const std::string& source, const std::string& topic, const rclcpp::QoS& qos) {
    counters_[source] = 0;
    subscriptions_.push_back(create_subscription<Image>(topic, qos,
      [this, source](Image::ConstSharedPtr m) {
        ++counters_[source];
        if (!image_due(source)) return;
        try {
          cv::Mat output = source == "rgb" ? rgb_image(*m) :
                           source == "semantic" ? semantic_image(*m) : slope_image(*m);
          emit_image(source, seconds(m->header.stamp), output);
        } catch (const std::exception& e) { report_error(source, e.what()); }
      }));
  }

  void status_subscription(const std::string& source, const std::string& topic, const rclcpp::QoS& qos) {
    counters_[source] = 0;
    status_source_ = source;
    subscriptions_.push_back(create_subscription<Status>(topic, qos,
      [this, source](Status::ConstSharedPtr m) {
        ++counters_[source];
        latest_status_ = m;
        status_received_ = Clock::now();
      }));
  }

  void emit_unreferenced_slope(const Image::ConstSharedPtr& image, const std::string& reason) {
    if (!image_due("slope")) return;
    try { emit_image("slope", seconds(image->header.stamp), slope_image(*image), nullptr, reason); }
    catch (const std::exception& e) { report_error("slope", e.what()); }
  }

  void flush_slopes(bool expire_unmatched) {
    const auto now = Clock::now();
    for (auto it = pending_slopes_.begin(); it != pending_slopes_.end();) {
      const auto match = std::find_if(recent_grids_.rbegin(), recent_grids_.rend(),
        [&it](const Grid::ConstSharedPtr& grid) { return same_capture(*it->image, *grid); });
      if (match != recent_grids_.rend()) {
        if (image_due("slope")) {
          try { emit_image("slope", seconds(it->image->header.stamp), slope_image(*it->image), match->get()); }
          catch (const std::exception& e) { report_error("slope", e.what()); }
        }
        it = pending_slopes_.erase(it);
      } else if (expire_unmatched && now - it->received >= std::chrono::seconds(1)) {
        emit_unreferenced_slope(it->image,
          "No OccupancyGrid with identical frame_id, capture stamp, width and height");
        it = pending_slopes_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void emit_image(const std::string& source, double stamp, const cv::Mat& pixels,
                  const Grid* grid = nullptr, const std::string& reason = "") {
    const bool is_map = source == "costmap" || source == "slope";
    std::vector<unsigned char> encoded;
    const auto parameters = is_map ? std::vector<int>{cv::IMWRITE_PNG_COMPRESSION, 3} :
                                    std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 65};
    if (!cv::imencode(is_map ? ".png" : ".jpg", pixels, encoded, parameters)) {
      throw std::runtime_error("Image encoding failed");
    }
    std::ostringstream out;
    out << std::setprecision(17) << "{\"kind\":\"image\",\"source\":" << quoted(source)
        << ",\"t\":" << stamp << ",\"encoding\":" << quoted(is_map ? "png" : "jpeg")
        << ",\"data\":\"" << base64(encoded)
        << "\",\"width\":" << pixels.cols << ",\"height\":" << pixels.rows;
    if (is_map) {
      std::string metadata;
      std::string failure = reason;
      if (grid) {
        try { metadata = grid_metadata(*grid); }
        catch (const std::exception& e) { failure = e.what(); }
      }
      out << ",\"georeferenced\":" << (metadata.empty() ? "false" : "true");
      if (!metadata.empty()) out << ",\"grid\":" << metadata;
      else out << ",\"georeference_reason\":" << quoted(failure.empty() ? "Map geometry unavailable" : failure);
    }
    out << '}';
    emit(out.str());
  }

  void emit_periodic() {
    const auto steady = Clock::now();
    const double elapsed = std::chrono::duration<double>(steady - rate_time_).count();
    std::ostringstream out;
    out << std::setprecision(17) << "{\"kind\":\"rates\",\"source\":" << quoted(mode_) << ",\"rates\":{";
    bool first = true;
    for (auto& counter : counters_) {
      if (!first) out << ',';
      first = false;
      out << quoted(counter.first) << ':' << (elapsed > 0 ? counter.second / elapsed : 0);
      counter.second = 0;
    }
    out << "}}";
    emit(out.str());
    rate_time_ = steady;

    double stamp = now().seconds();
    const char* state = "WAITING";
    std::string detail = "No module status received";
    if (latest_status_) {
      stamp = seconds(latest_status_->header.stamp);
      state = state_name(latest_status_->state);
      detail = latest_status_->detail;
      const double valid = latest_status_->valid_for.sec + latest_status_->valid_for.nanosec * 1e-9;
      const double capture_age = now().seconds() - stamp;
      const double receipt_age = std::chrono::duration<double>(steady - status_received_).count();
      if (valid <= 0 || capture_age < -0.05 || capture_age > valid || receipt_age > 1.0) {
        state = "WAITING";
        detail = "Expired/future module status; last: " + detail;
      }
    }
    std::ostringstream status;
    status << std::setprecision(17) << "{\"kind\":\"status\",\"source\":" << quoted(status_source_)
           << ",\"state\":" << quoted(state) << ",\"detail\":" << quoted(detail) << ",\"t\":" << stamp << '}';
    emit(status.str());
  }
};
}  // namespace

int main(int argc, char** argv) {
  try {
    // Logging goes to stderr even when the ROS environment normally selects stdout.
    setenv("RCUTILS_LOGGING_USE_STDOUT", "0", 1);
    setenv("RCUTILS_COLORIZED_OUTPUT", "0", 1);
    if (!getenv("ROS_DOMAIN_ID")) setenv("ROS_DOMAIN_ID", "59", 1);
    std::string mode;
    std::vector<char*> ros_argv{argv[0]};
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--mode" && i + 1 < argc) mode = argv[++i];
      else ros_argv.push_back(argv[i]);
    }
    if (mode != "pi" && mode != "jetson") {
      std::cerr << "Usage: nomad_monitor_probe --mode pi|jetson [--ros-args ...]\n";
      return 2;
    }
    const int ros_argc = static_cast<int>(ros_argv.size());
    ros_argv.push_back(nullptr);
    rclcpp::init(ros_argc, ros_argv.data(), rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigInt);
    auto probe = std::make_shared<Probe>(mode);
    std::atomic<bool> stopped{false};
    std::thread stdin_watch([&stopped] {
      while (!stopped.load() && rclcpp::ok()) {
        pollfd descriptor{STDIN_FILENO, POLLIN | POLLHUP, 0};
        const int result = poll(&descriptor, 1, 100);
        if (result < 0 && errno == EINTR) continue;
        if (result < 0 || (result > 0 && (descriptor.revents & POLLNVAL))) {
          rclcpp::shutdown();
          return;
        }
        if (result > 0 && (descriptor.revents & (POLLIN | POLLHUP | POLLERR))) {
          char ignored[256];
          const ssize_t count = read(STDIN_FILENO, ignored, sizeof(ignored));
          if (count == 0 || (count < 0 && errno != EINTR && errno != EAGAIN)) {
            rclcpp::shutdown();
            return;
          }
        }
      }
    });
    try { rclcpp::spin(probe); }
    catch (...) {
      stopped.store(true);
      if (rclcpp::ok()) rclcpp::shutdown();
      stdin_watch.join();
      throw;
    }
    stopped.store(true);
    if (rclcpp::ok()) rclcpp::shutdown();
    stdin_watch.join();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Monitor: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}

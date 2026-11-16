// Read-only ROS observer. Owns no sensor, estimator, publisher or control path.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "jpeg_metadata.hpp"
#include "slope_preview.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#if defined(NOMAD_MONITOR_WITH_GNSS_QUALITY) && __has_include(<nomad_interfaces/msg/gnss_quality.hpp>)
#include <nomad_interfaces/msg/gnss_quality.hpp>
#define NOMAD_MONITOR_HAS_GNSS_QUALITY 1
#endif
#ifdef NOMAD_MONITOR_WITH_BASALT_FLOW
#include <basalt_vio_msgs/msg/optical_flow_result.hpp>
#endif
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
#include <set>
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

std::string finite_json(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}

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

slope_preview::Scale preview_scale() {
  static slope_preview::Scale scale;
  static auto updated = std::chrono::steady_clock::time_point{};
  const auto now = std::chrono::steady_clock::now();
  if (now - updated >= std::chrono::seconds(1)) {
    scale = slope_preview::running_scale();
    updated = now;
  }
  return scale;
}

cv::Mat slope_image(const Image& m, const slope_preview::Scale& scale = preview_scale()) {
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
      pixels[x] = std::isfinite(value) ? heat_color(std::abs(value) / scale.max_deg) : cv::Vec3b(100, 100, 100);
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
      localization_telemetry(sensor_qos);
    } else {
      magnetometer_telemetry(sensor_qos);
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
    std::cerr << "Read-only monitor (frontend JPEG passthrough) mode=" << mode_ << "; image cap=1Hz; t=ROS capture seconds; no publishers\n";
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
  rclcpp::TimerBase::SharedPtr mag_timer_;
  sensor_msgs::msg::MagneticField::ConstSharedPtr magnetic_field_;
  sensor_msgs::msg::Imu::ConstSharedPtr magnetic_imu_;
  Clock::time_point mag_received_{}, mag_imu_received_{};
  sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_fix_;
  Clock::time_point fix_received_{}, quality_received_{};
  uint64_t fix_count_ = 0, quality_count_ = 0;
  double fix_hz_ = 0, quality_hz_ = 0;
#ifdef NOMAD_MONITOR_HAS_GNSS_QUALITY
  nomad_interfaces::msg::GnssQuality::ConstSharedPtr gnss_quality_;
#endif
#ifdef NOMAD_MONITOR_WITH_BASALT_FLOW
  using Flow = basalt_vio_msgs::msg::OpticalFlowResult;
  std::set<uint32_t> previous_flow_ids_;
  int64_t previous_callback_stamp_ = 0;
#endif
  struct PendingSlope {
    Image::ConstSharedPtr image;
    Clock::time_point received;
  };
  std::deque<Grid::ConstSharedPtr> recent_grids_;
  std::deque<PendingSlope> pending_slopes_;

  void emit(const std::string& line) {
    std::cout << line << '\n' << std::flush;
  }

  void magnetometer_telemetry(const rclcpp::QoS& qos) {
    // Local ZED topics only. Cache scalar sensor readings; never reopen the
    // SDK/device, subscribe to images, or publish into the estimator path.
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::MagneticField>(
      "/zed/zed_node/imu/mag", qos,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr m) {
        magnetic_field_ = m; mag_received_ = Clock::now();
      }));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>(
      "/zed/zed_node/imu/data_raw", qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        magnetic_imu_ = m; mag_imu_received_ = Clock::now();
      }));
    mag_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { emit_magnetometer(); });
    emit_magnetometer();
  }

  void emit_magnetometer() {
    const auto steady = Clock::now();
    const double ros_now = now().seconds();
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"kind\":\"telemetry\",\"source\":\"magnetometer\",\"t\":" << finite_json(ros_now)
        << ",\"received\":" << (magnetic_field_ ? "true" : "false")
        << ",\"calibration_status\":\"unknown\",\"magnetic_field_source\":\"zed_sdk_calibrated_field\""
        << ",\"forward_axis\":[1,0,0],\"mag\":";
    if (magnetic_field_) {
      const auto& m = *magnetic_field_;
      const bool unknown_covariance = std::all_of(m.magnetic_field_covariance.begin(),
        m.magnetic_field_covariance.end(), [](double v) { return v == 0.; });
      out << "{\"stamp\":" << finite_json(seconds(m.header.stamp))
          << ",\"age_s\":" << finite_json(ros_now - seconds(m.header.stamp))
          << ",\"received_age_s\":" << finite_json(std::chrono::duration<double>(steady - mag_received_).count())
          << ",\"frame_id\":" << quoted(m.header.frame_id)
          << ",\"x_ut\":" << finite_json(m.magnetic_field.x * 1e6)
          << ",\"y_ut\":" << finite_json(m.magnetic_field.y * 1e6)
          << ",\"z_ut\":" << finite_json(m.magnetic_field.z * 1e6)
          << ",\"covariance_unknown\":" << (unknown_covariance ? "true" : "false") << '}';
    } else out << "null";
    out << ",\"imu\":";
    if (magnetic_imu_) {
      const auto& m = *magnetic_imu_;
      out << "{\"stamp\":" << finite_json(seconds(m.header.stamp))
          << ",\"age_s\":" << finite_json(ros_now - seconds(m.header.stamp))
          << ",\"received_age_s\":" << finite_json(std::chrono::duration<double>(steady - mag_imu_received_).count())
          << ",\"frame_id\":" << quoted(m.header.frame_id)
          << ",\"ax\":" << finite_json(m.linear_acceleration.x)
          << ",\"ay\":" << finite_json(m.linear_acceleration.y)
          << ",\"az\":" << finite_json(m.linear_acceleration.z) << '}';
    } else out << "null";
    out << '}';
    emit(out.str());
  }

  void localization_telemetry(const rclcpp::QoS& qos) {
    counters_["gnss_fix"] = counters_["gnss_quality"] = counters_["fusion_diagnostics"] = 0;
    counters_["vio_features"] = counters_["vio_tracking"] = 0;
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        const auto received = Clock::now();
        if (fix_count_) {
          const double dt = std::chrono::duration<double>(received - fix_received_).count();
          if (dt > 0) fix_hz_ = 1.0 / dt;
        }
        gnss_fix_ = m; fix_received_ = received; ++fix_count_; ++counters_["gnss_fix"];
        emit_gnss("fix");
      }));
#ifdef NOMAD_MONITOR_HAS_GNSS_QUALITY
    subscriptions_.push_back(create_subscription<nomad_interfaces::msg::GnssQuality>("/gps/quality", qos,
      [this](nomad_interfaces::msg::GnssQuality::ConstSharedPtr m) {
        const auto received = Clock::now();
        if (quality_count_) {
          const double dt = std::chrono::duration<double>(received - quality_received_).count();
          if (dt > 0) quality_hz_ = 1.0 / dt;
        }
        gnss_quality_ = m; quality_received_ = received; ++quality_count_; ++counters_["gnss_quality"];
        emit_gnss("quality");
      }));
#endif
    subscriptions_.push_back(create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/fusion/status", qos,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
        ++counters_["fusion_diagnostics"];
        std::map<std::string, std::string> values;
        std::string mode;
        int level = -1;
        for (const auto& item : m->status) {
          if (item.name != "fusion") continue;
          mode = item.message.substr(0, 256); level = item.level;
          for (const auto& value : item.values) {
            if (values.size() >= 64) break;
            values[value.key.substr(0, 64)] = value.value.substr(0, 80);
          }
          break;
        }
        const auto number = [&values](const std::string& key) {
          auto it = values.find(key);
          if (it == values.end() || it->second.empty()) return std::string("null");
          char* end = nullptr;
          const double parsed = std::strtod(it->second.c_str(), &end);
          return end == it->second.c_str() + it->second.size() ? finite_json(parsed) : std::string("null");
        };
        std::ostringstream out;
        out << std::setprecision(17) << "{\"kind\":\"telemetry\",\"source\":\"fusion\",\"received\":true"
            << ",\"t\":" << seconds(m->header.stamp) << ",\"measurement_stamp\":" << seconds(m->header.stamp)
            << ",\"measurement_stamp_ns\":" << stamp_ns(m->header.stamp)
            << ",\"received_ros_stamp\":" << now().seconds()
            << ",\"measurement_age_s\":" << finite_json(now().seconds() - seconds(m->header.stamp))
            << ",\"received_age_s\":0,\"mode\":" << quoted(mode) << ",\"level\":" << level
            << ",\"gnss_accepted\":" << number("gps_accepted") << ",\"gnss_rejected\":" << number("rejected_gps")
            << ",\"optimization_state\":\"not_reported\",\"solve_ms\":" << number("solve_ms")
            << ",\"solves\":" << number("solves") << ",\"epoch\":" << number("epoch")
            << ",\"values\":{";
        bool first = true;
        for (const auto& entry : values) {
          if (!first) out << ',';
          first = false; out << quoted(entry.first) << ':' << quoted(entry.second);
        }
        out << "}}";
        emit(out.str());
      }));
    emit_gnss("initial");
    emit("{\"kind\":\"telemetry\",\"source\":\"fusion\",\"received\":false,\"t\":" +
         finite_json(now().seconds()) + ",\"mode\":null,\"measurement_stamp\":null,\"received_age_s\":null,\"values\":{}}");
    // This is a local, already encoded frontend preview. Never subscribe to
    // the remote OAK raw image just to draw a GUI overlay.
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(
      "/basalt/flow_image/compressed", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) { receive_tracking_image(m); }));
#ifdef NOMAD_MONITOR_WITH_BASALT_FLOW
    subscriptions_.push_back(create_subscription<Flow>("/basalt/optical_flow", qos,
      [this](Flow::ConstSharedPtr m) { receive_flow(m); }));
#endif
    emit("{\"kind\":\"telemetry\",\"source\":\"vio_features\",\"received\":false,\"t\":" +
         finite_json(now().seconds()) + ",\"feature_count\":null,\"measurement_stamp\":null,\"received_age_s\":null,\"supported\":" +
#ifdef NOMAD_MONITOR_WITH_BASALT_FLOW
         "true}");
#else
         "false}");
#endif
  }

  void emit_gnss(const std::string& update) {
    const double ros_now = now().seconds();
    const auto steady = Clock::now();
    double latest_stamp = ros_now;
    if (gnss_fix_) latest_stamp = seconds(gnss_fix_->header.stamp);
#ifdef NOMAD_MONITOR_HAS_GNSS_QUALITY
    if (gnss_quality_) latest_stamp = std::max(gnss_fix_ ? latest_stamp : 0.0, seconds(gnss_quality_->header.stamp));
#endif
    std::ostringstream out;
    out << std::setprecision(17) << "{\"kind\":\"telemetry\",\"source\":\"gnss\",\"t\":" << latest_stamp
        << ",\"updated\":" << quoted(update) << ",\"received\":" << (fix_count_ || quality_count_ ? "true" : "false")
        << ",\"received_ros_stamp\":" << ros_now
        << ",\"fix_received\":" << (gnss_fix_ ? "true" : "false")
        << ",\"fix_count\":" << fix_count_ << ",\"quality_count\":" << quality_count_
        << ",\"fix_hz\":" << fix_hz_ << ",\"quality_hz\":" << quality_hz_;
    if (gnss_fix_) {
      const auto& m = *gnss_fix_;
      const bool valid = m.status.status >= 0 && std::isfinite(m.latitude) && std::isfinite(m.longitude) &&
                         std::isfinite(m.altitude) && std::abs(m.latitude) <= 90 && std::abs(m.longitude) <= 180;
      const bool covariance = m.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
                              std::isfinite(m.position_covariance[0]) && m.position_covariance[0] >= 0 &&
                              std::isfinite(m.position_covariance[4]) && m.position_covariance[4] >= 0;
      out << ",\"fix_stamp\":" << seconds(m.header.stamp) << ",\"fix_stamp_ns\":" << stamp_ns(m.header.stamp)
          << ",\"fix_age_s\":" << finite_json(ros_now - seconds(m.header.stamp))
          << ",\"fix_received_age_s\":" << std::chrono::duration<double>(steady - fix_received_).count()
          << ",\"fix_status\":" << int(m.status.status) << ",\"fix_valid\":" << (valid ? "true" : "false")
          << ",\"latitude\":" << finite_json(m.latitude) << ",\"longitude\":" << finite_json(m.longitude)
          << ",\"altitude\":" << finite_json(m.altitude)
          << ",\"h_acc_m\":" << (covariance ? finite_json(std::sqrt(std::max(m.position_covariance[0], m.position_covariance[4]))) : "null")
          << ",\"h_acc_source\":\"sqrt_max_horizontal_covariance_diagonal\"";
    } else out << ",\"fix_stamp\":null,\"fix_stamp_ns\":null,\"fix_age_s\":null,\"fix_received_age_s\":null,\"fix_status\":null,\"fix_valid\":false,\"latitude\":null,\"longitude\":null,\"altitude\":null,\"h_acc_m\":null";
#ifdef NOMAD_MONITOR_HAS_GNSS_QUALITY
    out << ",\"quality_supported\":true,\"quality_received\":" << (gnss_quality_ ? "true" : "false");
    if (gnss_quality_) {
      const auto& m = *gnss_quality_;
      using Quality = nomad_interfaces::msg::GnssQuality;
      out << ",\"quality_stamp\":" << seconds(m.header.stamp) << ",\"quality_stamp_ns\":" << stamp_ns(m.header.stamp)
          << ",\"quality_age_s\":" << finite_json(ros_now - seconds(m.header.stamp))
          << ",\"quality_received_age_s\":" << std::chrono::duration<double>(steady - quality_received_).count()
          << ",\"quality_valid_fields\":" << m.valid_fields
          << ",\"fix_type\":" << ((m.valid_fields & Quality::SOLUTION_VALID) ? std::to_string(m.solution_type) : "null")
          << ",\"satellites\":" << ((m.valid_fields & Quality::SATELLITES_VALID) ? std::to_string(m.satellites_used) : "null")
          << ",\"pdop\":" << ((m.valid_fields & Quality::PDOP_VALID) ? finite_json(m.pdop) : "null")
          << ",\"correction_age_s\":" << ((m.valid_fields & Quality::CORRECTION_AGE_VALID) ? finite_json(m.correction_age_s) : "null");
    } else out << ",\"quality_stamp\":null,\"quality_stamp_ns\":null,\"quality_age_s\":null,\"quality_received_age_s\":null,\"quality_valid_fields\":0,\"fix_type\":null,\"satellites\":null,\"pdop\":null,\"correction_age_s\":null";
#else
    out << ",\"quality_supported\":false,\"quality_received\":false,\"quality_stamp\":null,\"quality_stamp_ns\":null,\"quality_age_s\":null,\"quality_received_age_s\":null,\"quality_valid_fields\":0,\"fix_type\":null,\"satellites\":null,\"pdop\":null,\"correction_age_s\":null";
#endif
    out << '}'; emit(out.str());
  }

#ifdef NOMAD_MONITOR_WITH_BASALT_FLOW
  void receive_flow(const Flow::ConstSharedPtr& m) {
    ++counters_["vio_features"];
    const int64_t ns = stamp_ns(m->header.stamp);
    if (ns <= previous_callback_stamp_ || ns - previous_callback_stamp_ > 1000000000LL) previous_flow_ids_.clear();
    const size_t count = m->observations.empty() ? 0 : m->observations[0].keypoints.size();
    size_t tracked = 0, finite_count = 0;
    std::set<uint32_t> current_ids;
    if (!m->observations.empty()) {
      for (const auto& kp : m->observations[0].keypoints) {
        if (!std::isfinite(kp.affine_transform[4]) || !std::isfinite(kp.affine_transform[5])) continue;
        ++finite_count;
        tracked += previous_flow_ids_.count(kp.id) != 0;
        current_ids.insert(kp.id);
      }
    }
    std::ostringstream out;
    out << std::setprecision(17) << "{\"kind\":\"telemetry\",\"source\":\"vio_features\",\"received\":true,\"supported\":true"
        << ",\"t\":" << seconds(m->header.stamp) << ",\"measurement_stamp\":" << seconds(m->header.stamp)
        << ",\"measurement_stamp_ns\":" << ns << ",\"received_ros_stamp\":" << now().seconds()
        << ",\"measurement_age_s\":" << finite_json(now().seconds() - seconds(m->header.stamp))
        << ",\"received_age_s\":0,\"feature_count\":" << count << ",\"finite_feature_count\":" << finite_count
        << ",\"tracked_count\":" << tracked << ",\"new_count\":" << (finite_count - tracked)
        << ",\"camera_index\":0,\"camera_feature_counts\":[";
    for (size_t i = 0; i < m->observations.size() && i < 8; ++i) {
      if (i) out << ',';
      out << m->observations[i].keypoints.size();
    }
    out << "],\"coordinate_source\":\"basalt_optical_flow\",\"observation_stage\":\"frontend_not_backend_inliers\"}";
    emit(out.str());
    previous_flow_ids_ = std::move(current_ids); previous_callback_stamp_ = ns;
  }

#endif

  void receive_tracking_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& m) {
    try {
      if (stamp_ns(m->header.stamp) <= 0)
        throw std::runtime_error("Frontend tracking JPEG has no positive capture stamp");
      if (m->format.find("jpeg") == std::string::npos || m->data.size() > 2 * 1024 * 1024)
        throw std::runtime_error("Invalid or oversized frontend tracking JPEG");
      if (!image_due("vio_tracking")) return;
      const auto dims = nomad_monitor::jpeg_dimensions(m->data);
      // Forward exactly the existing JPEG; do not decode, draw, or re-encode.
      std::ostringstream out;
      out << std::setprecision(17) << "{\"kind\":\"image\",\"source\":\"vio_tracking\""
          << ",\"t\":" << seconds(m->header.stamp) << ",\"encoding\":\"jpeg\",\"data\":\""
          << base64(m->data) << "\",\"width\":" << dims.first << ",\"height\":" << dims.second
          << ",\"image_source\":\"basalt_frontend_cached_input\",\"jpeg_passthrough\":true}";
      emit(out.str());
      ++counters_["vio_tracking"];
    } catch (const std::exception& error) { report_error("vio_tracking", error.what()); }
  }

  bool image_due(const std::string& source) {
    // Bound every GUI image stream before conversion/base64/SSH transmission.
    // Sensor/VIO/pose publication rates and the capture stamps stay unchanged.
    const auto now = Clock::now();
    auto it = last_image_.find(source);
    const auto period = std::chrono::seconds(1);
    if (it != last_image_.end() && now - it->second < period) return false;
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
      const auto scale = preview_scale();
      out << ",\"slope_display\":{\"max_deg\":" << scale.max_deg
          << ",\"applied\":" << (scale.applied ? "true" : "false");
      if (scale.applied) out << ",\"mapper\":\"terrain\",\"k\":" << scale.k;
      out << '}';
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

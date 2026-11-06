#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

// This external adapter names an explicitly verified identity of optical axes;
// it does not perform image rectification, geometric calibration, ID conversion,
// projection changes, or a physical sensor-frame rotation. It publishes no TF.
//
// Source evidence: Stereolabs zed-ros2-wrapper humble-v4.2.5,
// zed_camera_component.cpp retrieveVideoDepth/publishVideoDepth/fillCamInfo:
// rectified left and RGB use the same mMatLeft from sl::VIEW::LEFT; their camera
// info objects are shared. LEFT_UNRECTIFIED is a separate input path.
// https://github.com/stereolabs/zed-ros2-wrapper/blob/humble-v4.2.5/zed_components/src/zed_camera/src/zed_camera_component.cpp
// Optical axes follow REP-103: x right, y down, z forward.
// https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst
// Runtime confirmation supplied by the operator is still required: the source
// is /zed/zed_node/left/image_rect_color from ZED2 serial 23472639; both observed
// source message frame IDs are zed_left_camera_optical_frame.

namespace {
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
constexpr const char* kAliasFrame = "zed2_23472639_left_rectified_optical_frame";
constexpr std::int64_t kCameraSerial = 23472639;

class FrameAlias final : public rclcpp::Node {
 public:
  FrameAlias() : Node("nomad_external_frame_alias") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto serial = declare_parameter<std::int64_t>("confirmed_camera_serial", 0, descriptor);
    const bool confirmed = declare_parameter<bool>("same_left_rectified_optical_axis_confirmed", false, descriptor);
    expected_image_frame_ = declare_parameter<std::string>("expected_image_frame", "zed_left_camera_optical_frame", descriptor);
    expected_info_frame_ = declare_parameter<std::string>("expected_camera_info_frame", "zed_left_camera_optical_frame", descriptor);
    const auto input_image = declare_parameter<std::string>("input_label_topic", "/perception/semantic/ade20k/class_ids", descriptor);
    const auto output_image = declare_parameter<std::string>("output_label_topic", "/nomad_preview/ade20k/class_ids", descriptor);
    const auto input_info = declare_parameter<std::string>("input_camera_info_topic", "/zed/zed_node/left/camera_info", descriptor);
    const auto output_info = declare_parameter<std::string>("output_camera_info_topic", "/nomad_preview/zed/camera_info", descriptor);
    if (!confirmed || serial != kCameraSerial) {
      throw std::runtime_error("Explicit verified left-rectified optical identity and confirmed_camera_serial=23472639 are required");
    }
    if (expected_image_frame_.empty() || expected_info_frame_.empty() || input_image.empty() ||
        output_image.empty() || input_info.empty() || output_info.empty() ||
        input_image == output_image || input_info == output_info) {
      throw std::runtime_error("Source frames/topics must be explicit and input/output topics must differ");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    image_publisher_ = create_publisher<Image>(output_image, qos);
    info_publisher_ = create_publisher<CameraInfo>(output_info, qos);
    image_subscription_ = create_subscription<Image>(input_image, qos,
        [this](Image::UniquePtr message) { on_image(std::move(message)); });
    info_subscription_ = create_subscription<CameraInfo>(input_info, qos,
        [this](CameraInfo::UniquePtr message) { on_info(std::move(message)); });
    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] {
      RCLCPP_INFO(get_logger(), "aliased labels=%llu camera_info=%llu; rejected labels=%llu camera_info=%llu; capture stamps/data/calibration untouched",
          static_cast<unsigned long long>(images_), static_cast<unsigned long long>(infos_),
          static_cast<unsigned long long>(rejected_images_), static_cast<unsigned long long>(rejected_infos_));
    });
    RCLCPP_INFO(get_logger(), "Verified serial %lld optical alias: labels %s; camera_info %s -> %s. No restamping, resize, ID conversion, or TF publication.",
        static_cast<long long>(serial), expected_image_frame_.c_str(), expected_info_frame_.c_str(), kAliasFrame);
  }

 private:
  static bool valid_stamp(const builtin_interfaces::msg::Time& stamp, bool allow_zero) {
    return stamp.sec >= 0 && stamp.nanosec < 1'000'000'000U &&
           (allow_zero || stamp.sec != 0 || stamp.nanosec != 0);
  }
  template<typename Container> static bool finite(const Container& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  }
  void reject(const char* stream, const char* reason) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejected %s: %s; refusing to relabel unknown optical geometry", stream, reason);
  }
  void on_image(Image::UniquePtr message) {
    const char* error = nullptr;
    if (message->header.frame_id != expected_image_frame_) error = "source frame differs from verified frame";
    else if (message->width != 640 || message->height != 360 || message->encoding != "mono8")
      error = "expected unchanged 640x360 mono8 ADE label image";
    else if (!valid_stamp(message->header.stamp, false)) error = "invalid original capture stamp";
    else if (message->is_bigendian > 1 || message->step < message->width ||
             static_cast<std::uint64_t>(message->step) * message->height != message->data.size())
      error = "invalid original stride or data length";
    if (error) { ++rejected_images_; reject("labels", error); return; }
    // Unique ownership allows the native byte vector to stay in this message.
    // Changing this one string is the entire semantic operation of the adapter.
    message->header.frame_id = kAliasFrame;
    image_publisher_->publish(std::move(message));
    ++images_;
  }
  void on_info(CameraInfo::UniquePtr message) {
    const char* error = nullptr;
    if (message->header.frame_id != expected_info_frame_) error = "source frame differs from verified frame";
    else if (message->width != 1280 || message->height != 720) error = "expected original 1280x720 CameraInfo";
    else if (!valid_stamp(message->header.stamp, true)) error = "invalid original metadata stamp";
    else if (!finite(message->k) || !finite(message->p) || !finite(message->r) || !finite(message->d) ||
             message->k[0] <= 0.0 || message->k[4] <= 0.0 || message->p[0] <= 0.0 || message->p[5] <= 0.0)
      error = "invalid source intrinsic/projection metadata";
    else if (message->binning_x > 1 || message->binning_y > 1 || message->roi.x_offset != 0 ||
             message->roi.y_offset != 0 || (message->roi.width != 0 && message->roi.width != 1280) ||
             (message->roi.height != 0 && message->roi.height != 720))
      error = "unverified binning or crop geometry";
    if (error) { ++rejected_infos_; reject("camera_info", error); return; }
    // Includes original stamp, dimensions, distortion model, D/K/R/P, ROI and
    // binning. In particular no 1280->640 intrinsic scaling is performed here.
    message->header.frame_id = kAliasFrame;
    info_publisher_->publish(std::move(message));
    ++infos_;
  }

  std::string expected_image_frame_, expected_info_frame_;
  std::uint64_t images_ = 0, infos_ = 0, rejected_images_ = 0, rejected_infos_ = 0;
  rclcpp::Publisher<Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Subscription<Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try { rclcpp::spin(std::make_shared<FrameAlias>()); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; code = 1; }
  if (rclcpp::ok()) rclcpp::shutdown();
  return code;
}

// Isolated synthetic ROS transport test. Never run against a live vehicle.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

using Odom = nav_msgs::msg::Odometry;
using Trigger = std_srvs::srv::Trigger;
using Clock = std::chrono::steady_clock;
class HeadingResetScenario : public rclcpp::Node {
 public:
  HeadingResetScenario() : Node("synthetic_heading_reset_scenario"), began_(Clock::now()) {
    const auto qos = rclcpp::SensorDataQoS();
    vio_ = create_publisher<Odom>("/synthetic/viewer/vio", qos);
    imu_ = create_publisher<sensor_msgs::msg::Imu>("/synthetic/viewer/imu", qos);
    mag_ = create_publisher<sensor_msgs::msg::MagneticField>("/synthetic/viewer/mag", qos);
    fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("/synthetic/viewer/fix", qos);
    epoch_ = create_publisher<std_msgs::msg::UInt32>("/fusion/vio_epoch", rclcpp::QoS(1).transient_local());
    client_ = create_client<Trigger>("/fusion/reset_heading_from_magnetometer");
    odom_sub_ = create_subscription<Odom>("/nomad/localization/odometry", 100,
        [this](Odom::ConstSharedPtr value) { last_ = *value; have_odom_ = true; });
    status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/fusion/status", 100,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr value) {
          for (const auto &status : value->status) for (const auto &entry : status.values) values_[entry.key] = entry.value;
          if (values_["viewer_frame"] == "odom" && values_["viewer_child_frame"] == "base_link") saw_frame_ = true;
          if (values_["viewer_vio_valid"] == "true") saw_vio_ = true;
          if (values_["viewer_gnss_valid"] == "true") saw_gnss_ = true;
          if (restart_raw_published_ && values_["vio_epoch"] == "23" &&
              values_["viewer_vio_valid"] == "true" && !checked_restart_ &&
              last_.header.stamp.sec == value->header.stamp.sec &&
              last_.header.stamp.nanosec == value->header.stamp.nanosec) {
            const auto &a = before_restart_.pose.pose.position, &b = last_.pose.pose.position;
            restart_position_error_ = std::sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y, 2) + std::pow(a.z-b.z, 2));
            const auto &qa = before_restart_.pose.pose.orientation, &qb = last_.pose.pose.orientation;
            const double dot = std::abs(qa.w*qb.w + qa.x*qb.x + qa.y*qb.y + qa.z*qb.z);
            restart_orientation_error_ = 2. * std::acos(std::min(1., dot));
            checked_restart_ = true;
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
  }
  bool done() const { return elapsed() >= 9.1; }
  int result() {
    int failed = 0;
    auto check = [&](bool condition, const char *name) {
      if (!condition) ++failed;
      std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    };
    check(replies_.size() == 5, "all five service responses received");
    check(replies_.size() > 0 && !replies_[0].first, "initial request blocked without continuous ready input");
    check(replies_.size() > 1 && replies_[1].first, "VIO-only stationary magnetic initialization accepted");
    check(replies_.size() > 2 && !replies_[2].first && replies_[2].second == "HOLD_VEHICLE_STILL", "moving vehicle rejected");
    check(replies_.size() > 3 && !replies_[3].first && replies_[3].second == "VIO_ONLY_REQUIRED", "GNSS-fused mode rejected");
    check(replies_.size() > 4 && !replies_[4].first && replies_[4].second == "VIO_INPUT_NOT_READY", "stale VIO rejected");
    check(checked_pivot_ && pivot_error_ < 1e-5, "nonzero mounting lever retains public base position");
    check(checked_pivot_ && heading_change_ > .1 && heading_change_ < .5, "independent MAG changes heading despite continuous VIO seeds");
    check(saw_frame_ && saw_vio_ && saw_gnss_, "all three comparisons share odom/base_link");
    check(values_["heading_reset_count"] == "1", "rejected requests never increment accepted reset count");
    check(values_["global_heading_valid"] == "false", "provisional magnetic calibration never promoted");
    check(checked_restart_ && restart_position_error_ < 1e-5, "new raw VIO origin after epoch retains last public position");
    check(checked_restart_ && restart_orientation_error_ < 1e-6, "new raw VIO yaw after epoch retains last public direction");
    std::cout << "synthetic_only=true physical_accuracy_validated=false failures=" << failed << '\n';
    return failed ? 1 : 0;
  }
 private:
  double elapsed() const { return std::chrono::duration<double>(Clock::now() - began_).count(); }
  static double yaw(const Odom &pose) {
    const auto &q = pose.pose.pose.orientation;
    return std::atan2(2. * (q.w*q.z + q.x*q.y), 1. - 2. * (q.y*q.y + q.z*q.z));
  }
  void request() {
    if (!client_->service_is_ready()) return;
    ++requested_;
    if (requested_ == 2) before_ = last_;
    client_->async_send_request(std::make_shared<Trigger::Request>(), [this](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      replies_.emplace_back(response->success, response->message);
    });
  }
  void tick() {
    const double t = elapsed();
    if (t < 7.) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = now(); imu.header.frame_id = "synthetic_zed_imu";
      imu.linear_acceleration.z = 9.80665;
      imu.orientation_covariance[0] = -1.; imu_->publish(imu);
      if (ticks_ % 5 == 0) {
        Odom vio;
        vio.header.stamp = imu.header.stamp; vio.header.frame_id = "synthetic_raw_vio";
        vio.child_frame_id = "oak_imu_frame";
        vio.pose.pose.orientation.w = 1.;
        vio.pose.pose.position.x = t < 5. ? 0. : .2 * (t - 5.);
        vio_->publish(vio);
        if (t > 1.5) {
          const double angle = t < 2.8 ? .6 : .9;
          sensor_msgs::msg::MagneticField mag;
          mag.header = imu.header; mag.magnetic_field.x = 50e-6 * std::sin(angle);
          mag.magnetic_field.y = 50e-6 * std::cos(angle); mag.magnetic_field.z = -10e-6;
          mag_->publish(mag);
        }
      }
      if (t > 6. && ticks_ % 10 == 0) {
        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp = now(); fix.header.frame_id = "synthetic_gnss_antenna";
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix.latitude = 37.; fix.longitude = 127.; fix.altitude = 0.;
        fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        fix.position_covariance[0] = fix.position_covariance[4] = .01; fix.position_covariance[8] = .04;
        fix_->publish(fix);
      }
    }
    const double schedule[] = {.8, 4.2, 5.6, 6.6, 8.};
    if (requested_ < 5 && t >= schedule[requested_] && have_odom_) request();
    if (t > 4.7 && t < 5. && !checked_pivot_ && replies_.size() >= 2 && replies_[1].first) {
      const auto &a = before_.pose.pose.position; const auto &b = last_.pose.pose.position;
      pivot_error_ = std::sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y, 2) + std::pow(a.z-b.z, 2));
      heading_change_ = std::abs(std::atan2(std::sin(yaw(last_)-yaw(before_)), std::cos(yaw(last_)-yaw(before_))));
      checked_pivot_ = true;
    }
    // The fusion process stays alive. After the stale-input rejection has
    // completed, reproduce an external VIO restart with a completely different
    // raw coordinate origin/yaw. Wait for the epoch acknowledgment before the
    // first new measurement, matching the operator restart-shell contract.
    if (!restart_epoch_published_ && t > 8.15 && replies_.size() == 5 && epoch_->get_subscription_count() > 0) {
      before_restart_ = last_;
      std_msgs::msg::UInt32 epoch; epoch.data = 23; epoch_->publish(epoch);
      restart_epoch_published_ = true;
    }
    if (restart_epoch_published_ && !restart_raw_published_ && t > 8.3 && values_["vio_epoch"] == "23") {
      Odom restarted;
      restarted.header.stamp = now(); restarted.header.frame_id = "new_synthetic_raw_vio_world";
      restarted.child_frame_id = "oak_imu_frame";
      restarted.pose.pose.position.x = 50.; restarted.pose.pose.position.y = -30.;
      restarted.pose.pose.position.z = 4.;
      restarted.pose.pose.orientation.z = std::sin(-2.2 / 2.);
      restarted.pose.pose.orientation.w = std::cos(-2.2 / 2.);
      vio_->publish(restarted);
      restart_raw_published_ = true;
    }
    ++ticks_;
  }
  Clock::time_point began_;
  Odom last_, before_, before_restart_;
  bool have_odom_ = false, checked_pivot_ = false, saw_frame_ = false, saw_vio_ = false, saw_gnss_ = false;
  double pivot_error_ = 1e9, heading_change_ = 0.;
  bool restart_epoch_published_ = false, restart_raw_published_ = false, checked_restart_ = false;
  double restart_position_error_ = 1e9, restart_orientation_error_ = 1e9;
  int ticks_ = 0, requested_ = 0;
  std::vector<std::pair<bool, std::string>> replies_;
  std::map<std::string, std::string> values_;
  rclcpp::Publisher<Odom>::SharedPtr vio_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr epoch_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;
  rclcpp::Client<Trigger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) {
  const char *enabled = std::getenv("NOMAD_SYNTHETIC_ONLY"), *domain = std::getenv("ROS_DOMAIN_ID");
  if (!enabled || std::string(enabled) != "1" || !domain || std::atoi(domain) < 150 || std::atoi(domain) > 230) {
    std::cerr << "Synthetic test requires NOMAD_SYNTHETIC_ONLY=1 and isolated ROS_DOMAIN_ID=150..230\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeadingResetScenario>();
  while (rclcpp::ok() && !node->done()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const int result = node->result();
  rclcpp::shutdown();
  return result;
}

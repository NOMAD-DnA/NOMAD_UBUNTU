#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/version.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nomad_interfaces/msg/gnss_quality.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nomad_localization/heading_estimator.hpp>
#include <nomad_localization/localization_viewer.hpp>
#include <nomad_localization/public_output.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using V3 = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
using Odom = nav_msgs::msg::Odometry;
using Fix = sensor_msgs::msg::NavSatFix;
using Quality = nomad_interfaces::msg::GnssQuality;
using HeadingEstimator = nomad_localization::HeadingEstimator;
using HeadingConfig = nomad_localization::HeadingConfig;
using HeadingEstimate = nomad_localization::HeadingEstimate;
using ModuleStatus = nomad_interfaces::msg::ModuleStatus;
static constexpr double kPi = 3.14159265358979323846;
static double stamp(const builtin_interfaces::msg::Time &s) {
  return s.sec + s.nanosec * 1e-9;
}
static double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
using Pose = nomad_localization::ViewerPose;
static Pose compose(const Pose &a, const Pose &b) { return {a.p + a.q * b.p, (a.q * b.q).normalized()}; }
static Pose inverse(const Pose &a) { const Q q = a.q.conjugate(); return {-(q * a.p), q}; }
static double yaw(const Q &q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                   1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
static Q yaw_rotation(double angle) { return Q(Eigen::AngleAxisd(angle, V3::UnitZ())); }
static Pose read(const geometry_msgs::msg::Pose &m) {
  return {{m.position.x, m.position.y, m.position.z},
          {m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z}};
}
static bool finite_pose(const Pose &p) {
  const double norm = p.q.norm();
  return p.p.allFinite() && p.q.coeffs().allFinite() && std::isfinite(norm) && norm >= 0.9 && norm <= 1.1;
}
static geometry_msgs::msg::Pose message(const Pose &p) {
  geometry_msgs::msg::Pose m;
  m.position.x = p.p.x(); m.position.y = p.p.y(); m.position.z = p.p.z();
  m.orientation.w = p.q.w(); m.orientation.x = p.q.x();
  m.orientation.y = p.q.y(); m.orientation.z = p.q.z();
  return m;
}
struct GPS { double t; V3 p; V3 sigma; bool trusted; };
struct Frame {
  double t = 0.0;
  Pose raw, local;
  std::array<double, 3> p{};
  std::array<double, 4> q{};
  Pose delta;
  double dt = 0.0;
  std::optional<GPS> gps;
};
static Pose estimate(const Frame &f) {
  return {{f.p[0], f.p[1], f.p[2]}, {f.q[0], f.q[1], f.q[2], f.q[3]}};
}
static void set(Frame &f, const Pose &p) {
  f.p = {p.p.x(), p.p.y(), p.p.z()};
  f.q = {p.q.w(), p.q.x(), p.q.y(), p.q.z()};
}
// Hamilton wxyz quaternions; each pose maps the reference IMU into its world.
struct Relative {
  Pose d; double sp, sr;
  template<typename T> bool operator()(const T *pa, const T *qa, const T *pb, const T *qb, T *r) const {
    T qi[4] = {qa[0], -qa[1], -qa[2], -qa[3]};
    T difference[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]}, v[3], relative[4];
    ceres::QuaternionRotatePoint(qi, difference, v);
    ceres::QuaternionProduct(qi, qb, relative);
    T invd[4] = {T(d.q.w()), T(-d.q.x()), T(-d.q.y()), T(-d.q.z())}, error[4], aa[3];
    ceres::QuaternionProduct(invd, relative, error);
    ceres::QuaternionToAngleAxis(error, aa);
    for (int k = 0; k < 3; ++k) { r[k] = (v[k] - T(d.p[k])) / T(sp); r[k + 3] = aa[k] / T(sr); }
    return true;
  }
};
struct GPSCost {
  V3 p, lever, sigma;
  template<typename T> bool operator()(const T *x, const T *q, T *r) const {
    T l[3] = {T(lever.x()), T(lever.y()), T(lever.z())}, v[3];
    ceres::QuaternionRotatePoint(q, l, v);
    for (int k = 0; k < 3; ++k) r[k] = (x[k] + v[k] - T(p[k])) / T(sigma[k]);
    return true;
  }
};
struct Prior {
  Pose p; double sp, sr;
  template<typename T> bool operator()(const T *x, const T *q, T *r) const {
    T qi[4] = {T(p.q.w()), T(-p.q.x()), T(-p.q.y()), T(-p.q.z())}, error[4], aa[3];
    ceres::QuaternionProduct(qi, q, error);
    ceres::QuaternionToAngleAxis(error, aa);
    for (int k = 0; k < 3; ++k) { r[k] = (x[k] - T(p.p[k])) / T(sp); r[k + 3] = aa[k] / T(sr); }
    return true;
  }
};
struct YawCost {
  double target, sigma;
  template<typename T> bool operator()(const T *q, T *r) const {
    using std::atan2; using std::sin; using std::cos;
    const T angle = atan2(T(2) * (q[0] * q[3] + q[1] * q[2]),
                         T(1) - T(2) * (q[2] * q[2] + q[3] * q[3]));
    r[0] = atan2(sin(angle - T(target)), cos(angle - T(target))) / T(sigma);
    return true;
  }
};

struct SolveParameters {
  V3 lever;
  double translation_sigma, rotation_sigma, robust_sigma, heading_sigma, time_limit;
  int max_iterations;
  std::optional<double> map_yaw;
};
struct WindowSolution {
  std::deque<Frame> frames;
  std::uint64_t generation = 0;
  std::uint32_t epoch = 0;
  double milliseconds = 0;
  bool valid = false;
};

// The worker receives immutable-by-ownership snapshots only. It does not access
// the ROS node, publishers, heading state, live sensor buffers, or parameters.
static WindowSolution optimize_window(WindowSolution result, const SolveParameters parameters) {
  auto &frames = result.frames;
  ceres::Problem problem;
  for (auto &frame : frames) {
    problem.AddParameterBlock(frame.p.data(), 3);
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
    problem.AddParameterBlock(frame.q.data(), 4, new ceres::QuaternionManifold());
#else
    problem.AddParameterBlock(frame.q.data(), 4, new ceres::QuaternionParameterization());
#endif
  }
  // Approximate bounded boundary, not an exact marginalized posterior.
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Prior, 6, 3, 4>(
      new Prior{estimate(frames.front()), .2, .05}), nullptr, frames.front().p.data(), frames.front().q.data());
  for (std::size_t i = 0; i < frames.size(); ++i) {
    auto &frame = frames[i];
    if (i) {
      auto &previous = frames[i - 1];
      const double dt = std::max(.01, frame.dt);
      problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Relative, 6, 3, 4, 3, 4>(new Relative{
          frame.delta, parameters.translation_sigma * std::sqrt(dt), parameters.rotation_sigma * std::sqrt(dt)}),
          new ceres::HuberLoss(parameters.robust_sigma), previous.p.data(), previous.q.data(), frame.p.data(), frame.q.data());
    }
    if (frame.gps) problem.AddResidualBlock(new ceres::AutoDiffCostFunction<GPSCost, 3, 3, 4>(
        new GPSCost{frame.gps->p, parameters.lever, frame.gps->sigma}), new ceres::HuberLoss(parameters.robust_sigma),
        frame.p.data(), frame.q.data());
    if (parameters.map_yaw) problem.AddResidualBlock(new ceres::AutoDiffCostFunction<YawCost, 1, 4>(
        new YawCost{wrap(*parameters.map_yaw + yaw(frame.local.q)), parameters.heading_sigma}),
        new ceres::HuberLoss(parameters.robust_sigma), frame.q.data());
  }
  ceres::Solver::Options options;
  options.max_num_iterations = parameters.max_iterations;
  options.max_solver_time_in_seconds = parameters.time_limit; options.num_threads = 1;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  const auto began = std::chrono::steady_clock::now();
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  result.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
  result.valid = summary.IsSolutionUsable();
  for (const auto &frame : frames) result.valid = result.valid && finite_pose(estimate(frame));
  return result;
}

class Fusion : public rclcpp::Node {
 public:
  Fusion() : Node("fusion") {
    frame_id_ = parameter<std::string>("frame_id", "fusion_map");
    local_id_ = parameter<std::string>("local_frame_id", "fusion_odom");
    raw_local_id_ = parameter<std::string>("raw_local_frame_id", "fusion_vio_odom");
    body_id_ = parameter<std::string>("body_frame_id", "oak_imu_frame");
    const std::vector<std::pair<std::string, double>> defaults = {
      {"publish_hz", 20}, {"window_s", 10}, {"keyframe_hz", 5}, {"solve_hz", 2},
      {"solver_time_s", .05}, {"vio_translation_sigma_per_sqrt_s", .08},
      {"vio_rotation_sigma_per_sqrt_s", .02}, {"gps_horizontal_sigma_m", .02},
      {"gps_vertical_sigma_m", .05}, {"gps_nonfixed_sigma_m", 1.5},
      {"gps_max_horizontal_sigma_m", 20}, {"robust_loss_sigma", 3},
      {"gps_max_speed_mps", 5}, {"gps_jump_margin_m", .5}, {"gps_max_age_s", 1},
      {"gps_reacquire_s", 2}, {"gps_reacquire_agreement_m", .5}, {"quality_max_age_s", .5},
      {"vio_max_speed_mps", 5}, {"vio_max_rotation_rate_rps", 3},
      {"vio_jump_margin_m", .15}, {"vio_stale_s", 2},
      {"restart_disagreement_m", 3}, {"restart_persistence_s", 3}, {"restart_cooldown_s", 15},
      {"alignment_min_span_m", 2}, {"alignment_max_rmse_m", .5},
      {"alignment_heading_sigma_rad", .15}, {"gnss_velocity_max_age_s", .5},
      {"heading_reset_vio_only_s", 1.0}, {"heading_reset_stationary_s", .5},
      {"heading_reset_max_speed_mps", .1}, {"heading_reset_max_rotation_rps", .15}
    };
    for (const auto &entry : defaults) {
      const double value = parameter<double>(entry.first, entry.second);
      if (!std::isfinite(value) || value <= 0.0) throw std::runtime_error("Positive finite parameter required: " + entry.first);
    }
    parameter<int>("max_iterations", 10);
    parameter<int>("max_restarts", 0);  // Compatibility only; no service client or restart action exists.
    parameter<int>("path_max_points", 2000);
    if (parameter<bool>("auto_restart", false)) throw std::runtime_error("auto_restart is prohibited; modules are managed independently");
    parameter<bool>("clear_gps_on_failure", true);
    parameter<bool>("alignment_require_rtk_fixed", true);
    parameter<bool>("use_reported_gps_covariance", true);
    antenna_confirmed_ = parameter<bool>("antenna_offset_confirmed", false);
    if (get("window_s") * get("keyframe_hz") > 500 || integer("max_iterations") < 1 || integer("path_max_points") < 2)
      throw std::runtime_error("Invalid window or integer limit");
    const auto lever = parameter<std::vector<double>>("antenna_in_imu", {0, 0, 0});
    if (lever.size() != 3 || !finite_vector(lever)) throw std::runtime_error("antenna_in_imu must be three finite metres");
    lever_ = V3(lever[0], lever[1], lever[2]);
    datum_mode_ = parameter<std::string>("datum_mode", "first_fix");
    const auto datum = parameter<std::vector<double>>("datum", {});
    if (!datum.empty()) {
      if (datum.size() != 3 || !finite_vector(datum) || std::abs(datum[0]) > 90 || std::abs(datum[1]) > 180)
        throw std::runtime_error("datum must be [latitude_deg,longitude_deg,altitude_m]");
      initialize_datum(datum[0], datum[1], datum[2]);
      datum_mode_ = "explicit";
    } else if (datum_mode_ != "first_fix") throw std::runtime_error("Explicit datum_mode requires a three-value datum");
    configure_heading();
    configure_public_output();
    odom_pub_ = create_publisher<Odom>("/fusion/odometry", 20);
    global_pub_ = create_publisher<Odom>("/fusion/global_odometry", 20);
    local_pub_ = create_publisher<Odom>("/fusion/local_odometry", 20);
    raw_pub_ = create_publisher<Odom>("/fusion/raw_vio_aligned", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/fusion/path", 5);
    window_pub_ = create_publisher<nav_msgs::msg::Path>("/fusion/optimized_window", 5);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fusion/status", 20);
    auto sensor_qos = rclcpp::SensorDataQoS();
    vio_sub_ = create_subscription<Odom>(parameter<std::string>("vio_topic", "/basalt/odom"), sensor_qos,
        [this](Odom::ConstSharedPtr m) { on_vio(*m); });
    gps_sub_ = create_subscription<Fix>(parameter<std::string>("gps_topic", "/gps/fix"), sensor_qos,
        [this](Fix::ConstSharedPtr m) { on_gps(*m); });
    quality_sub_ = create_subscription<Quality>(parameter<std::string>("quality_topic", "/gps/quality"), sensor_qos,
        [this](Quality::ConstSharedPtr m) { on_quality(*m); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(parameter<std::string>("imu_topic", "/zed/zed_node/imu/data_raw"), sensor_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          const V3 gyro(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
          const V3 acceleration(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
          if (!heading_->addImu(stamp(m->header.stamp), gyro, acceleration, now().seconds())) ++rejected_imu_;
          magnetic_heading_->addImu(stamp(m->header.stamp), gyro, acceleration, now().seconds());
        });
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(parameter<std::string>("mag_topic", "/zed/zed_node/imu/mag"), sensor_qos,
        [this](sensor_msgs::msg::MagneticField::ConstSharedPtr m) {
          mag_seen_ = true;
          if (!heading_->addMag(stamp(m->header.stamp), V3(m->magnetic_field.x, m->magnetic_field.y, m->magnetic_field.z), now().seconds())) ++rejected_mag_;
          magnetic_heading_->addMag(stamp(m->header.stamp),
              V3(m->magnetic_field.x, m->magnetic_field.y, m->magnetic_field.z), now().seconds());
        });
    const auto velocity_topic = parameter<std::string>("gnss_velocity_topic", "");
    if (!velocity_topic.empty()) velocity_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        velocity_topic, sensor_qos, [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) { on_velocity(*m); });
    epoch_sub_ = create_subscription<std_msgs::msg::UInt32>("/fusion/vio_epoch", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::UInt32::ConstSharedPtr m) { on_epoch(m->data); });
    heading_reset_service_ = create_service<std_srvs::srv::Trigger>("/fusion/reset_heading_from_magnetometer",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          reset_heading_from_magnetometer(*response);
        });
    publish_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / get("publish_hz"))),
        [this]() { publish_tick(); });
    solve_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / get("solve_hz"))),
        [this]() { solve(); });
    last_reason_ = "Waiting for independent sensor modules; no restart requests will be sent";
  }
  ~Fusion() override { if (pending_solve_.valid()) pending_solve_.wait(); }

 private:
  std::deque<Frame> frames_;
  std::deque<GPS> gps_;
  std::deque<std::pair<Pose, GPS>> alignment_;
  std::optional<GPS> last_gps_sample_;
  std::unique_ptr<HeadingEstimator> heading_;
  // Receives only real ZED IMU/MAG observations. Healthy VIO seeds must never
  // overwrite the independent magnetic observation used by a manual reset.
  std::unique_ptr<HeadingEstimator> magnetic_heading_;
  std::unique_ptr<nomad_localization::PublicOutputAdapter> public_adapter_;
  nomad_localization::PublicOutputConfig public_config_;
  Pose last_public_pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> public_tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> public_static_tf_;
  bool public_frame_contract_valid_ = false;
  HeadingConfig heading_config_;
  Quality quality_;
  double quality_t_ = -1, last_gps_input_ = -1, last_vio_ = -1, last_velocity_ = -1;
  double candidate_t_ = -1, reacquire_since_ = -1, last_alignment_ = -1;
  V3 candidate_ = V3::Zero(), lever_ = V3::Zero(), ecef_origin_ = V3::Zero(), velocity_ = V3::Zero();
  Eigen::Matrix3d enu_ = Eigen::Matrix3d::Identity();
  Pose local_, map_local_, output_from_global_, last_raw_, raw_local_;
  nomad_localization::FrozenVioFrame viewer_vio_frame_;
  Pose viewer_gnss_pose_;
  bool viewer_vio_needs_anchor_ = true, viewer_gnss_finite_ = false;
  double viewer_gnss_t_ = -1., vio_only_since_ = -1., stationary_since_ = -1.;
  bool datum_initialized_ = false, global_translation_initialized_ = false;
  bool have_position_ = false, have_raw_ = false, gps_available_ = false, gps_suspect_ = false;
  bool heading_initialized_ = false, motion_heading_valid_ = false, antenna_confirmed_ = false, mag_seen_ = false;
  bool provisional_heading_initialized_ = false;
  bool position_valid_ = false, orientation_valid_ = false, global_position_valid_ = false, global_heading_valid_ = false;
  std::optional<double> motion_map_yaw_;
  std::uint32_t external_epoch_ = 0, epoch_ = 0;
  std::uint64_t accepted_vio_ = 0, accepted_gps_ = 0, rejected_vio_ = 0, rejected_gps_ = 0, rejected_mag_ = 0, rejected_imu_ = 0;
  std::uint64_t solves_ = 0, cleared_gps_ = 0, reanchors_ = 0;
  std::uint64_t heading_reset_count_ = 0;
  std::string heading_reset_last_result_ = "NOT_REQUESTED";
  std::future<WindowSolution> pending_solve_;
  std::uint64_t generation_ = 0, observation_version_ = 0, last_submitted_version_ = 0, discarded_solves_ = 0;
  double disagreement_ = 0, solve_ms_ = 0;
  std::string frame_id_, local_id_, raw_local_id_, body_id_, datum_mode_, state_ = "WAITING", source_ = "NONE", last_reason_, heading_reason_;
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_, global_pub_, local_pub_, raw_pub_;
  rclcpp::Publisher<Odom>::SharedPtr public_odom_pub_;
  rclcpp::Publisher<ModuleStatus>::SharedPtr public_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, window_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<Odom>::SharedPtr vio_sub_;
  rclcpp::Subscription<Fix>::SharedPtr gps_sub_;
  rclcpp::Subscription<Quality>::SharedPtr quality_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr epoch_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr heading_reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_, solve_timer_;
  template<typename T> T parameter(const std::string &name, T value) { return declare_parameter<T>(name, value); }
  double get(const std::string &name) const { return get_parameter(name).as_double(); }
  std::int64_t integer(const std::string &name) const { return get_parameter(name).as_int(); }
  static bool finite_vector(const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  }
  static double age(double now_s, double measurement) { return measurement >= 0 ? now_s - measurement : -1.0; }
  static std::string flag(bool value) { return value ? "true" : "false"; }
  bool valid_time(double t, double max_age) const {
    const double elapsed = now().seconds() - t;
    return std::isfinite(t) && t > 0 && elapsed >= -0.3 && elapsed <= max_age;
  }
  bool vio_healthy(double t) const { return have_raw_ && last_vio_ > 0 && t - last_vio_ >= -.3 && t - last_vio_ <= get("vio_stale_s"); }
  bool gps_usable(double t) const {
    return gps_available_ && !gps_suspect_ && last_gps_sample_ &&
        t - last_gps_sample_->t >= -.3 && t - last_gps_sample_->t <= get("gps_max_age_s");
  }
  bool trusted(double t) const {
    return quality_t_ > 0 && std::abs(t - quality_t_) <= get("quality_max_age_s") &&
        (quality_.valid_fields & Quality::SOLUTION_VALID) && quality_.solution_type == Quality::RTK_FIXED;
  }
  bool quality_no_fix(double t) const {
    return quality_t_ > 0 && std::abs(t - quality_t_) <= get("quality_max_age_s") &&
        (quality_.valid_fields & Quality::SOLUTION_VALID) && quality_.solution_type == Quality::NO_FIX;
  }
  static V3 ecef(double latitude, double longitude, double altitude) {
    const double p = latitude * kPi / 180.0, l = longitude * kPi / 180.0;
    const double s = std::sin(p), c = std::cos(p);
    const double n = 6378137.0 / std::sqrt(1.0 - 6.69437999014e-3 * s * s);
    return {(n + altitude) * c * std::cos(l), (n + altitude) * c * std::sin(l),
            (n * (1.0 - 6.69437999014e-3) + altitude) * s};
  }
  void initialize_datum(double latitude, double longitude, double altitude) {
    ecef_origin_ = ecef(latitude, longitude, altitude);
    const double p = latitude * kPi / 180.0, l = longitude * kPi / 180.0;
    enu_ << -std::sin(l), std::cos(l), 0,
        -std::sin(p) * std::cos(l), -std::sin(p) * std::sin(l), std::cos(p),
        std::cos(p) * std::cos(l), std::cos(p) * std::sin(l), std::sin(p);
    datum_initialized_ = true;
  }
  void configure_heading() {
    auto rotation = parameter<std::vector<double>>("reference_from_zed_quaternion", {1, 0, 0, 0});
    auto hard = parameter<std::vector<double>>("mag_hard_iron_tesla", {0, 0, 0});
    auto soft = parameter<std::vector<double>>("mag_soft_iron", {1, 0, 0, 0, 1, 0, 0, 0, 1});
    auto bias = parameter<std::vector<double>>("gyro_bias_rad_s", {0, 0, 0});
    if (rotation.size() != 4 || hard.size() != 3 || soft.size() != 9 || bias.size() != 3 ||
        !finite_vector(rotation) || !finite_vector(hard) || !finite_vector(soft) || !finite_vector(bias))
      throw std::runtime_error("Invalid heading calibration vector");
    Q q(rotation[0], rotation[1], rotation[2], rotation[3]);
    if (q.norm() < .9 || q.norm() > 1.1) throw std::runtime_error("reference_from_zed_quaternion must be normalized wxyz");
    heading_config_.q_reference_sensor = q.normalized();
    heading_config_.hard_iron_tesla = V3(hard[0], hard[1], hard[2]);
    heading_config_.gyro_bias_rad_s = V3(bias[0], bias[1], bias[2]);
    for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) heading_config_.soft_iron(row, col) = soft[3 * row + col];
    heading_config_.declination_rad = parameter<double>("magnetic_declination_rad", 0.0);
    heading_config_.calibration_confirmed = parameter<bool>("mag_calibration_confirmed", false);
    heading_config_.extrinsics_confirmed = parameter<bool>("zed_to_reference_rotation_confirmed", false);
    heading_config_.allow_uncalibrated = parameter<bool>("allow_uncalibrated_heading", false);
    heading_config_.max_input_age_s = parameter<double>("heading_max_input_age_s", 1.0);
    heading_config_.gyro_fresh_timeout_s = parameter<double>("gyro_fresh_timeout_s", .3);
    heading_config_.mag_fresh_timeout_s = parameter<double>("mag_fresh_timeout_s", 1.0);
    heading_config_.orientation_stale_timeout_s = parameter<double>("orientation_stale_timeout_s", 2.0);
    heading_config_.gyro_only_timeout_s = parameter<double>("gyro_only_timeout_s", 3.0);
    heading_config_.max_prediction_s = parameter<double>("heading_max_prediction_s", .1);
    if (!std::isfinite(heading_config_.declination_rad)) throw std::runtime_error("Non-finite magnetic declination");
    heading_ = std::make_unique<HeadingEstimator>(heading_config_);
    magnetic_heading_ = std::make_unique<HeadingEstimator>(heading_config_);
  }
  void configure_public_output() {
    const auto translation = parameter<std::vector<double>>("base_to_imu_translation_m", {0, 0, 0});
    const auto rotation = parameter<std::vector<double>>("base_to_imu_quaternion", {1, 0, 0, 0});
    if (translation.size() != 3 || rotation.size() != 4 || !finite_vector(translation) || !finite_vector(rotation))
      throw std::runtime_error("base_to_imu requires three finite metres and a finite wxyz quaternion");
    public_config_.t_base_imu = V3(translation[0], translation[1], translation[2]);
    public_config_.q_base_imu = Q(rotation[0], rotation[1], rotation[2], rotation[3]);
    public_config_.base_imu_transform_confirmed = parameter<bool>("base_to_imu_extrinsics_confirmed", false);
    public_adapter_ = std::make_unique<nomad_localization::PublicOutputAdapter>(public_config_);
    public_config_.q_base_imu.normalize();
    public_frame_contract_valid_ = body_id_ == nomad_localization::kReferenceFrameId &&
        !local_id_.empty() && local_id_ != nomad_localization::kPublicChildFrameId &&
        local_id_ != nomad_localization::kReferenceFrameId;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    public_odom_pub_ = create_publisher<Odom>("/nomad/localization/odometry", qos);
    public_status_pub_ = create_publisher<ModuleStatus>("/nomad/localization/status", qos);
    public_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    public_static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    if (!public_config_.base_imu_transform_confirmed || !public_frame_contract_valid_) return;
    // This node owns only its advertised odom/base/IMU chain. It never starts,
    // stops, or rewrites other TF authorities. Confirmed mounting is required
    // before publishing either static edge, including the internal-frame alias.
    geometry_msgs::msg::TransformStamped mounting;
    mounting.header.stamp.sec = 0; mounting.header.stamp.nanosec = 0;
    mounting.header.frame_id = nomad_localization::kPublicChildFrameId;
    mounting.child_frame_id = nomad_localization::kReferenceFrameId;
    mounting.transform.translation.x = public_config_.t_base_imu.x();
    mounting.transform.translation.y = public_config_.t_base_imu.y();
    mounting.transform.translation.z = public_config_.t_base_imu.z();
    mounting.transform.rotation.w = public_config_.q_base_imu.w();
    mounting.transform.rotation.x = public_config_.q_base_imu.x();
    mounting.transform.rotation.y = public_config_.q_base_imu.y();
    mounting.transform.rotation.z = public_config_.q_base_imu.z();
    std::vector<geometry_msgs::msg::TransformStamped> transforms{mounting};
    if (local_id_ != nomad_localization::kPublicFrameId) {
      geometry_msgs::msg::TransformStamped alias;
      alias.header.stamp.sec = 0; alias.header.stamp.nanosec = 0;
      alias.header.frame_id = nomad_localization::kPublicFrameId;
      alias.child_frame_id = local_id_;
      alias.transform.rotation.w = 1.0;
      transforms.push_back(alias);
    }
    public_static_tf_->sendTransform(transforms);
  }
  void on_quality(const Quality &m) {
    const double t = stamp(m.header.stamp);
    if (!valid_time(t, get("quality_max_age_s")) || t <= quality_t_) return;
    quality_ = m; quality_t_ = t;
    if (quality_no_fix(t)) gps_failure("Fresh GNSS quality explicitly reports NO_FIX");
  }
  std::optional<GPS> gps_at(double t) const {
    if (gps_.empty()) return {};
    auto it = std::lower_bound(gps_.begin(), gps_.end(), t, [](const GPS &g, double value) { return g.t < value; });
    if (it == gps_.begin()) return std::abs(it->t - t) <= .15 ? std::optional<GPS>(*it) : std::nullopt;
    if (it == gps_.end()) return t - gps_.back().t <= .15 ? std::optional<GPS>(gps_.back()) : std::nullopt;
    const auto &a = *(it - 1), &b = *it;
    if (b.t - a.t > std::min(2.0, get("gps_max_age_s") * 2.0)) return {};
    const double weight = (t - a.t) / (b.t - a.t);
    return GPS{t, (1.0 - weight) * a.p + weight * b.p,
        a.sigma.cwiseMax(b.sigma), a.trusted && b.trusted};
  }
  void gps_failure(const std::string &reason, bool suspect = false) {
    gps_available_ = false;
    if (suspect) gps_suspect_ = true;
    last_reason_ = reason;
    if (get_parameter("clear_gps_on_failure").as_bool()) {
      bool removed = !gps_.empty() || motion_map_yaw_.has_value();
      gps_.clear();
      alignment_.clear();
      motion_map_yaw_.reset();  // This is also a GNSS-derived observation cache.
      for (auto &frame : frames_) { removed = removed || frame.gps.has_value(); frame.gps.reset(); }
      if (removed) { ++cleared_gps_; ++generation_; ++observation_version_; }
      // Keep optimized states and map->odom unchanged: removing a residual is
      // not a request to roll back a previously accepted pose correction.
    }
  }
  Pose global_pose() const { return compose(map_local_, local_); }
  void apply_map_transform(const Pose &transform) {
    ++generation_;
    const Pose correction = compose(transform, inverse(map_local_));
    for (auto &frame : frames_) set(frame, compose(correction, estimate(frame)));
    map_local_ = transform;
  }
  Pose viewer_base_pose(const Pose &reference) const {
    const auto output = public_adapter_->make(reference.p, reference.q, true, true);
    return {output.p_world_base, output.q_world_base};
  }
  void anchor_viewer_vio() {
    if (!have_raw_) return;
    viewer_vio_frame_.anchor(last_raw_, compose(output_from_global_, global_pose()));
    viewer_vio_needs_anchor_ = false;
  }
  void cache_viewer_gnss(const GPS &measurement) {
    // The fix is ENU at its original timestamp. Position is never replaced by
    // the fused translation. Only the antenna->IMU and IMU->base lever arms
    // need orientation; use the nearest existing keyframe when available.
    Pose attitude = global_pose();
    double attitude_age = std::abs(measurement.t - last_vio_);
    for (const auto &frame : frames_) {
      const double elapsed = std::abs(frame.t - measurement.t);
      if (elapsed < attitude_age) { attitude = estimate(frame); attitude_age = elapsed; }
    }
    if (!vio_healthy(now().seconds())) {
      const auto magnetic = magnetic_heading_->estimate(now().seconds());
      if (magnetic.valid && std::abs(magnetic.timestamp_s - measurement.t) < attitude_age) {
        attitude.q = magnetic.q_world_reference;
        attitude_age = std::abs(magnetic.timestamp_s - measurement.t);
      }
    }
    const bool orientation_required = lever_.norm() > 1e-9 || public_config_.t_base_imu.norm() > 1e-9;
    viewer_gnss_finite_ = !orientation_required || attitude_age <= .3;
    const Pose reference{measurement.p - attitude.q * lever_, attitude.q};
    viewer_gnss_pose_ = viewer_base_pose(compose(output_from_global_, reference));
    viewer_gnss_finite_ = viewer_gnss_finite_ && finite_pose(viewer_gnss_pose_);
    viewer_gnss_t_ = measurement.t;
  }
  void update_vio_only_duration(double t) {
    if (!vio_healthy(t) || gps_usable(t)) vio_only_since_ = -1.;
    else if (vio_only_since_ < 0.) vio_only_since_ = t;
  }
  nomad_localization::HeadingResetInputs heading_reset_inputs(double t) const {
    nomad_localization::HeadingResetInputs inputs;
    inputs.vio_healthy = vio_healthy(t) && t - last_vio_ >= -heading_config_.future_tolerance_s;
    inputs.gnss_usable = gps_usable(t);
    inputs.have_pose = have_position_ && have_raw_;
    inputs.frame_valid = public_frame_contract_valid_;
    inputs.vio_only_duration_s = vio_only_since_ < 0. ? -1. : t - vio_only_since_;
    // The final stationary sample must itself remain fresh; a paused VIO or
    // public zero/invalid twist can never manufacture a stationary hold.
    inputs.stationary_duration_s = stationary_since_ < 0. ? -1. : last_vio_ - stationary_since_;
    inputs.required_vio_only_s = get("heading_reset_vio_only_s");
    inputs.required_stationary_s = get("heading_reset_stationary_s");
    inputs.magnetic = magnetic_heading_->estimate(t);
    return inputs;
  }
  void reset_heading_from_magnetometer(std_srvs::srv::Trigger::Response &response) {
    const double t = now().seconds();
    const auto inputs = heading_reset_inputs(t);
    const auto blocked = nomad_localization::headingResetBlockReason(inputs);
    if (!blocked.empty()) {
      response.success = false;
      response.message = heading_reset_last_result_ = blocked;
      return;
    }
    const Pose desired = nomad_localization::yawResetReferencePose(
        global_pose(), HeadingEstimator::yaw_of(inputs.magnetic.q_world_reference),
        public_config_.t_base_imu, public_config_.q_base_imu);
    // Reuse the normal map correction path: its generation invalidation keeps
    // any already-running Ceres result from undoing this explicit user action.
    apply_map_transform(compose(desired, inverse(local_)));
    alignment_.clear(); motion_map_yaw_.reset(); motion_heading_valid_ = false; last_alignment_ = -1.;
    heading_initialized_ = inputs.magnetic.global_heading_valid;
    provisional_heading_initialized_ = !heading_initialized_;
    // Preserve the actual observation timestamp/provenance. Calling seed at
    // last_vio_ again would be a duplicate timestamp, while stamping it "now"
    // would invent a fresh VIO measurement. The next genuine VIO pose resumes
    // the usual attitude-seed path without resetting the independent helper.
    heading_ = std::make_unique<HeadingEstimator>(*magnetic_heading_);
    anchor_viewer_vio();
    path_.poses.clear();
    ++heading_reset_count_;
    ++observation_version_;
    response.success = true;
    response.message = heading_reset_last_result_ = heading_initialized_ ?
        "MAG_HEADING_INITIALIZED_POSITION_PRESERVED" : "PROVISIONAL_MAG_HEADING_INITIALIZED_POSITION_PRESERVED";
    last_reason_ = response.message;
    RCLCPP_INFO(get_logger(), "Manual magnetic heading initialization accepted; public base position retained, viewer segment=%llu, calibration_confirmed=%s",
        static_cast<unsigned long long>(viewer_vio_frame_.epoch()), flag(heading_initialized_).c_str());
  }
  void initialize_global_translation(const GPS &measurement) {
    if (global_translation_initialized_) return;
    const Pose previous_output = compose(output_from_global_, global_pose());
    Pose transform = map_local_;
    const Q global_q = transform.q * local_.q;
    transform.p = measurement.p - global_q * lever_ - transform.q * local_.p;
    apply_map_transform(transform);
    // A late first fix changes the internal ENU embedding, not the already
    // established output origin. Freeze this anchor after first GNSS adoption;
    // subsequent GNSS/Ceres corrections then remain visible in primary output.
    output_from_global_ = compose(previous_output, inverse(global_pose()));
    global_translation_initialized_ = true;
    last_reason_ = "First GNSS anchored global translation; continuous local odometry retained";
  }
  void on_gps(const Fix &m) {
    const double t = stamp(m.header.stamp);
    if (!valid_time(t, get("gps_max_age_s")) || t <= last_gps_input_) {
      ++rejected_gps_;
      last_reason_ = "GNSS packet timestamp stale, future, duplicate or reversed; latest accepted fix retained";
      return;
    }
    last_gps_input_ = t;
    if (quality_no_fix(t)) {
      ++rejected_gps_; gps_failure("GNSS fix rejected because fresh quality reports NO_FIX"); return;
    }
    if (m.status.status < 0 || !std::isfinite(m.latitude) || !std::isfinite(m.longitude) ||
        !std::isfinite(m.altitude) || std::abs(m.latitude) > 90 || std::abs(m.longitude) > 180) {
      ++rejected_gps_; gps_failure("GNSS no fix or invalid coordinates"); return;
    }
    V3 sigma(get("gps_horizontal_sigma_m"), get("gps_horizontal_sigma_m"), get("gps_vertical_sigma_m"));
    if (get_parameter("use_reported_gps_covariance").as_bool() && m.position_covariance_type != Fix::COVARIANCE_TYPE_UNKNOWN) {
      for (double value : m.position_covariance) {
        if (!std::isfinite(value)) { ++rejected_gps_; gps_failure("GNSS covariance is non-finite"); return; }
      }
      for (int i = 0; i < 3; ++i) {
        const double variance = m.position_covariance[4 * i];
        if (variance < 0) { ++rejected_gps_; gps_failure("GNSS covariance diagonal is negative"); return; }
        sigma[i] = std::max(sigma[i], std::sqrt(variance));
      }
    }
    if (!trusted(t)) sigma = sigma.cwiseMax(V3::Constant(get("gps_nonfixed_sigma_m")));
    if (sigma.head<2>().maxCoeff() > get("gps_max_horizontal_sigma_m")) {
      ++rejected_gps_; gps_failure("GNSS uncertainty exceeds configured quality bound"); return;
    }
    if (!datum_initialized_) initialize_datum(m.latitude, m.longitude, m.altitude);
    const V3 p = enu_ * (ecef(m.latitude, m.longitude, m.altitude) - ecef_origin_);
    if (!p.allFinite()) { ++rejected_gps_; gps_failure("GNSS coordinate conversion is non-finite"); return; }
    bool jump = false;
    if (last_gps_sample_) {
      const double dt = t - last_gps_sample_->t;
      if (dt > 0 && dt <= get("gps_max_age_s") * 2.0)
        jump = (p - last_gps_sample_->p).head<2>().norm() >
            get("gps_max_speed_mps") * dt + get("gps_jump_margin_m") + 3.0 * sigma.head<2>().norm();
    }
    if (jump || gps_suspect_) {
      ++rejected_gps_;
      const double dt = t - candidate_t_;
      if (candidate_t_ < 0 || dt <= 0 || dt > get("gps_max_age_s") * 2.0 ||
          (p - candidate_).head<2>().norm() > get("gps_max_speed_mps") * dt + get("gps_jump_margin_m")) reacquire_since_ = t;
      candidate_ = p; candidate_t_ = t;
      gps_failure("GNSS jump: waiting for an independently continuous reacquisition interval", true);
      // The candidate stream is checked against itself, not against a possibly
      // failed VIO. Recovery therefore does not require agreement with dead VIO.
      if (reacquire_since_ < 0 || t - reacquire_since_ < get("gps_reacquire_s")) return;
      gps_suspect_ = false;
      reacquire_since_ = -1;
    }
    GPS measurement{t, p, sigma, trusted(t)};
    last_gps_sample_ = measurement; gps_available_ = true; ++accepted_gps_;
    ++observation_version_;
    gps_.push_back(measurement);
    while (gps_.size() > 200) gps_.pop_front();
    if (!have_position_) {
      local_.p.setZero();
      have_position_ = true;
      const auto orientation = heading_->estimate(now().seconds());
      if (orientation.valid) local_.q = map_local_.q.conjugate() * orientation.q_world_reference;
    }
    initialize_global_translation(measurement);
    for (auto &frame : frames_) { auto value = gps_at(frame.t); if (value) frame.gps = *value; }
    cache_viewer_gnss(measurement);
    update_vio_only_duration(now().seconds());
  }
  void on_velocity(const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
    const double t = stamp(m.header.stamp);
    const V3 value(m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z);
    if (!valid_time(t, get("gnss_velocity_max_age_s")) || t <= last_velocity_ || !value.allFinite() ||
        value.norm() > get("gps_max_speed_mps")) return;
    // An explicitly configured velocity input must already be ENU in frame_id.
    if (m.header.frame_id != frame_id_) { last_reason_ = "GNSS velocity frame is not fusion_map; sample ignored"; return; }
    for (int i = 0; i < 3; ++i) if (!std::isfinite(m.twist.covariance[7 * i]) || m.twist.covariance[7 * i] < 0) return;
    last_velocity_ = t; velocity_ = value;
  }
  void on_epoch(std::uint32_t value) {
    if (value == external_epoch_) return;
    external_epoch_ = value; ++epoch_;
    ++generation_;
    frames_.clear(); alignment_.clear(); have_raw_ = false;
    viewer_vio_needs_anchor_ = true;
    stationary_since_ = vio_only_since_ = -1.;
    motion_heading_valid_ = false; motion_map_yaw_.reset();
    last_vio_ = -1;
    last_reason_ = "External VIO epoch changed; next pose will attach to current local state";
  }
  void anchor_raw(const Pose &raw, bool restarting) {
    if (!have_position_) {
      local_.p.setZero(); have_position_ = true;
    }
    if (accepted_vio_ == 0) {
      // Gravity determines tilt; an arbitrary VIO startup yaw does not define
      // geographic heading. Before a real heading exists the output yaw is 0.
      const Q tilt = yaw_rotation(-yaw(raw.q)) * raw.q;
      const Q desired_global = (heading_initialized_ || provisional_heading_initialized_) ?
          yaw_rotation(yaw(global_pose().q)) * tilt : output_from_global_.q.conjugate() * tilt;
      local_.q = (map_local_.q.conjugate() * desired_global).normalized();
    }
    raw_local_ = compose(local_, inverse(raw));
    viewer_vio_needs_anchor_ = true;
    stationary_since_ = -1.;
    last_raw_ = raw; have_raw_ = true;
    frames_.clear(); alignment_.clear();
    motion_map_yaw_.reset(); motion_heading_valid_ = false; last_alignment_ = -1;
    ++generation_;
    if (restarting) { ++epoch_; ++reanchors_; last_reason_ = "VIO resumed or reset; new raw origin attached without a local pose jump"; }
  }
  void adopt_initial_heading(double t) {
    if (heading_initialized_ || !have_position_) return;
    const auto orientation = heading_->estimate(t);
    const bool trusted = orientation.global_heading_valid;
    const bool provisional = heading_config_.allow_uncalibrated && orientation.using_uncalibrated &&
        orientation.mag_fresh && orientation.anchor_source == nomad_localization::HeadingSource::MAGNETOMETER;
    if (!orientation.valid || (!trusted && !provisional) ||
        (provisional_heading_initialized_ && !trusted)) return;
    const Pose current = global_pose();
    Pose transform = map_local_;
    transform.q = yaw_rotation(wrap(yaw(orientation.q_world_reference) - yaw(current.q))) * transform.q;
    transform.p = current.p - transform.q * local_.p;
    apply_map_transform(transform);
    // A development heading supplies an initial direction, never proof of
    // calibration. Keep the trusted flag false so GNSS motion can still align
    // the frames later and VIO seeds cannot promote this to a trusted heading.
    heading_initialized_ = trusted;
    provisional_heading_initialized_ = !trusted;
    // A delayed first magnetic heading changes the initial comparison axes.
    // Begin a fresh displayed reference segment instead of splicing axes.
    viewer_vio_needs_anchor_ = true;
    last_reason_ = trusted ? "Global yaw initialized from accepted magnetometer heading" :
        "Provisional initial yaw from uncalibrated magnetometer; GNSS/VIO motion can refine it";
  }
  void collect_alignment(double t) {
    if (!gps_usable(now().seconds())) return;
    auto gps = gps_at(t);
    if (!gps || (get_parameter("alignment_require_rtk_fixed").as_bool() && !gps->trusted)) return;
    if (!alignment_.empty() && t - alignment_.back().second.t < 1.0 / get("keyframe_hz")) return;
    alignment_.push_back({local_, *gps});
    while (alignment_.size() > 150) alignment_.pop_front();
    if (alignment_.size() < 8 || (last_alignment_ >= 0 && t - last_alignment_ < 1.0 / get("solve_hz"))) return;
    V3 center_local = V3::Zero(), center_global = V3::Zero();
    for (const auto &entry : alignment_) { center_local += entry.first.p + entry.first.q * lever_; center_global += entry.second.p; }
    center_local /= static_cast<double>(alignment_.size()); center_global /= static_cast<double>(alignment_.size());
    double a = 0, b = 0, span = 0, local_span = 0;
    for (const auto &entry : alignment_) {
      const V3 local = entry.first.p + entry.first.q * lever_ - center_local;
      const V3 global = entry.second.p - center_global;
      a += local.x() * global.x() + local.y() * global.y();
      b += local.x() * global.y() - local.y() * global.x();
      span = std::max(span, (entry.second.p - alignment_.front().second.p).head<2>().norm());
      local_span = std::max(local_span, (entry.first.p - alignment_.front().first.p).head<2>().norm());
    }
    if (span < get("alignment_min_span_m") || local_span < get("alignment_min_span_m") || std::hypot(a, b) < 1e-9) return;
    const Q rotation = yaw_rotation(std::atan2(b, a));
    const V3 translation = center_global - rotation * center_local;
    double error = 0;
    for (const auto &entry : alignment_)
      error += (rotation * (entry.first.p + entry.first.q * lever_) + translation - entry.second.p).head<2>().squaredNorm();
    error = std::sqrt(error / static_cast<double>(alignment_.size()));
    if (!std::isfinite(error) || error > get("alignment_max_rmse_m")) { last_reason_ = "Motion heading observation rejected by GNSS/VIO fit RMSE"; return; }
    motion_map_yaw_ = yaw(rotation); motion_heading_valid_ = true; last_alignment_ = t;
    if (!heading_initialized_) {
      apply_map_transform({translation, rotation}); heading_initialized_ = true;
      provisional_heading_initialized_ = false;
      last_reason_ = "Global yaw initialized by GNSS/VIO motion; output was already active before alignment";
    } else last_reason_ = "GNSS/VIO motion supplied a soft yaw observation for the Ceres window";
  }
  void on_vio(const Odom &m) {
    const double t = stamp(m.header.stamp);
    Pose raw = read(m.pose.pose);
    if (!valid_time(t, get("vio_stale_s")) || t <= last_vio_ || !finite_pose(raw)) {
      ++rejected_vio_; last_reason_ = "VIO timestamp or pose invalid; no automatic process restart"; return;
    }
    raw.q.normalize();
    const bool healthy_before = vio_healthy(now().seconds());
    bool discontinuity = false;
    Pose delta;
    if (have_raw_) {
      delta = compose(inverse(last_raw_), raw);
      const double dt = t - last_vio_;
      discontinuity = delta.p.norm() > get("vio_max_speed_mps") * dt + get("vio_jump_margin_m") ||
          Eigen::AngleAxisd(delta.q).angle() > get("vio_max_rotation_rate_rps") * dt + .05;
      const bool stationary = healthy_before && !discontinuity && dt > 0. &&
          delta.p.norm() / dt <= get("heading_reset_max_speed_mps") &&
          Eigen::AngleAxisd(delta.q).angle() / dt <= get("heading_reset_max_rotation_rps");
      if (!stationary) stationary_since_ = -1.;
      else if (stationary_since_ < 0.) stationary_since_ = t;
    }
    if (!have_raw_ || !healthy_before || discontinuity) {
      const bool restarting = have_raw_ || accepted_vio_ > 0;
      if (discontinuity) ++rejected_vio_;
      anchor_raw(raw, restarting);
    } else local_ = compose(local_, delta);
    last_raw_ = raw; last_vio_ = t; ++accepted_vio_;
    adopt_initial_heading(now().seconds());
    collect_alignment(t);
    if (viewer_vio_needs_anchor_ || !viewer_vio_frame_.initialized()) anchor_viewer_vio();
    update_vio_only_duration(now().seconds());
    // Healthy VIO is the attitude source. The external gyro only receives an
    // attitude seed here; it is not fused a second time into healthy VIO.
    heading_->seed(t, global_pose().q, heading_initialized_ || motion_heading_valid_);
    const Pose global = global_pose();
    if (gps_usable(now().seconds())) disagreement_ = (global.p + global.q * lever_ - last_gps_sample_->p).head<2>().norm();
    if (frames_.empty() || t - frames_.back().t >= 1.0 / get("keyframe_hz")) {
      Frame frame;
      frame.t = t; frame.raw = raw; frame.local = local_; frame.gps = gps_at(t);
      frame.dt = frames_.empty() ? .2 : t - frames_.back().t;
      frame.delta = frames_.empty() ? Pose{} : compose(inverse(frames_.back().raw), raw);
      set(frame, global); frames_.push_back(frame);
      ++observation_version_;
      while (frames_.size() > 2 && t - frames_.front().t > get("window_s")) frames_.pop_front();
    }
    Odom aligned = m;
    aligned.header.frame_id = local_id_; aligned.child_frame_id = body_id_;
    aligned.pose.pose = message(compose(output_from_global_, compose(map_local_, compose(raw_local_, raw))));
    raw_pub_->publish(aligned);
  }
  void update_fallback(double t, const HeadingEstimate &heading, bool use_heading) {
    Pose global = global_pose();
    if (use_heading) global.q = heading.q_world_reference.normalized();
    // Empty velocity topic means hold the latest measured GNSS position. An
    // optional ENU velocity is used only while both it and the fix remain fresh.
    if (gps_usable(t)) {
      V3 antenna = last_gps_sample_->p;
      if (last_velocity_ > 0 && t - last_velocity_ >= 0 && t - last_velocity_ <= get("gnss_velocity_max_age_s"))
        antenna += velocity_ * std::max(0.0, t - last_gps_sample_->t);
      global.p = antenna - global.q * lever_;
      have_position_ = true;
    }
    local_ = compose(inverse(map_local_), global);
  }
  void publish_tick() {
    const double t = now().seconds();
    if (gps_available_ && !gps_usable(t)) gps_failure("GNSS stream expired; window GPS residuals/cache removed without pose rollback");
    collect_solve_result(t);
    const bool vio = vio_healthy(t), gps = gps_usable(t);
    update_vio_only_duration(t);
    adopt_initial_heading(t);
    const auto heading = heading_->estimate(t);
    const bool heading_input_fresh =
        (heading.source == nomad_localization::HeadingSource::GYRO && heading.gyro_fresh) ||
        (heading.source == nomad_localization::HeadingSource::MAGNETOMETER && heading.mag_fresh);
    const bool fallback_heading_valid = heading.valid && heading_input_fresh &&
        (heading_config_.extrinsics_confirmed || heading_config_.allow_uncalibrated);
    if (!vio) update_fallback(t, heading, fallback_heading_valid);
    position_valid_ = vio || gps;
    orientation_valid_ = vio || fallback_heading_valid;
    global_position_valid_ = position_valid_ && datum_initialized_ && global_translation_initialized_;
    global_heading_valid_ = vio ? (heading_initialized_ || motion_heading_valid_) :
        (fallback_heading_valid && heading.global_heading_valid);
    source_ = vio ? (gps ? "VIO+GNSS" : "VIO") :
        (gps ? (fallback_heading_valid ? "GNSS+IMU/MAG" : "GNSS") : (fallback_heading_valid ? "IMU/MAG" : "NONE"));
    state_ = vio ? (gps ? "FUSED" : "VIO_ONLY") :
        (gps ? (fallback_heading_valid ? "GNSS_HEADING" : "GNSS_ONLY") : (have_position_ ? "NO_POSITION" : "WAITING"));
    if (!vio && !fallback_heading_valid) heading_reason_ = "No fresh external gyro/MAG attitude; last orientation held and invalid";
    else if (provisional_heading_initialized_ && !heading_initialized_)
      heading_reason_ = "Provisional initial MAG direction; calibration unconfirmed, GNSS/VIO motion can refine it";
    else if (!heading_initialized_ && !mag_seen_) heading_reason_ = "MAG absent; temporary yaw is not a global heading";
    else if (!heading_initialized_ && mag_seen_ && !heading_config_.allow_uncalibrated &&
        (!heading_config_.calibration_confirmed || !heading_config_.extrinsics_confirmed))
      heading_reason_ = "MAG received but calibration/extrinsics unconfirmed; global yaw remains provisional";
    else heading_reason_ = heading.last_rejection;
    publish_pose(t);
    publish_status(t, heading);
  }
  void publish_pose(double t) {
    const Pose global = global_pose();
    const Pose primary = compose(output_from_global_, global);
    Odom output;
    output.header.stamp = rclcpp::Time(static_cast<std::int64_t>(t * 1e9));
    output.header.frame_id = local_id_; output.child_frame_id = body_id_; output.pose.pose = message(primary);
    for (int i = 0; i < 6; ++i) {
      const bool valid = i < 3 ? position_valid_ : orientation_valid_;
      output.pose.covariance[7 * i] = valid ? 1.0 : 1e6;
      output.twist.covariance[7 * i] = 1e6;
    }
    odom_pub_->publish(output);
    publish_public_output(output, primary);
    output.header.frame_id = frame_id_; output.pose.pose = message(global); global_pub_->publish(output);
    output.header.frame_id = raw_local_id_; output.pose.pose = message(local_); local_pub_->publish(output);
    if (position_valid_ && (path_.poses.empty() || t - stamp(path_.poses.back().header.stamp) >= .19)) {
      geometry_msgs::msg::PoseStamped point;
      point.header.frame_id = local_id_; point.header.stamp = output.header.stamp; point.pose = message(primary);
      path_.poses.push_back(point);
      if (path_.poses.size() > static_cast<std::size_t>(integer("path_max_points"))) path_.poses.erase(path_.poses.begin());
    }
    path_.header.frame_id = local_id_; path_.header.stamp = output.header.stamp; path_pub_->publish(path_);
  }
  void publish_public_output(const Odom &imu_output, const Pose &primary) {
    nomad_localization::PoseCovariance imu_covariance;
    for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col)
      imu_covariance(row, col) = imu_output.pose.covariance[6 * row + col];
    const auto common = public_adapter_->make(primary.p, primary.q,
        position_valid_ && public_frame_contract_valid_, orientation_valid_ && public_frame_contract_valid_, imu_covariance);
    Odom output;
    output.header.stamp = imu_output.header.stamp;
    output.header.frame_id = nomad_localization::kPublicFrameId;
    output.child_frame_id = nomad_localization::kPublicChildFrameId;
    if (common.pose_finite) last_public_pose_ = {common.p_world_base, common.q_world_base};
    output.pose.pose = message(last_public_pose_);
    for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col) {
      output.pose.covariance[6 * row + col] = common.pose_covariance(row, col);
      output.twist.covariance[6 * row + col] = common.twist_covariance(row, col);
    }
    // No base velocity is estimated in this adapter. A zero numeric twist
    // with the helper's large covariance is explicitly marked invalid below.
    public_odom_pub_->publish(output);
    ModuleStatus status;
    status.header = output.header;
    status.state = !common.pose_finite ? ModuleStatus::FAULT : (common.ready ? ModuleStatus::READY : ModuleStatus::WAITING);
    status.valid_for.sec = 0; status.valid_for.nanosec = 300000000U;
    status.detail = "mode=" + state_ + ";position_valid=" + flag(position_valid_) +
        ";orientation_valid=" + flag(orientation_valid_) +
        ";base_to_imu_extrinsics_confirmed=" + flag(common.base_imu_transform_confirmed) +
        ";reference_frame_valid=" + flag(public_frame_contract_valid_) +
        ";global_heading_valid=" + flag(global_heading_valid_) +
        ";twist_valid=false;covariance=conservative_placeholder_not_calibrated_posterior";
    public_status_pub_->publish(status);
    if (common.ready) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = common.p_world_base.x();
      transform.transform.translation.y = common.p_world_base.y();
      transform.transform.translation.z = common.p_world_base.z();
      transform.transform.rotation = output.pose.pose.orientation;
      public_tf_->sendTransform(transform);
    }
  }
  void publish_status(double t, const HeadingEstimate &heading) {
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = rclcpp::Time(static_cast<std::int64_t>(std::max(0.0, t) * 1e9));
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fusion"; status.hardware_id = "external_vio_gnss"; status.message = state_;
    status.level = position_valid_ && orientation_valid_ ? 0 : 1;
    auto add = [&](const std::string &key, const std::string &value) {
      diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value; status.values.push_back(item);
    };
    add("mode", state_); add("source", source_); add("reason", last_reason_);
    add("heading_reason", heading_reason_);
    add("output_frame", local_id_); add("output_coordinates", "fixed_local_anchor_fused_pose_metres");
    add("position_valid", flag(position_valid_)); add("orientation_valid", flag(orientation_valid_));
    add("global_position_valid", flag(global_position_valid_)); add("global_heading_valid", flag(global_heading_valid_));
    add("heading_provisional", flag(!global_heading_valid_ && orientation_valid_ &&
        (provisional_heading_initialized_ || heading.using_uncalibrated)));
    add("datum_initialized", flag(datum_initialized_)); add("datum_mode", datum_mode_);
    add("epoch", std::to_string(epoch_)); add("vio_epoch", std::to_string(external_epoch_));
    add("reanchors", std::to_string(reanchors_)); add("restarts", "0"); add("auto_restart", "false");
    add("vio_age_s", std::to_string(age(t, last_vio_)));
    add("gnss_age_s", std::to_string(last_gps_sample_ ? age(t, last_gps_sample_->t) : -1));
    add("gyro_age_s", std::to_string(heading.gyro_age_s)); add("mag_age_s", std::to_string(heading.mag_age_s));
    add("heading_age_s", std::to_string(heading.heading_age_s));
    add("heading_source", nomad_localization::headingSourceName(heading.source));
    add("heading_using_uncalibrated", flag(heading.using_uncalibrated ||
        (provisional_heading_initialized_ && !heading_initialized_)));
    add("mag_last_rejection", heading.last_rejection);
    add("mag_calibration_confirmed", flag(heading.calibration_confirmed));
    add("zed_to_reference_rotation_confirmed", flag(heading.extrinsics_confirmed));
    add("antenna_offset_confirmed", flag(antenna_confirmed_));
    add("calibration_confirmed", flag(heading.calibration_confirmed && heading.extrinsics_confirmed && antenna_confirmed_));
    add("gps_accepted", std::to_string(accepted_gps_)); add("vio_accepted", std::to_string(accepted_vio_));
    add("rejected_gps", std::to_string(rejected_gps_)); add("rejected_vio", std::to_string(rejected_vio_));
    add("rejected_mag", std::to_string(rejected_mag_)); add("rejected_imu", std::to_string(rejected_imu_));
    add("cleared_gps", std::to_string(cleared_gps_)); add("solves", std::to_string(solves_));
    add("discarded_solves", std::to_string(discarded_solves_));
    add("solve_ms", std::to_string(solve_ms_)); add("raw_vio_gps_disagreement_m", std::to_string(disagreement_));
    add("gnss_healthy", flag(gps_usable(t) && trusted(t)));
    const auto reset_inputs = heading_reset_inputs(t);
    const std::string reset_blocked = nomad_localization::headingResetBlockReason(reset_inputs);
    add("heading_reset_available", flag(reset_blocked.empty()));
    add("heading_reset_reason", reset_blocked.empty() ? "READY" : reset_blocked);
    add("heading_reset_count", std::to_string(heading_reset_count_));
    add("heading_reset_last_result", heading_reset_last_result_);
    add("heading_reset_service", "/fusion/reset_heading_from_magnetometer");
    add("heading_reset_vio_only_s", std::to_string(reset_inputs.vio_only_duration_s));
    add("heading_reset_stationary_s", std::to_string(reset_inputs.stationary_duration_s));
    add("heading_reset_required_vio_only_s", std::to_string(reset_inputs.required_vio_only_s));
    add("heading_reset_required_stationary_s", std::to_string(reset_inputs.required_stationary_s));
    add("heading_reset_max_speed_mps", std::to_string(get("heading_reset_max_speed_mps")));
    add("heading_reset_max_rotation_rps", std::to_string(get("heading_reset_max_rotation_rps")));
    add("heading_reset_mag_age_s", std::to_string(reset_inputs.magnetic.mag_age_s));
    add("viewer_frame", nomad_localization::kPublicFrameId);
    add("viewer_child_frame", nomad_localization::kPublicChildFrameId);
    add("viewer_epoch", std::to_string(viewer_vio_frame_.epoch()));
    add("viewer_vio_alignment", "frozen_first_local_output_per_segment_no_gnss_optimization");
    auto viewer_point = [&](const std::string &name, bool valid, double sample_t, const Pose &pose) {
      add("viewer_" + name + "_valid", flag(valid));
      add("viewer_" + name + "_t", std::to_string(sample_t));
      add("viewer_" + name + "_x", std::to_string(pose.p.x()));
      add("viewer_" + name + "_y", std::to_string(pose.p.y()));
      add("viewer_" + name + "_z", std::to_string(pose.p.z()));
    };
    Pose viewer_vio;
    const bool viewer_vio_valid = vio_healthy(t) && viewer_vio_frame_.initialized() && !viewer_vio_needs_anchor_;
    if (viewer_vio_frame_.initialized()) viewer_vio = viewer_base_pose(viewer_vio_frame_.transform(last_raw_));
    viewer_point("vio", viewer_vio_valid && finite_pose(viewer_vio), last_vio_, viewer_vio);
    viewer_point("gnss", gps_usable(t) && viewer_gnss_finite_, viewer_gnss_t_, viewer_gnss_pose_);
    add("viewer_gnss_reference", "accepted_fix_enu_minus_antenna_lever_then_local_base_transform");
    add("covariance", "conservative_placeholder_not_calibrated_posterior");
    output.status.push_back(status); status_pub_->publish(output);
  }
  void solve() {
    const double t = now().seconds();
    if (pending_solve_.valid() || !vio_healthy(t) || frames_.size() < 2) return;
    if (gps_available_ && !gps_usable(t)) gps_failure("GNSS expired before solve; old residuals removed");
    if (observation_version_ == last_submitted_version_) return;
    bool have_observation = motion_map_yaw_.has_value();
    for (const auto &frame : frames_) have_observation = have_observation || frame.gps.has_value();
    if (!have_observation) return;
    WindowSolution snapshot;
    snapshot.frames = frames_; snapshot.generation = generation_; snapshot.epoch = epoch_;
    const SolveParameters parameters{lever_, get("vio_translation_sigma_per_sqrt_s"),
        get("vio_rotation_sigma_per_sqrt_s"), get("robust_loss_sigma"), get("alignment_heading_sigma_rad"),
        get("solver_time_s"), static_cast<int>(integer("max_iterations")), motion_map_yaw_};
    try {
      pending_solve_ = std::async(std::launch::async, optimize_window, std::move(snapshot), parameters);
      last_submitted_version_ = observation_version_;
    } catch (const std::exception &error) {
      last_reason_ = std::string("Cannot start Ceres snapshot worker: ") + error.what();
    }
  }
  void collect_solve_result(double t) {
    if (!pending_solve_.valid() || pending_solve_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    WindowSolution result;
    try { result = pending_solve_.get(); }
    catch (const std::exception &error) {
      ++discarded_solves_;
      last_reason_ = std::string("Ceres snapshot worker failed: ") + error.what();
      return;
    }
    if (result.generation != generation_ || result.epoch != epoch_ || !vio_healthy(t)) {
      ++discarded_solves_;
      return;
    }
    solve_ms_ = result.milliseconds;
    if (!result.valid || result.frames.empty()) {
      last_reason_ = "Ceres failure; previous global correction retained";
      return;
    }
    const auto &latest = result.frames.back();
    const Pose corrected_map = compose(estimate(latest), inverse(latest.local));
    const Pose live_correction = compose(corrected_map, inverse(map_local_));
    // Keep newer live keyframes and GPS associations. Their initial estimates
    // must follow the map correction while matched states use the solved poses.
    for (auto &frame : frames_) {
      auto found = std::lower_bound(result.frames.begin(), result.frames.end(), frame.t,
          [](const Frame &candidate, double value) { return candidate.t < value; });
      if (found != result.frames.end() && found->t == frame.t) set(frame, estimate(*found));
      else set(frame, compose(live_correction, estimate(frame)));
    }
    const Q heading_correction = (corrected_map.q * map_local_.q.conjugate()).normalized();
    // This transports the retained attitude into the corrected world. It is
    // not a fresh observation, so the helper preserves measurement ages/sigma.
    heading_->applyWorldRotation(heading_correction);
    map_local_ = corrected_map;
    ++solves_;
    nav_msgs::msg::Path updated;
    updated.header.frame_id = frame_id_; updated.header.stamp = now();
    for (const auto &frame : result.frames) {
      geometry_msgs::msg::PoseStamped point; point.header = updated.header;
      point.header.stamp = rclcpp::Time(static_cast<std::int64_t>(frame.t * 1e9)); point.pose = message(estimate(frame));
      updated.poses.push_back(point);
    }
    window_pub_->publish(updated);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Fusion>()); }
  catch (const std::exception &error) { std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}

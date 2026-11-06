#pragma once
#include "runtime.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <nomad_interfaces/msg/planning_cost_layers.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <cstring>

namespace nomad {
  using Status = nomad_interfaces::msg::ModuleStatus;
  using Layers = nomad_interfaces::msg::PlanningCostLayers;
  using Grid = nav_msgs::msg::OccupancyGrid;
  inline int64_t stamp(const std_msgs::msg::Header& h)
  {
    return int64_t(h.stamp.sec) * 1000000000 + h.stamp.nanosec;
  }
  inline void set_stamp(std_msgs::msg::Header& h, int64_t ns, const std::string& frame = "odom")
  {
    h.frame_id = frame;
    h.stamp.sec = ns / 1000000000;
    h.stamp.nanosec = ns % 1000000000;
  }
  inline rclcpp::QoS map_qos()
  {
    return rclcpp::QoS(1).reliable().transient_local();
  }
  inline rclcpp::QoS status_qos()
  {
    return rclcpp::QoS(10).reliable().durability_volatile();
  }
  inline rclcpp::QoS sensor_qos()
  {
    return rclcpp::SensorDataQoS().keep_last(5);
  }
  inline M4 pose_matrix(const Pose& p)
  {
    M4 m = M4::Identity();
    m.block<3, 3>(0, 0) = p.q.toRotationMatrix();
    m.block<3, 1>(0, 3) = p.p;
    return m;
  }
  inline M4 tf_matrix(const geometry_msgs::msg::Transform& t)
  {
    Pose p;
    p.stamp = 1;
    p.p = V3(t.translation.x, t.translation.y, t.translation.z);
    p.q = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    return pose_matrix(pose_from_json(pose_json(p)));
  }
  inline Pose matrix_pose(int64_t ns, const M4& m)
  {
    Pose p;
    p.stamp = ns;
    p.p = m.block<3, 1>(0, 3);
    p.q = Eigen::Quaterniond(m.block<3, 3>(0, 0)).normalized();
    return p;
  }
  inline Json status_json(const Status& m, int64_t wall)
  {
    return {{"stamp_ns", stamp(m.header)}, {"frame", m.header.frame_id}, {"state", m.state}, {"valid_ns", int64_t(m.valid_for.sec) * 1000000000 + m.valid_for.nanosec}, {"wall_ns", wall}};
  }
  inline bool status_ready(const Json& s, int64_t now, int64_t wall)
  {
    if (s.empty())
      return false;
    int64_t ns = s.at("stamp_ns"), valid = s.at("valid_ns"), received = s.at("wall_ns");
    return s.at("frame") == "odom"&&s.at("state") == Status::READY&&valid > 0&&valid <= 1000000000&&now - ns >= -50000000&&now - ns <= valid&&wall - received >= 0&&wall - received <= 1000000000;
  }
  inline M4 calibration_matrix(const Json& j)
  {
    M4 m = M4::Identity();
    for (int r = 0; r < 3; ++r)
    {
      for (int c = 0; c < 3; ++c)
        m(r, c) = j.at("R_camera_lidar")[r][c];
      m(r, 3) = j.at("t_camera_lidar_m")[r];
    }
    return rigid(matrix_json(m));
  }
  inline Points cloud_points(const sensor_msgs::msg::PointCloud2& m, const std::string& frame)
  {
    if (m.header.frame_id != frame||stamp(m.header) <= 0||m.height != 1||!m.width||m.width > 1000000||m.point_step != 12||m.row_step != m.width * 12||m.data.size() != size_t(m.row_step)||m.is_bigendian||m.fields.size() != 3)
      throw std::runtime_error("Invalid external OAK packed XYZ cloud");
    std::array<std::string, 3> names{"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
      if (m.fields[i].name != names[i]||m.fields[i].offset != uint32_t(i * 4)||m.fields[i].datatype != 7||m.fields[i].count != 1)
        throw std::runtime_error("Expected little-endian float32 XYZ at offsets 0,4,8");
    Points p;
    p.reserve(m.width);
    for (size_t i = 0; i < m.width; ++i)
    {
      float xyz[3];
      std::memcpy(xyz, m.data.data() + 12 * i, 12);
      V3 v(xyz[0], xyz[1], xyz[2]);
      if (!v.allFinite()||v.z() <= 0)
        throw std::runtime_error("Invalid external OAK optical coordinates");
      p.push_back(v);
    }
    return p;
  }
  int run_vio(const Arguments&);
  int run_points(const Arguments&);
  int run_slope(const Arguments&);
  int run_mapper(const Arguments&);
}

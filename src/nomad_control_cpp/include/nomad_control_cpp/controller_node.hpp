#pragma once

// Public face of the VESC trial / LPP controller node. The implementation lives in
// src/controller_node.cpp and is built once into nomad_controller_core, which both the
// controller_node executable and the ROS integration tests link.

#include "nomad_control_cpp/srv/commissioning_control.hpp"
#include "nomad_control_cpp/srv/manual_control.hpp"
#include "nomad_control_cpp/srv/set_steering_range.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <chrono>
#include <memory>

namespace nomad {
using Scalar = std_msgs::msg::Float64;
using Manual = nomad_control_cpp::srv::ManualControl;
using Commissioning = nomad_control_cpp::srv::CommissioningControl;
using Trigger = std_srvs::srv::Trigger;
using SteeringRange = nomad_control_cpp::srv::SetSteeringRange;
using VescState = vesc_msgs::msg::VescStateStamped;

inline double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// One output owner arbitrating manual and LPP input. Node name: nomad_vesc_trial.
class ControllerNode : public rclcpp::Node {
public:
    // Throws std::invalid_argument for unsafe or inconsistent parameters, before any output exists.
    static std::shared_ptr<ControllerNode> create(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Call before rclcpp::shutdown on a normal Ctrl+C/SIGTERM so the owned output gets 0A.
    virtual void prepareShutdown() noexcept = 0;

protected:
    explicit ControllerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("nomad_vesc_trial", options) {}
};
} // namespace nomad

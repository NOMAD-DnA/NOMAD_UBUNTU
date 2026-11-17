#pragma once
// Wire stamps (2026-11-17). Stamps and durations that arrive in ROS messages and service requests
// (header.stamp, request.stamp, valid_for) are converted only through stampSeconds().
// - A malformed stamp (sec < 0 or nanosec >= 1e9) becomes nullopt, and the caller refuses the input.
// - rclcpp::Time(stamp) throws on sec < 0. Any exception inside a callback ends the controller
//   (controller_main.cpp catches it, sends the stop output and exits). So wire input must never reach
//   a throwing conversion.
// Kept free of ROS headers so the unit test can use plain structs.
#include <cstdint>
#include <optional>

namespace nomad {

inline std::optional<double> stampSeconds(std::int64_t sec, std::uint64_t nanosec) {
    if (sec < 0 || nanosec >= 1000000000u) return std::nullopt;
    return static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9;
}

// builtin_interfaces Time/Duration, or anything else with sec and nanosec members.
template <class Stamp>
auto stampSeconds(const Stamp& stamp) -> decltype(stamp.sec, stamp.nanosec, std::optional<double>{}) {
    return stampSeconds(static_cast<std::int64_t>(stamp.sec), static_cast<std::uint64_t>(stamp.nanosec));
}

// The older lenient conversion, sec + nanosec * 1e-9. It never throws and never refuses anything.
// It stays where the downstream checks were built around it: the LPP command guard (a malformed STOP still clears
// the live command), VESC telemetry freshness, and the IMU attitude stamp. That way the 11-17 change alters no
// behaviour there (review 11-17).
template <class Stamp>
auto rawStampSeconds(const Stamp& stamp) -> decltype(stamp.sec, stamp.nanosec, double{}) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

} // namespace nomad

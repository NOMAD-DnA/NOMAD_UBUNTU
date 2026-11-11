#pragma once

// Shared safety limits for the controller, raw state machine, planning guard and GUI bridge.
// Change a value here once instead of editing matching literals in several files.
namespace nomad::limits {

// Hard ERPM ceiling for any raw or planning motor command.
inline constexpr double kMotorMaxErpm = 1850.0;

// A command or telemetry stamp may be this far in the future (clock skew allowance).
inline constexpr double kFutureStampToleranceS = 0.05;

// LPP DriveCommand lease: valid_for must be in (0, kPlanningMaxValidForS], and a command
// expires kPlanningReceiveLeaseS after it was received even if valid_for is longer.
inline constexpr double kPlanningMaxValidForS = 0.5;
inline constexpr double kPlanningReceiveLeaseS = 0.35;

// GUI command freshness. A manual timeout above kWifiManualTimeoutThresholdS selects the
// Wi-Fi manual profile, which accepts older MANUAL commands; every other case uses the wired limit.
inline constexpr double kWiredCommandMaxAgeS = 0.30;
inline constexpr double kWifiManualCommandMaxAgeS = 1.5;
inline constexpr double kWifiManualTimeoutThresholdS = 0.35;

// Short MOTOR/STEERING trials keep their watchdog at or below this even on the Wi-Fi profile.
inline constexpr double kShortAxisWatchdogMaxS = 0.35;

// A GUI-supervised autonomy session stops when the operator heartbeat is older than this.
inline constexpr double kOperatorHeartbeatS = 2.0;

inline constexpr bool wifiManualProfile(double manual_timeout_s) {
    return manual_timeout_s > kWifiManualTimeoutThresholdS;
}

inline constexpr double commandMaxAge(bool manual_axis, double manual_timeout_s) {
    return manual_axis && wifiManualProfile(manual_timeout_s) ? kWifiManualCommandMaxAgeS : kWiredCommandMaxAgeS;
}

} // namespace nomad::limits

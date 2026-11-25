#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace nomad_planning_cpp {
// Diagnostic evidence only: the existing status_ready() gate is unchanged.
struct StatusSample {
  bool present = false;
  int state = -1;
  std::int64_t stamp_ns = 0;
  double received_wall = 0, evaluated_ros = 0, evaluated_wall = 0;
  double valid_for = 0, timeout = 0, future_tolerance = 0;
  std::string detail;
};

struct StatusDiagnostic {
  enum Failure : unsigned {
    no_message = 1, receive_time_invalid = 2, receive_timeout = 4,
    stamp_invalid = 8, stamp_in_future = 16, source_timeout = 32,
    state_not_ready = 64, valid_for_invalid = 128, lease_expired = 256
  };
  StatusSample sample;
  unsigned failures = 0;
  bool ready() const { return failures == 0; }
  double source_age() const { return sample.evaluated_ros - sample.stamp_ns * 1e-9; }
  double receive_age() const { return sample.evaluated_wall - sample.received_wall; }
  const char* cause() const {
    // Follow the existing gate's evaluation order when failures overlap.
    if (failures & no_message) return "NO_MESSAGE";
    if (failures & receive_time_invalid) return "RECEIVE_TIME_INVALID";
    if (failures & receive_timeout) return "RECEIVE_TIMEOUT";
    if (failures & stamp_invalid) return "STAMP_INVALID";
    if (failures & stamp_in_future) return "STAMP_IN_FUTURE";
    if (failures & source_timeout) return "SOURCE_TIMEOUT";
    if (failures & state_not_ready) return "STATE_NOT_READY";
    if (failures & valid_for_invalid) return "VALID_FOR_INVALID";
    if (failures & lease_expired) return "LEASE_EXPIRED";
    return "READY";
  }
  const char* category() const {
    const std::string primary = cause();
    if (primary == "NO_MESSAGE") return "MISSING";
    if (primary == "STATE_NOT_READY") return "STATE_NOT_READY";
    return ready() ? "READY" : "TIMING_INVALID";
  }
  std::string failure_names() const {
    std::string names;
    for (unsigned bit = no_message; bit <= lease_expired; bit <<= 1) {
      if (!(failures & bit)) continue;
      auto one = *this; one.failures = bit;
      if (!names.empty()) names += '|';
      names += one.cause();
    }
    return names;
  }
  // Ages, source stamps and free-text detail change continuously. They are
  // evidence, not a reason to print the same failure at command frequency.
  std::string key() const {
    return std::to_string(failures) + ":" + std::to_string(sample.state);
  }
};

inline StatusDiagnostic diagnose_status(StatusSample sample, int ready_state) {
  StatusDiagnostic d{std::move(sample), 0};
  if (!d.sample.present) { d.failures = StatusDiagnostic::no_message; return d; }
  const auto& s = d.sample;
  const double source = s.stamp_ns * 1e-9;
  if (s.received_wall <= 0) d.failures |= StatusDiagnostic::receive_time_invalid;
  if (d.receive_age() > s.timeout) d.failures |= StatusDiagnostic::receive_timeout;
  if (!(source > 0)) d.failures |= StatusDiagnostic::stamp_invalid;
  if (!(source <= s.evaluated_ros + s.future_tolerance)) d.failures |= StatusDiagnostic::stamp_in_future;
  if (!(d.source_age() <= s.timeout)) d.failures |= StatusDiagnostic::source_timeout;
  if (s.state != ready_state) d.failures |= StatusDiagnostic::state_not_ready;
  if (!(s.valid_for > 0)) d.failures |= StatusDiagnostic::valid_for_invalid;
  if (!(d.source_age() <= s.valid_for)) d.failures |= StatusDiagnostic::lease_expired;
  return d;
}

struct StatusFailureEvidence {
  StatusDiagnostic failure;
  bool from_worker = false;
  std::string key() const {
    return std::string(from_worker ? "worker_result:" : "current_input:") + failure.key();
  }
};

inline StatusFailureEvidence status_failure_evidence(
    const StatusDiagnostic& current, const std::optional<StatusDiagnostic>& worker) {
  // A timer may still reject a stored worker result after the input recovers.
  // Keep the original failure instead of falsely attributing READY to failure.
  if (current.ready() && worker) return {*worker, true};
  return {current, false};
}
}  // namespace nomad_planning_cpp

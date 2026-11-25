#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>

namespace nomad_planning_cpp {
// Timer-owned diagnostic state. No actuator permission or navigation state is
// changed here. Keep every reason transition, but not repeated 20 Hz snapshots.
class LppDiagnostics {
 public:
  enum class Event { none, pause, recovered };
  Event update(bool has_goal, bool arrived, bool executable,
               std::uint64_t revision, std::uint64_t epoch,
               const std::string& state, const std::string& reason) {
    if (!has_goal || arrived) {
      previous_.reset();
      return Event::none;
    }
    if (executable) {
      const bool recovering = previous_ && std::get<0>(*previous_) == revision
                              && std::get<1>(*previous_) == epoch;
      previous_.reset();
      return recovering ? Event::recovered : Event::none;
    }
    const Key key{revision, epoch, state, reason};
    if (previous_ && *previous_ == key) return Event::none;
    previous_ = key;
    return Event::pause;
  }
 private:
  using Key = std::tuple<std::uint64_t, std::uint64_t, std::string, std::string>;
  std::optional<Key> previous_;
};
}  // namespace nomad_planning_cpp

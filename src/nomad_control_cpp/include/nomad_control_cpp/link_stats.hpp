#pragma once
// Link and control-loop figures (2026-11-17), one window per second. They appear in the link_trace log line
// and in status_json. Display only: no control decision reads them.
// Why: on 11-16, Wi-Fi gaps between the Jetson planner and this Pi4 controller stopped the car 31 times
// (DriveCommand expiry). Those field logs could not show how long the gaps were.
// - commands: DriveCommands received in the window.
// - max_gap_s: the longest time without a DriveCommand. At window close this includes the interval since the
//   last command, so a gap that spans windows shows as a growing value until the next command arrives.
//   0 before the first command.
// - min_age_s / max_age_s: the stamp age at receipt (this node's ROS clock minus the sender's stamp). It mixes
//   network delay with the clock offset between boards. A negative minimum means the sender's clock is ahead.
// - expiries: LPP input lost while LPP was driving (a stop ramp or pause started for expired or invalid input).
// - max_tick_s: the longest interval between two control ticks (20 ms timer). A large value means this process
//   was not scheduled in time (CPU load), not a network problem.
#include <algorithm>
#include <optional>

namespace nomad {

struct LinkWindow {
    double length_s = 0.0;
    int commands = 0;
    double max_gap_s = 0.0;
    std::optional<double> min_age_s, max_age_s;
    int expiries = 0;
    double max_tick_s = 0.0;
};

class LinkStats {
public:
    void onCommand(double steady, std::optional<double> age_s) {
        start(steady);
        if (last_command_) window_.max_gap_s = std::max(window_.max_gap_s, steady - *last_command_);
        last_command_ = steady;
        ++window_.commands;
        if (age_s) {
            window_.min_age_s = window_.min_age_s ? std::min(*window_.min_age_s, *age_s) : *age_s;
            window_.max_age_s = window_.max_age_s ? std::max(*window_.max_age_s, *age_s) : *age_s;
        }
    }
    void onExpiry() { ++window_.expiries; }
    void onTick(double steady) {
        start(steady);
        if (last_tick_) window_.max_tick_s = std::max(window_.max_tick_s, steady - *last_tick_);
        last_tick_ = steady;
    }
    // Closes the window once it is at least period_s long and returns it; nullopt otherwise.
    std::optional<LinkWindow> roll(double steady, double period_s = 1.0) {
        start(steady);
        if (steady - *window_start_ < period_s) return std::nullopt;
        LinkWindow done = window_;
        done.length_s = steady - *window_start_;
        if (last_command_) done.max_gap_s = std::max(done.max_gap_s, steady - *last_command_);
        window_ = LinkWindow{};
        window_start_ = steady;
        last_ = done;
        return done;
    }
    // The last closed window (status_json), nullopt before the first one.
    const std::optional<LinkWindow>& last() const { return last_; }

private:
    void start(double steady) { if (!window_start_) window_start_ = steady; }
    LinkWindow window_;
    std::optional<double> window_start_, last_command_, last_tick_;
    std::optional<LinkWindow> last_;
};

} // namespace nomad

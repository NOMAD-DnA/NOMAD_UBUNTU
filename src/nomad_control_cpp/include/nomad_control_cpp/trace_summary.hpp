#pragma once
// Summary figures for the restart and stop-ramp log lines, and the repeat rule for [PLANNING_WAIT] (2026-11-19).
// They are display only: no control decision reads them. They are ROS-free so they can be unit tested.
// Why: the team GUI reads only about 1 KiB of this controller's log per second. In the 11-18 corridor run, 25 of
// 62 log updates dropped older lines. The lines needed in the field ([STOP_RAMP] end, link_trace) were among them.
// - One [RESTART_TRACE] line per 20 ms control tick for 3 s (about 150 lines per restart) took 32 % of the log.
// - [PLANNING_WAIT] (about 400 characters, every second while waiting) took 10 %.
// - The per-tick [STOP_RAMP] lines took 7 %.
// The per-tick lines are now off unless trace_every_tick is true. Each restart and each stop ramp ends with one
// summary line, and [PLANNING_WAIT] repeats every 10 s instead of every second.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <optional>
#include <string>

namespace nomad {

namespace trace_detail {
inline std::string fixed(double value, int decimals) {
    char text[48];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    return text;
}
inline std::string fixedOrNa(const std::optional<double>& value, int decimals) {
    return value ? fixed(*value, decimals) : std::string("na");
}
}  // namespace trace_detail

// The first restart_trace_s seconds after a not-driving -> driving change, one onTick per control tick. The trace
// ends early when the drive stops or a stop ramp begins (the ramp has its own [STOP_RAMP] lines).
// - reach_s: the first t at which the measured ERPM reached 90 % of the commanded ERPM (same sign), na if never.
// - erpm_min / erpm_max: the measured ERPM range. A negative minimum on a forward restart means it rolled back.
// - current_max_a: the largest |current_a|; iq_peak_a: the |avg_iq| peak with its sign and its t (iq_peak_t).
//   current_a is the sign-weighted avg_motor_current that under-reads at standstill; iq is the torque current.
//   Both count only drive samples (drive_sample): the VESC sample read before the speed command and the first one
//   after it still average the handbrake hold current (11-18 10 deg slope: 30.4 A at t=0, 29.3 A at t=0.019 after a
//   30 A hold, 14.0 A at t=0.041), which would otherwise hide the drive peak. The hold shows in the start line. na =
//   no drive sample in the window.
// - telemetry_age_max_ms: the oldest telemetry seen at a tick, na if there was none.
class RestartSummary {
public:
    static constexpr double kReachFraction = 0.9;

    // t: seconds since the first speed command. age_ms < 0 = no telemetry received yet.
    void onTick(double t, double cmd_erpm, double measured_erpm, double current_a, double iq_a, double age_ms,
                bool drive_sample) {
        ++ticks_;
        cmd_erpm_ = cmd_erpm;
        erpm_min_ = ticks_ == 1 ? measured_erpm : std::min(erpm_min_, measured_erpm);
        erpm_max_ = ticks_ == 1 ? measured_erpm : std::max(erpm_max_, measured_erpm);
        if (drive_sample) {
            current_max_a_ = std::max(current_max_a_, std::abs(current_a));
            if (!seen_drive_ || std::abs(iq_a) > std::abs(iq_peak_a_)) {
                iq_peak_a_ = iq_a;
                iq_peak_t_ = t;
            }
            seen_drive_ = true;
        }
        if (age_ms >= 0.0) age_max_ms_ = std::max(age_max_ms_.value_or(age_ms), age_ms);
        const double sign = cmd_erpm > 0.0 ? 1.0 : (cmd_erpm < 0.0 ? -1.0 : 0.0);
        if (!reach_s_ && sign != 0.0 && sign * measured_erpm >= kReachFraction * std::abs(cmd_erpm)) reach_s_ = t;
    }
    int ticks() const { return ticks_; }
    const std::optional<double>& reachSeconds() const { return reach_s_; }
    // The fields appended to the "[RESTART_TRACE] end" line.
    std::string fields() const {
        using namespace trace_detail;
        if (ticks_ == 0) return "ticks=0";
        const std::string na("na");
        return "ticks=" + std::to_string(ticks_) + " reach_s=" + fixedOrNa(reach_s_, 2) +
               " cmd_erpm=" + fixed(cmd_erpm_, 0) + " erpm_min=" + fixed(erpm_min_, 0) +
               " erpm_max=" + fixed(erpm_max_, 0) + " current_max_a=" + (seen_drive_ ? fixed(current_max_a_, 2) : na) +
               " iq_peak_a=" + (seen_drive_ ? fixed(iq_peak_a_, 2) : na) +
               " iq_peak_t=" + (seen_drive_ ? fixed(iq_peak_t_, 2) : na) +
               " telemetry_age_max_ms=" + fixedOrNa(age_max_ms_, 0);
    }

private:
    int ticks_ = 0;
    bool seen_drive_ = false;
    double cmd_erpm_ = 0.0, erpm_min_ = 0.0, erpm_max_ = 0.0, current_max_a_ = 0.0, iq_peak_a_ = 0.0, iq_peak_t_ = 0.0;
    std::optional<double> reach_s_, age_max_ms_;
};

// One stop ramp ([STOP_RAMP] begin .. end/cancelled), one onTick per control tick while it runs and onEndTick for
// the tick on which it ended (the raw controller ends it on that tick's sample, before the trace sees it).
// The ramp ends on telemetry samples (raw_trial: 2 of the last 3 at or below end_erpm), so the figures count
// samples, not ticks:
// - wait_s: from the tick the speed command reached its end (command_end_erpm; end_erpm unless
//   planning.stop_ramp_command_erpm is set) to the last tick seen; na if it never did. With a command end below
//   end_erpm the ramp often ends on the measured rule first, so wait_s=na and end_samples=0 are normal then.
// - end_samples / end_above: telemetry samples after the command reached its end, and how many of them were
//   still above end_erpm. Many samples above = the wheel was slow to reach the handbrake speed (or HFI noise).
// - erpm_last: the measured ERPM of the last 5 samples since the ramp began, oldest first, the end tick's included.
//   Two of the last three at or below end_erpm = the ramp ended by the rule (also before the command reached
//   its end, as raw_trial counts from the ramp start); otherwise wait_s 0.60 = the settle timeout, or a stop.
// - iq_peak_a: the |avg_iq| peak with its sign (negative = braking torque).
class StopRampSummary {
public:
    static constexpr std::size_t kLastSamples = 5;

    explicit StopRampSummary(double end_erpm) : StopRampSummary(end_erpm, end_erpm) {}
    // 2026-11-20: command_end_erpm = where the ramp command stops falling (wait_s counts from there); end_erpm = the
    // measured speed that hands over (end_above counts samples above it). The same value unless
    // planning.stop_ramp_command_erpm is set.
    StopRampSummary(double command_end_erpm, double end_erpm) : command_end_erpm_(command_end_erpm), end_erpm_(end_erpm) {}
    // new_sample: a telemetry sample arrived since the previous tick.
    void onTick(double t, double cmd_erpm, double measured_erpm, double iq_a, bool new_sample) {
        if (!at_end_t_ && cmd_erpm <= command_end_erpm_ + 0.5) at_end_t_ = t;
        record(t, measured_erpm, iq_a, new_sample);
    }
    // The tick on which the ramp ended. Its sample is the one that decided a "2 of the last 3" end, and its t is the
    // settle-timeout end. The command is not used: it is already 0 after the stop, and a fault or operator stop
    // before the end speed must still print wait_s=na.
    void onEndTick(double t, double measured_erpm, double iq_a, bool new_sample) {
        record(t, measured_erpm, iq_a, new_sample);
    }
    int endSamples() const { return end_samples_; }
    int endAbove() const { return end_above_; }
    // The fields appended to the "[STOP_RAMP] end" and "[STOP_RAMP] cancelled" lines.
    std::string fields() const {
        using namespace trace_detail;
        std::string last;
        for (double erpm : last_) last += (last.empty() ? "" : "/") + fixed(erpm, 0);
        // A string, not a local optional: GCC's -Wmaybe-uninitialized misreads a local optional<double> here.
        const std::string wait = at_end_t_ ? fixed(std::max(0.0, last_t_ - *at_end_t_), 2) : std::string("na");
        return "wait_s=" + wait + " end_samples=" + std::to_string(end_samples_) +
               " end_above=" + std::to_string(end_above_) + " erpm_last=" + (last.empty() ? "na" : last) +
               " iq_peak_a=" + (seen_iq_ ? fixed(iq_peak_a_, 2) : std::string("na"));
    }

private:
    void record(double t, double measured_erpm, double iq_a, bool new_sample) {
        last_t_ = t;
        if (!seen_iq_ || std::abs(iq_a) > std::abs(iq_peak_a_)) {
            iq_peak_a_ = iq_a;
            seen_iq_ = true;
        }
        if (!new_sample) return;
        if (at_end_t_) {
            ++end_samples_;
            if (measured_erpm > end_erpm_) ++end_above_;
        }
        last_.push_back(measured_erpm);
        while (last_.size() > kLastSamples) last_.pop_front();
    }

    double command_end_erpm_, end_erpm_, last_t_ = 0.0, iq_peak_a_ = 0.0;
    bool seen_iq_ = false;
    std::optional<double> at_end_t_;
    int end_samples_ = 0, end_above_ = 0;
    std::deque<double> last_;
};

// When a repeating log line is due. It is printed:
// - when its key (what it says, without ages or currents) changes, but at most once per min_interval_s; a change
//   inside that interval is printed on a later call if the key still differs then;
// - when a new episode starts: no call for more than episode_gap_s (a pause reported once, minutes after the
//   last one with the same reason, is still printed);
// - otherwise every repeat_s while the calls go on.
class RepeatGate {
public:
    RepeatGate(double min_interval_s = 1.0, double repeat_s = 10.0, double episode_gap_s = 2.0)
        : min_interval_s_(min_interval_s), repeat_s_(repeat_s), episode_gap_s_(episode_gap_s) {}
    bool due(const std::string& key, double steady) {
        const bool new_episode = !last_call_ || steady - *last_call_ > episode_gap_s_;
        last_call_ = steady;
        if (last_print_ && steady - *last_print_ < min_interval_s_) return false;
        if (last_print_ && !new_episode && key == key_ && steady - *last_print_ < repeat_s_) return false;
        key_ = key;
        last_print_ = steady;
        return true;
    }
    // Hold the line back for quiet_s: another line just said the gist of it. The next call at least quiet_s from
    // now prints whatever it says then (the key is cleared), so a wait that goes on still gets its detail line.
    void defer(double steady, double quiet_s) {
        last_call_ = steady;
        key_.clear();
        last_print_ = steady + std::max(0.0, quiet_s - min_interval_s_);
    }

private:
    double min_interval_s_, repeat_s_, episode_gap_s_;
    std::string key_;
    std::optional<double> last_call_, last_print_;
};

// A line printed at most once per period_s on the steady clock; the first call prints (2026-11-19: control_trace and
// vesc_trace, which used RCLCPP_INFO_THROTTLE on the ROS clock and went quiet when the wall clock was set back).
class PeriodGate {
public:
    explicit PeriodGate(double period_s) : period_s_(period_s) {}
    bool due(double steady) {
        if (last_ && steady - *last_ < period_s_) return false;
        last_ = steady;
        return true;
    }

private:
    double period_s_;
    std::optional<double> last_;
};

}  // namespace nomad

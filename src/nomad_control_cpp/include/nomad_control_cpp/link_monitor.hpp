#pragma once
// The LPP link in one place (2026-11-19 refactor, same behaviour as the 11-17 code in controller_node.cpp):
// - the 1 s link/loop window (link_stats.hpp: commands, gaps, stamp ages, expiries, control-tick intervals),
// - the link-loss resume wait: after a stop caused by lost or invalid LPP input (Wi-Fi), the drive resumes only after
//   planning.link_resume_stable_s of continuous commands. On 11-16 the car stopped and restarted 0.1-0.44 s later 6
//   times. It counts arrivals, not the lease: one late command with a long valid_for must not pass the wait on its
//   own (review 11-17). A gap over kGapS (3 periods at 20 Hz) restarts the count. A planner STOP, an operator or
//   global STOP, a RESET, a resume, a start or a cancelled stop ramp is not a link loss and clears the wait.
// - when the window is worth a link_trace line, and the line's fields.
// ROS-free so the wait can be unit tested (before, only the ROS tests covered it).
#include "nomad_control_cpp/link_stats.hpp"

#include <cstdio>
#include <optional>
#include <string>

namespace nomad {

class LinkMonitor {
public:
    static constexpr double kGapS = 0.15;           // longest gap between commands that still counts as arriving
    static constexpr double kTraceRepeatS = 10.0;   // link_trace without news: once per this many seconds
    static constexpr double kTraceTickS = 0.04;     // news: a control tick later than this
    static constexpr double kTraceGapS = 0.2;       // news: a gap longer than this while commands were arriving

    // planning.link_resume_stable_s (the node checks 0..2 s). 0 = resume at once, as before 11-17.
    void setResumeStableSeconds(double seconds) { resume_stable_s_ = seconds; }
    double resumeStableSeconds() const { return resume_stable_s_; }

    // Every DriveCommand received (also STOP and invalid ones): the window figures. age_s = this node's ROS time minus
    // the sender's stamp; nullopt for a malformed stamp.
    void onCommand(double steady, std::optional<double> age_s) { stats_.onCommand(steady, age_s); }
    // A command the planning guard accepted: the arrival that counts for the resume wait.
    void onAccepted(double steady) { last_accept_ = steady; }
    // LPP input expired or became invalid while LPP was driving: an expiry, and the next resume waits.
    void onLost() {
        stats_.onExpiry();
        pause_ = true;
    }
    // Not a link loss (see above): no wait on the next resume.
    void clearPause() { pause_ = false; }

    // Every control tick, first: the control-loop interval for the window.
    void onControlTick(double steady) { stats_.onTick(steady); }
    // Every planning tick, before its decisions: has_target = the planning guard holds a live target.
    void updateFlow(double steady, bool has_target) {
        const bool flowing = has_target && last_accept_ && steady - *last_accept_ <= kGapS;
        if (!flowing) fresh_since_.reset();
        else if (!fresh_since_) fresh_since_ = steady;
    }
    // True while a resume after a link loss must still wait for continuous commands.
    bool resumeWaiting(double steady) const {
        return pause_ && resume_stable_s_ > 0 && (!fresh_since_ || steady - *fresh_since_ < resume_stable_s_);
    }
    // The current or last LPP pause came from lost or invalid input (status_json planning_link_pause).
    bool paused() const { return pause_; }

    // Closes the 1 s window (LinkStats::roll) and returns it; nullopt while it is still open.
    std::optional<LinkWindow> roll(double steady) { return stats_.roll(steady); }
    // The last closed window (status_json link_* keys), nullopt before the first one.
    const std::optional<LinkWindow>& last() const { return stats_.last(); }

    // A closed window is news when it had an expiry, a control tick over 40 ms late, or a gap over 0.2 s while
    // commands were arriving. A window with no command at all is not news: either the planner is off, or an outage is
    // in progress (its start shows as expired=1, its full length when the next command arrives).
    static bool news(const LinkWindow& w) {
        return w.expiries > 0 || w.max_tick_s > kTraceTickS || (w.commands > 0 && w.max_gap_s > kTraceGapS);
    }
    // Whether to print this closed window as a link_trace line: news, or kTraceRepeatS since the last line. The team
    // GUI reads only about 1 KiB of this log per second, so quiet windows are not printed every second.
    bool traceDue(const LinkWindow& w, double steady) {
        if (!news(w) && steady - trace_at_ < kTraceRepeatS) return false;
        trace_at_ = steady;
        return true;
    }
    // The link_trace fields after "link_trace " (vesc_status.py link_trace_from_line). na = no command or no valid
    // stamp in the window.
    static std::string traceFields(const LinkWindow& w) {
        char text[192];
        std::snprintf(text, sizeof(text), "n=%d gap_ms=%.0f age_min_ms=%s age_max_ms=%s expired=%d tick_max_ms=%.0f",
                      w.commands, 1000.0 * w.max_gap_s, ms(w.min_age_s).c_str(), ms(w.max_age_s).c_str(), w.expiries,
                      1000.0 * w.max_tick_s);
        return text;
    }

private:
    static std::string ms(const std::optional<double>& seconds) {
        if (!seconds) return "na";
        char text[32];
        std::snprintf(text, sizeof(text), "%.0f", 1000.0 * *seconds);
        return text;
    }

    LinkStats stats_;
    double resume_stable_s_ = 0.3;
    bool pause_ = false;
    std::optional<double> fresh_since_, last_accept_;
    double trace_at_ = -1e9;
};

}  // namespace nomad

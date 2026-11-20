#pragma once
// The LPP link in one place (2026-11-19 refactor, same behaviour as the 11-17 code in controller_node.cpp):
// - the 1 s link/loop window (link_stats.hpp: commands, gaps, stamp ages, expiries, control-tick intervals),
// - the link-loss resume wait: after a stop caused by lost or invalid LPP input (Wi-Fi), the drive resumes only after
//   planning.link_resume_stable_s of continuous commands. On 11-16 the car stopped and restarted 0.1-0.44 s later 6
//   times. It counts arrivals, not the lease: one late command with a long valid_for must not pass the wait on its
//   own (review 11-17). A gap over kGapS (3 periods at 20 Hz) restarts the count. An operator or global STOP, a
//   RESET, a resume, a start or a cancelled stop ramp is not a link loss and clears the wait. A planner STOP no
//   longer does (2026-11-20, onPlannerStop).
// - when the window is worth a link_trace line, and the line's fields.
// - 2026-11-20: two lines per outage for the field log, since the 1 s link_trace could not tell late delivery from a
//   planner skip on 11-19. [LPP_LOST] when the input is lost (lostFields: how old the last accepted command is), and
//   [LPP_BACK] at the first accepted command after it (onAccepted: what arrived meanwhile). Late commands only arrive
//   after the expiry (a newer command is never older than the one that expired), so only the second line shows them.
// ROS-free so the wait can be unit tested (before, only the ROS tests covered it).
#include "nomad_control_cpp/link_stats.hpp"

#include <algorithm>
#include <cmath>
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
    void onCommand(double steady, std::optional<double> age_s) {
        stats_.onCommand(steady, age_s);
        if (outage_ && !outage_->first_rx) outage_->first_rx = steady;  // traffic is back ([LPP_BACK] first_ms)
    }
    // A command the planning guard accepted: the arrival that counts for the resume wait. age_s (receipt age, nullopt
    // for a malformed stamp) and valid_for_s are kept for the [LPP_LOST] line. The first one after onLost() returns the
    // [LPP_BACK] fields and ends that outage record; nullopt otherwise.
    std::optional<std::string> onAccepted(double steady, std::optional<double> age_s = std::nullopt,
                                          double valid_for_s = NAN) {
        std::optional<std::string> back;
        if (outage_) {
            const auto& o = *outage_;
            const auto since_accept = [&](const std::optional<double>& t) {
                return o.last_accept && t ? std::optional<double>{*t - *o.last_accept} : std::nullopt;
            };
            char text[224];
            std::snprintf(text, sizeof(text),
                          "gap_ms=%s first_ms=%s refused=%d refused_age_max_ms=%s stops=%d stop_age_max_ms=%s "
                          "stop_valid_ms=%s age_ms=%s",
                          ms(since_accept(steady)).c_str(), ms(since_accept(o.first_rx)).c_str(), o.refused, ms(o.refused_age_max).c_str(), o.stops, ms(o.stop_age_max).c_str(),
                          ms(finite(o.stop_valid_for)).c_str(), ms(age_s).c_str());
            back = text;
            outage_.reset();
        }
        last_accept_ = steady;
        accepted_age_ = age_s;
        accepted_valid_for_ = valid_for_s;
        return back;
    }
    // A moving command the planning guard refused (older than its valid_for, invalid, out of order). Counted for
    // [LPP_BACK] while an outage record is open. The subscription keeps one message (QoS depth 1), so a burst that
    // arrives faster than the callback runs is counted only in part: a lower bound.
    void onRefused(std::optional<double> age_s) {
        if (!outage_) return;
        ++outage_->refused;
        outage_->refused_age_max = maxAge(outage_->refused_age_max, age_s);
    }
    // LPP input expired or became invalid while LPP was driving: an expiry, and the next resume waits. Opens the outage
    // record for [LPP_BACK] (a second loss before any accepted command keeps the first record).
    void onLost() {
        stats_.onExpiry();
        pause_ = true;
        if (outage_) return;
        Outage o;  // not an aggregate initializer list: GCC 11 (Pi4) warns on missing fields
        o.last_accept = last_accept_;
        outage_ = o;
    }
    // Not a link loss (see above): no wait on the next resume.
    void clearPause() { pause_ = false; }
    // An operator RESET ends the session's outage record, so its STOPs and gap are not reported at the next run's
    // first command (review 11-20). The wait itself is cleared by clearPause().
    void endOutage() { outage_.reset(); }
    // An LPP STOP. 2026-11-20: it no longer clears the link-loss wait. During a Wi-Fi outage the planner itself sends
    // STOP (VEHICLE_STATE_STALE, or a wait). Those STOPs arrived in the burst when the link came back and cleared the
    // wait: on 11-19, 6 of 8 expiry stops restarted 16-101 ms after the commands came back, and 3 of them stopped
    // again within 0.6 s. Judging the STOP's own age does not help, because the newest STOP of the burst (and the
    // STALE ones sent just after the link returns) arrive on time (review 11-20). Cost: after a link loss followed by a
    // deliberate planner STOP, the next start also waits link_resume_stable_s of continuous commands. Recorded for
    // [LPP_BACK] (count, oldest age, the planner's STOP valid_for, not yet known on 11-20) while an outage is open.
    void onPlannerStop(std::optional<double> age_s, double valid_for_s) {
        if (!outage_) return;
        ++outage_->stops;
        outage_->stop_age_max = maxAge(outage_->stop_age_max, age_s);
        outage_->stop_valid_for = valid_for_s;
    }
    // The [LPP_LOST] fields: ms since the last accepted command, and that command's receipt age and valid_for. na =
    // none or unknown.
    std::string lostFields(double steady) const {
        char text[128];
        std::snprintf(text, sizeof(text), "since_ms=%s age_ms=%s valid_ms=%s",
                      ms(last_accept_ ? std::optional<double>{steady - *last_accept_} : std::nullopt).c_str(),
                      ms(accepted_age_).c_str(), ms(finite(accepted_valid_for_)).c_str());
        return text;
    }
    // The receipt age of the last accepted command in ms ("na"), for the [STOP_RAMP] cancelled line.
    std::string acceptedAgeMs() const { return ms(accepted_age_); }

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
    static std::optional<double> finite(double seconds) {
        return std::isfinite(seconds) ? std::optional<double>{seconds} : std::nullopt;
    }
    static std::optional<double> maxAge(const std::optional<double>& so_far, const std::optional<double>& age_s) {
        if (!age_s || !std::isfinite(*age_s)) return so_far;
        return so_far ? std::max(*so_far, *age_s) : *age_s;
    }

    // From onLost() to the next accepted command: what arrived meanwhile ([LPP_BACK]).
    struct Outage {
        std::optional<double> last_accept;      // steady time of the last accepted command before the loss
        std::optional<double> first_rx;         // steady time of the first DriveCommand of any kind after the loss
        int refused = 0, stops = 0;
        std::optional<double> refused_age_max, stop_age_max;
        double stop_valid_for = NAN;            // valid_for of the last planner STOP
    };

    LinkStats stats_;
    double resume_stable_s_ = 0.3;
    bool pause_ = false;
    std::optional<double> fresh_since_, last_accept_;
    std::optional<double> accepted_age_;
    double accepted_valid_for_ = NAN;
    std::optional<Outage> outage_;
    double trace_at_ = -1e9;
};

}  // namespace nomad

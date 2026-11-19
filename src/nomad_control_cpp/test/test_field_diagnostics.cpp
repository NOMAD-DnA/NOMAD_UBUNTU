// 2026-11-17 field diagnostics: wire stamps, the link/loop window and the long-hold warning (ROS-free).
// 2026-11-19: the restart / stop-ramp summary lines and the [PLANNING_WAIT] repeat rule (trace_summary.hpp), and the
// link-loss resume wait and link_trace rule (link_monitor.hpp), before only covered by the ROS tests.
#include "nomad_control_cpp/link_monitor.hpp"
#include "nomad_control_cpp/link_stats.hpp"
#include "nomad_control_cpp/stop_output.hpp"
#include "nomad_control_cpp/trace_summary.hpp"
#include "nomad_control_cpp/wire_stamp.hpp"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
using namespace std;
using namespace nomad;
static int checks = 0;
static void expect(bool v, const char* why) { ++checks; if (!v) throw runtime_error(why); }
static bool near(double a, double b, double tol = 1e-9) { return abs(a - b) <= tol; }
struct FakeStamp { int32_t sec; uint32_t nanosec; };

static void wireStamps() {
    expect(near(*stampSeconds(FakeStamp{12, 500000000u}), 12.5), "valid stamp converts");
    expect(near(*stampSeconds(FakeStamp{0, 0u}), 0.0), "zero stamp converts (callers refuse it as too old)");
    expect(!stampSeconds(FakeStamp{-1, 0u}), "negative seconds are malformed, not an exception");
    expect(!stampSeconds(FakeStamp{1, 1000000000u}), "nanosec >= 1e9 is malformed");
    expect(near(*stampSeconds(FakeStamp{1, 999999999u}), 1.999999999), "largest nanosec is valid");
}

static void linkWindow() {
    LinkStats stats;
    expect(!stats.last() && !stats.roll(0.0), "no window before one second");
    stats.onTick(0.00); stats.onTick(0.02); stats.onTick(0.07); stats.onTick(0.09);
    stats.onCommand(0.10, 0.004); stats.onCommand(0.15, 0.012); stats.onCommand(0.45, -0.002);
    stats.onExpiry();
    expect(!stats.roll(0.99), "window stays open below one second");
    auto w = stats.roll(1.00);
    expect(w && near(w->length_s, 1.0) && w->commands == 3 && w->expiries == 1, "window counts commands and expiries");
    expect(near(w->max_tick_s, 0.05), "longest tick interval");
    // Receipt gaps were 0.05 and 0.30 s; the 0.55 s still open at the close is the longest.
    expect(near(w->max_gap_s, 0.55), "an open gap at the close counts");
    expect(near(*w->min_age_s, -0.002) && near(*w->max_age_s, 0.012), "stamp age min (negative = sender ahead) and max");
    expect(stats.last() && stats.last()->commands == 3, "last() keeps the closed window");
    // A 2.6 s outage that spans windows: growing open gap, then the full gap at the next command.
    stats.onTick(1.5);
    auto quiet = stats.roll(2.0);
    expect(quiet && quiet->commands == 0 && near(quiet->max_gap_s, 1.55) && !quiet->min_age_s,
           "window without commands shows the time since the last one and no age");
    stats.onCommand(3.05, nullopt);
    auto back = stats.roll(3.1);
    expect(back && back->commands == 1 && near(back->max_gap_s, 2.6) && !back->max_age_s,
           "the next command reports the whole gap; a malformed stamp adds no age");
    LinkStats manual;  // no planner: no command gap at all
    manual.onTick(0.0); manual.onTick(0.02);
    auto m = manual.roll(1.0);
    expect(m && m->commands == 0 && m->max_gap_s == 0.0, "no commands ever: gap stays 0");
}

static void longHoldWarning() {
    expect(near(holdCopperLossW(30.0), 15.12, 1e-6) && near(holdCopperLossW(12.5), 2.625, 1e-6) &&
           holdCopperLossW(0.0) == 0.0, "copper loss estimate 1.5 R I^2");
    expect(longHold(10.0, 30.0) && !longHold(9.9, 30.0) && !longHold(60.0, 24.9), "long hold = >= 25 A for >= 10 s");
    LongHoldWarning warn;
    int lines = 0;
    for (double t = 0.0; t <= 75.0; t += 0.1) lines += warn.due(t, 30.0);
    expect(lines == 3, "30 A for 75 s: warnings at 10, 40 and 70 s");
    expect(!warn.due(0.0, 0.0), "hold ended");
    lines = 0;
    for (double t = 0.0; t <= 12.0; t += 0.1) lines += warn.due(t, 30.0);
    expect(lines == 1, "a new hold warns again at 10 s");
    lines = 0;
    for (double t = 0.0; t <= 15.0; t += 0.1) lines += warn.due(t, 12.5);  // flat load-sized hold
    expect(lines == 0, "no warning below 25 A");
    LongHoldWarning chained;  // a new hold right after another, with no 0 sample in between
    lines = 0;
    for (double t = 0.0; t <= 11.0; t += 0.1) lines += chained.due(t, 30.0);
    for (double t = 0.1; t <= 11.0; t += 0.1) lines += chained.due(t, 30.0);
    expect(lines == 2, "a hold that restarts its clock is a new hold");
}

static bool has(const string& text, const string& part) { return text.find(part) != string::npos; }

static void restartSummary() {
    RestartSummary none;
    expect(none.fields() == "ticks=0", "no tick: nothing but the count");
    // A hill start: the wheel rolls back, then reaches 1000 ERPM. All samples count as drive samples here.
    RestartSummary hill;
    hill.onTick(0.00, 1000, 0, 0.5, 24.0, -1, true);    // no telemetry yet
    hill.onTick(0.02, 1000, -40, 3.0, 31.5, 8, true);
    hill.onTick(0.04, 1000, 300, 12.0, -36.1, 12, true);
    hill.onTick(0.06, 1000, 899, 9.0, 20.0, 6, true);   // 89.9 %: not reached
    hill.onTick(0.08, 1000, 905, 4.0, 8.0, 7, true);    // first at or above 90 %
    hill.onTick(0.10, 975, 1275, 2.0, 5.0, 4, true);
    const string f = hill.fields();
    expect(hill.ticks() == 6 && hill.reachSeconds() && near(*hill.reachSeconds(), 0.08), "reach = first tick at 90 %");
    expect(has(f, "ticks=6 reach_s=0.08 cmd_erpm=975 erpm_min=-40 erpm_max=1275"),
           "summary: ticks, reach, last command, measured range (negative = rolled back)");
    expect(has(f, "current_max_a=12.00 iq_peak_a=-36.10 iq_peak_t=0.04 telemetry_age_max_ms=12"),
           "summary: |current| max, signed iq peak and its time, oldest telemetry");
    // The 11-18 10 deg slope restart after a 30 A hold: the first two samples still carry the hold current.
    RestartSummary slope;
    slope.onTick(0.000, 1000, 0, 21.0, 30.35, 3, false);
    slope.onTick(0.019, 1000, 0, 20.0, 29.29, 11, false);
    slope.onTick(0.041, 1000, 60, 9.0, 13.98, 7, true);
    slope.onTick(1.000, 1000, 950, 18.5, 24.40, 6, true);
    expect(has(slope.fields(), "erpm_min=0 erpm_max=950 current_max_a=18.50 iq_peak_a=24.40 iq_peak_t=1.00"),
           "hold samples stay out of the current/iq peaks; the drive peak shows");
    RestartSummary hold_only;  // stopped before a drive sample arrived
    hold_only.onTick(0.0, 1000, 0, 21.0, 30.0, 3, false);
    expect(has(hold_only.fields(), "current_max_a=na iq_peak_a=na iq_peak_t=na"), "no drive sample: peaks are na");
    RestartSummary reverse;  // operator reverse: the sign follows the command
    reverse.onTick(0.0, -800, 100, 1, 1, 5, true);
    reverse.onTick(0.1, -800, -700, 1, 1, 5, true);
    reverse.onTick(0.2, -800, -760, 1, 1, 5, true);
    expect(reverse.reachSeconds() && near(*reverse.reachSeconds(), 0.2), "reverse restart reaches at -720 or below");
    RestartSummary slow;  // never reached, no telemetry at all
    slow.onTick(0.0, 1000, 0, 0, 0, -1, true);
    slow.onTick(0.1, 1000, 500, 0, 0, -1, true);
    expect(has(slow.fields(), "reach_s=na") && has(slow.fields(), "telemetry_age_max_ms=na"), "na when never reached / no telemetry");
    RestartSummary zero;  // a zero command never counts as reached
    zero.onTick(0.0, 0, 0, 0, 0, 1, true);
    expect(!zero.reachSeconds(), "zero command: no reach");
}

static void stopRampSummary() {
    StopRampSummary none(200);
    expect(none.fields() == "wait_s=na end_samples=0 end_above=0 erpm_last=na iq_peak_a=na", "no tick: all na");
    // The 11-18 corridor shape, in the node's order: the command reaches 200 at 0.30 s, the HFI readings straddle
    // 200, and the raw controller ends the ramp on the 99 (39 and 99: 2 of the last 3), which onEndTick records.
    StopRampSummary ramp(200);
    ramp.onTick(0.02, 945, 967, 6.9, true);
    ramp.onTick(0.04, 888, 986, 5.0, false);           // no new sample: not counted
    ramp.onTick(0.30, 200, 400, -8.2, true);           // the command reached the end speed
    const double readings[] = {245, 226, 39, 258};
    double t = 0.32;
    for (double erpm : readings) { ramp.onTick(t, 200, erpm, 2.0, true); t += 0.02; }
    ramp.onEndTick(0.40, 99, 2.0, true);
    const string f = ramp.fields();
    expect(ramp.endSamples() == 6 && ramp.endAbove() == 4, "samples after the command reached the end, 4 above 200");
    expect(has(f, "wait_s=0.10 end_samples=6 end_above=4 erpm_last=245/226/39/258/99 iq_peak_a=-8.20"),
           "summary: wait after the end speed, counts with the deciding sample, last 5 oldest first, signed iq peak");
    StopRampSummary settle(200);  // the 0.60 s settle timeout: the end tick's time is the timeout
    settle.onTick(0.30, 200, 350, 3.0, true);
    for (int i = 1; i < 30; ++i) settle.onTick(0.30 + 0.02 * i, 200, 350, 3.0, true);
    settle.onEndTick(0.90, 350, 3.0, true);
    expect(has(settle.fields(), "wait_s=0.60 end_samples=31 end_above=31"), "a settle-timeout end shows wait_s 0.60, all above");
    StopRampSummary fault(200);  // a fault or operator stop before the end speed: the end tick does not start a wait
    fault.onTick(0.02, 900, 1000, 3.0, true);
    fault.onEndTick(0.04, 990, 3.0, true);
    expect(has(fault.fields(), "wait_s=na end_samples=0 end_above=0 erpm_last=1000/990"),
           "stopped before the end speed: no wait, the readings are still listed");
    StopRampSummary low_start(200);  // a ramp that began at or below the end speed waits from its first tick
    low_start.onTick(0.0, 150, 180, 1.0, true);
    low_start.onTick(0.1, 150, 170, 1.0, true);
    expect(has(low_start.fields(), "wait_s=0.10 end_samples=2 end_above=0"), "started below the end speed");
}

static void repeatGate() {
    RepeatGate gate;  // 1 s min, 10 s repeat, 2 s episode gap
    int lines = 0;
    for (double t = 0.0; t <= 25.0; t += 0.02) lines += gate.due("LPP pause", t);  // a planner STOP every tick
    expect(lines == 3, "one wait reported each 50 Hz: at 0, 10 and 20 s (was 26 lines, once a second)");
    expect(gate.due("LPP pause", 30.0), "a new wait after a quiet gap is reported even with the same text");
    expect(!gate.due("telemetry unavailable", 30.5), "a change within 1 s of the last line waits");
    expect(gate.due("telemetry unavailable", 31.0), "the change is reported once 1 s has passed");
    bool quiet = true;  // the wait goes on: called every tick with the same text
    for (double t = 31.02; t < 40.99; t += 0.02) quiet = !gate.due("telemetry unavailable", t) && quiet;
    expect(quiet, "same text: quiet until 10 s");
    expect(gate.due("telemetry unavailable", 41.0), "same text: again after 10 s");
    RepeatGate flip;  // two texts swapping every tick: still at most one line a second
    lines = 0;
    for (int i = 0; i <= 250; ++i) lines += flip.due(i % 2 ? "a" : "b", i * 0.02);
    expect(lines <= 6 && lines >= 5, "flipping texts: at most one line per second");
    RepeatGate once;  // a pause reported once (an expiry): every one is printed when they are > 2 s apart
    expect(once.due("expired", 0.0) && once.due("expired", 3.0) && once.due("expired", 5.5),
           "separate pauses with the same reason are each reported");
    RepeatGate held;  // a [STOP_RAMP] end line said the gist: the wait line waits 2 s, then prints its details
    held.defer(0.0, 2.0);
    bool quiet_held = true;
    for (double t = 0.02; t < 1.99; t += 0.02) quiet_held = !held.due(t < 1.0 ? "LPP pause" : "link recovering", t) && quiet_held;
    expect(quiet_held, "held back: no wait line for 2 s, whatever it says");
    expect(held.due("link recovering", 2.0), "held back: the wait that goes on prints at 2 s");
    expect(!held.due("link recovering", 2.5), "held back: then the usual rules");
    RepeatGate held_same;  // the same text as before the hold is not hidden for 10 s
    expect(held_same.due("LPP pause", 0.0), "first wait line");
    held_same.defer(5.0, 2.0);
    expect(!held_same.due("LPP pause", 6.0) && held_same.due("LPP pause", 7.0), "held back: same text still prints at 2 s");
}

static void periodGate() {
    PeriodGate every(1.0);
    int lines = 0;
    for (double t = 0.0; t < 5.0; t += 0.1) lines += every.due(t);  // status every 0.1 s
    expect(lines == 5, "status every 0.1 s: one line a second, the first at once");
    PeriodGate late(1.0);
    expect(late.due(100.0) && !late.due(100.99) && late.due(101.0), "at most once per second on the steady clock");
}

static void linkResumeWait() {
    LinkMonitor link;
    expect(near(link.resumeStableSeconds(), 0.3) && !link.paused() && !link.resumeWaiting(0.0), "default 0.3 s, no wait");
    // Commands at 20 Hz, then a Wi-Fi loss while driving.
    double t = 0.0;
    for (; t < 1.0; t += 0.05) { link.onCommand(t, 0.01); link.onAccepted(t); link.updateFlow(t, true); }
    link.onLost();
    link.updateFlow(t, false);  // as in the node: the loss came with no live target, so the count restarts
    expect(link.paused() && link.resumeWaiting(t), "a loss sets the wait");
    // One late command after 1.2 s: it alone does not end the wait (arrivals count, not its lease).
    t = 2.2;
    link.onCommand(t, 0.02); link.onAccepted(t);
    bool waited = true;
    for (double u = t; u < t + 0.45; u += 0.02) { link.updateFlow(u, true); waited = link.resumeWaiting(u) && waited; }
    expect(waited, "one late command with a long lease keeps waiting (its arrival is over 0.15 s old)");
    // A steady stream: the wait ends after 0.3 s of it, not before.
    t = 3.0;
    double first_free = -1.0;
    for (double u = t; u < t + 0.6; u += 0.05) {
        link.onCommand(u, 0.01); link.onAccepted(u);
        for (double v = u; v < u + 0.049; v += 0.02) {
            link.updateFlow(v, true);
            if (first_free < 0 && !link.resumeWaiting(v)) first_free = v;
        }
    }
    expect(first_free >= t + 0.3 - 1e-9 && first_free < t + 0.35, "0.3 s of continuous commands ends the wait");
    // A gap over 0.15 s restarts the count; no live target restarts it too.
    LinkMonitor gap;
    gap.onLost();
    gap.onAccepted(0.0); gap.updateFlow(0.0, true);
    gap.updateFlow(0.2, true);               // last arrival 0.2 s ago: not flowing
    gap.onAccepted(0.21); gap.updateFlow(0.21, true);
    expect(gap.resumeWaiting(0.21) && gap.resumeWaiting(0.5), "a gap over 0.15 s restarts the count");
    gap.onAccepted(0.55); gap.updateFlow(0.55, false);  // the guard has no live target (e.g. invalid)
    expect(gap.resumeWaiting(0.55), "no live target restarts the count");
    // Refused commands (received, not accepted by the guard: stale, out of order) do not count as arriving.
    LinkMonitor refused;
    refused.onLost();
    bool refused_waits = true;
    for (double u = 0.0; u < 0.6; u += 0.05) {
        refused.onCommand(u, 0.5);  // only the window figures
        refused.updateFlow(u, true);
        refused_waits = refused.resumeWaiting(u) && refused_waits;
    }
    expect(refused_waits, "received but refused commands do not end the wait");
    // Not a link loss: no wait. 0 s: no wait even after a loss (before 11-17).
    LinkMonitor stop;
    stop.onLost(); stop.clearPause();
    expect(!stop.paused() && !stop.resumeWaiting(0.0), "a planner/operator STOP, RESET or resume clears the wait");
    LinkMonitor off;
    off.setResumeStableSeconds(0.0); off.onLost();
    expect(off.paused() && !off.resumeWaiting(0.0), "link_resume_stable_s 0 never waits");
    // Each loss is an expiry in the 1 s window.
    LinkMonitor count;
    count.onControlTick(0.0); count.onLost(); count.onLost(); count.onControlTick(0.02);
    const auto w = count.roll(1.0);
    expect(w && w->expiries == 2 && near(w->max_tick_s, 0.02) && count.last(), "losses count as expiries");
}

static void linkTraceRule() {
    LinkWindow quiet; quiet.length_s = 1.0; quiet.commands = 20; quiet.max_gap_s = 0.06; quiet.min_age_s = 0.003;
    quiet.max_age_s = 0.006; quiet.max_tick_s = 0.022;
    LinkWindow expired = quiet; expired.expiries = 1;
    LinkWindow late = quiet; late.max_tick_s = 0.041;
    LinkWindow gap = quiet; gap.max_gap_s = 0.21;
    LinkWindow outage; outage.length_s = 1.0; outage.max_gap_s = 1.5;  // no command: not news
    expect(!LinkMonitor::news(quiet) && LinkMonitor::news(expired) && LinkMonitor::news(late) && LinkMonitor::news(gap) &&
           !LinkMonitor::news(outage), "news = expiry, a tick over 40 ms, or a gap over 0.2 s while commands arrived");
    LinkMonitor link;
    expect(link.traceDue(quiet, 0.0), "the first window is printed");
    expect(!link.traceDue(quiet, 1.0) && !link.traceDue(outage, 9.9) && link.traceDue(quiet, 10.0),
           "quiet windows: once per 10 s");
    expect(link.traceDue(expired, 11.0) && link.traceDue(late, 12.0), "news: every window");
    expect(!link.traceDue(quiet, 21.9) && link.traceDue(quiet, 22.0), "a news line restarts the 10 s quiet timer");
    expect(LinkMonitor::traceFields(quiet) == "n=20 gap_ms=60 age_min_ms=3 age_max_ms=6 expired=0 tick_max_ms=22",
           "link_trace fields as before");
    expect(LinkMonitor::traceFields(outage) == "n=0 gap_ms=1500 age_min_ms=na age_max_ms=na expired=0 tick_max_ms=0",
           "na without a command");
}

int main() {
    try {
        wireStamps();
        linkWindow();
        longHoldWarning();
        restartSummary();
        stopRampSummary();
        repeatGate();
        periodGate();
        linkResumeWait();
        linkTraceRule();
        cout << checks << " field diagnostics checks PASS\n";
        return 0;
    } catch (const exception& e) { cerr << "FAIL: " << e.what() << "\n"; return 1; }
}

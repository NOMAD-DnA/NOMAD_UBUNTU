// 2026-11-17 field diagnostics: wire stamps, the link/loop window and the long-hold warning (ROS-free).
#include "nomad_control_cpp/link_stats.hpp"
#include "nomad_control_cpp/stop_output.hpp"
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

int main() {
    try {
        wireStamps();
        linkWindow();
        longHoldWarning();
        cout << checks << " field diagnostics checks PASS\n";
        return 0;
    } catch (const exception& e) { cerr << "FAIL: " << e.what() << "\n"; return 1; }
}

// Stop output policy without ROS: coast/brake/hold selection, hold latch and its fallback,
// VESC fault release, and the load-sized hold current (2026-11-13).
#include "nomad_control_cpp/stop_output.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
using namespace std;
using namespace nomad;

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; cerr << "FAIL: " << name << '\n'; }
}
bool is(const StopOutputCommand& command, StopOutputKind kind, double amps) {
    return command.kind == kind && abs(command.current_a - amps) < 1e-12;
}
bool rejected(StopOutputConfig config) {
    try { StopOutput output(config); } catch (const invalid_argument&) { return true; }
    return false;
}
// Drive for `seconds` at 50 Hz telemetry: the node calls onDrive every control tick (with the ERPM sent) and
// observeDrive per sample (avg_iq and the reported ERPM).
double drive(StopOutput& output, double t, double seconds, double command, double measured, double iq) {
    const double end = t + seconds;
    for (; t <= end + 1e-9; t += 0.02) { output.onDrive(command); output.observeDrive(t, iq, measured); }
    return t;
}
// Settled drive at 1000 ERPM.
double cruise(StopOutput& output, double t, double seconds, double iq) { return drive(output, t, seconds, 1000.0, 1000.0, iq); }
// Speed ramp from `from` to `to` ERPM over `seconds` at a constant current (command 1000 ERPM).
double ramp(StopOutput& output, double t, double seconds, double from, double to, double iq) {
    const double start = t, end = t + seconds;
    for (; t <= end + 1e-9; t += 0.02) {
        output.onDrive(1000.0);
        output.observeDrive(t, iq, from + (to - from) * (t - start) / seconds);
    }
    return t;
}
StopOutputInput still(double t) { return {t, true, true, 0.0, false, true}; }
// Real 11-13 20:18 flat hop (kp 0.010, drive 6): reported ERPM / avg_iq every 20 ms from the speed command on. The
// handover jump (645 -> 95 ERPM at 20-27 A) must not read as a stalled push or a lower bound.
const double kRealHop[][2] = {{0, 14}, {0, 14}, {0, 14}, {645, 5}, {160, 21}, {95, 22}, {244, 25}, {194, 27},
    {205, 27}, {411, 24}, {501, 24}, {554, 21}, {720, 20}, {810, 18}, {866, 17}, {969, 14}, {1057, 13},
    {1090, 11}, {1161, 9}, {1209, 8}, {1280, 6}, {1250, 5}, {1302, 3}, {1331, 3}, {1274, 2}, {1297, 1},
    {1242, 1}, {1283, 1}, {1251, 1}, {1204, 1}, {1271, 0}, {1174, 1}, {1174, 1}, {1192, 1}, {1149, 2},
    {1155, 0}, {1096, 1}, {1118, 2}, {1159, 0}, {1038, 2}};
// Real 11-13 20:18 steady flat cruise (drive 1, 1.0-2.4 s): ERPM / avg_iq with the kp 0.010 ripple.
const double kRealCruise[][2] = {{1046, 4.36}, {1049, 4.23}, {1054, 3.24}, {1041, 3.6}, {1035, 3.93}, {1026, 2.93},
    {1014, 3.85}, {1009, 3.54}, {1018, 3.06}, {1010, 3.82}, {994, 3.62}, {1004, 3.4}, {982, 3.99}, {1020, 3.99},
    {993, 3.4}, {989, 4.21}, {997, 4.23}, {964, 3.73}, {967, 4.41}, {991, 4.53}, {972, 3.67}, {976, 4.56},
    {969, 4.77}, {966, 4.1}, {955, 5.08}, {955, 5.21}, {968, 4.49}, {965, 5.17}, {971, 5.53}, {966, 5.11},
    {971, 5.68}, {974, 6.1}, {967, 5.5}, {986, 5.93}, {945, 6.29}, {952, 5.57}, {958, 6.2}, {944, 6.49},
    {964, 5.64}, {970, 6.22}, {962, 6.64}, {984, 5.98}, {985, 6.56}, {935, 6.77}, {990, 6.42}, {1005, 6.84},
    {928, 6.63}, {988, 6.47}, {1028, 6.68}, {940, 6.48}, {1022, 6.11}, {1039, 5.81}, {1022, 5.64}, {1036, 5.16},
    {1029, 5.08}, {1016, 4.92}, {1043, 4.78}, {1033, 4.74}, {1027, 4.62}, {1073, 4.03}, {1052, 4.64}, {1041, 4.69},
    {1054, 3.99}, {1042, 4.57}, {1019, 4.44}, {1031, 3.86}, {1014, 4.58}, {1016, 4.43}, {1011, 3.59}, {1003, 4.33},
    {1010, 4.19}};
// Feed {ERPM, avg_iq} rows every 20 ms (command 1000 ERPM), with `iq_offset` added to the current.
template <std::size_t N>
double replay(StopOutput& output, double t, const double (&rows)[N][2], double iq_offset = 0.0) {
    for (const auto& row : rows) { output.onDrive(1000.0); output.observeDrive(t, row[1] + iq_offset, row[0]); t += 0.02; }
    return t;
}

// Deployed candidate: brake 20 A, handbrake 12.5..30 A, gain 1.45, rolling 6 A, margin 8 A.
const StopOutputConfig kLoad{20.0, 30.0, 200.0, 12.5, 1.45, 6.0, 8.0};
// Gentle climb: iq 16 A = 10 A gravity over 6 A rolling -> 22.5 A (a 14 deg climb, 27 A, clamps at 30 A).
const double kGentle = 1.45 * 10.0 + 8.0;
}

int main() {
    const double nan_value = numeric_limits<double>::quiet_NaN();
    {
        StopOutput coast;
        expect(is(coast.decide({0.0, true, true, 0.0, false, true}), StopOutputKind::Coast, 0.0), "both disabled: 0A coast");
        expect(!coast.holding() && coast.erpmValid(), "coast never claims a hold");
        expect(string(stopOutputKindName(StopOutputKind::Coast)) == "coast" &&
               string(stopOutputKindName(StopOutputKind::Brake)) == "brake" &&
               string(stopOutputKindName(StopOutputKind::Hold)) == "hold", "kind names");
    }
    {
        StopOutput brake({20.0, 0.0, 200.0});
        expect(is(brake.decide({0.0, true, true, 0.0, false, true}), StopOutputKind::Brake, 20.0), "brake only: brake at standstill");
        expect(is(brake.decide({0.1, true, true, 1500.0, false, true}), StopOutputKind::Brake, 20.0), "brake only: brake at speed");
        expect(!brake.holding(), "brake only never holds");
    }
    {
        StopOutput hold_only({0.0, 15.0, 200.0});
        expect(is(hold_only.decide({0.0, true, true, 800.0, false, true}), StopOutputKind::Coast, 0.0),
               "hold only: 0A coast above the threshold");
        expect(is(hold_only.decide({0.1, true, true, 150.0, false, true}), StopOutputKind::Hold, 15.0), "hold only: hold at or below");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        expect(output.holdReleaseErpm() == 400.0, "release threshold is max(2x, +200) of the entry threshold");
        expect(is(output.decide({0.0, true, true, 900.0, false, true}), StopOutputKind::Brake, 20.0), "brake while above threshold");
        expect(!output.holding() && output.erpmValid(), "no hold while braking");
        expect(is(output.decide({0.1, true, true, 200.0, false, true}), StopOutputKind::Hold, 15.0), "hold at exactly the threshold");
        expect(output.holding() && !output.erpmValid(), "hold marks ERPM invalid");
        expect(is(output.decide({0.15, true, true, 900.0, false, true}), StopOutputKind::Hold, 15.0),
               "a high reading inside the grace period keeps the hold");
        expect(is(output.decide({0.3, true, true, 350.0, false, true}), StopOutputKind::Hold, 15.0),
               "latched hold ignores readings up to the release threshold");
        expect(is(output.decide({0.4, false, false, 0.0, false, true}), StopOutputKind::Hold, 15.0), "latched hold survives stale telemetry");
        expect(is(output.decide({0.45, false, false, 900.0, false, true}), StopOutputKind::Hold, 15.0),
               "a stale high reading does not end the hold");
        expect(abs(output.holdSeconds(1.1) - 1.0) < 1e-12, "hold time counts from entry");
        expect(output.last() && output.last()->kind == StopOutputKind::Hold, "last output is the hold");
        output.onDrive(1000.0);
        expect(!output.holding() && output.erpmValid() && !output.last(), "drive command ends the hold");
        expect(is(output.decide({2.0, true, true, 1200.0, false, true}), StopOutputKind::Brake, 20.0), "next stop selects again");
        expect(output.holdsAbandoned() == 0, "no hold abandoned so far");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        output.decide({0.0, true, true, 0.0, false, true});
        expect(is(output.decide({0.2, true, true, 600.0, false, true}), StopOutputKind::Brake, 20.0),
               "a fresh reading above the release threshold falls back to the brake");
        expect(!output.holding() && output.erpmValid() && output.holdsAbandoned() == 1, "abandoned hold is counted");
        expect(is(output.decide({0.3, true, true, 100.0, false, true}), StopOutputKind::Hold, 15.0), "slow again: hold again");
    }
    {
        StopOutput output({0.0, 15.0, 200.0});
        output.decide({0.0, true, true, 0.0, false, true});
        expect(is(output.decide({0.2, true, true, 600.0, false, true}), StopOutputKind::Coast, 0.0),
               "hold only: an ineffective hold falls back to 0A");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        expect(is(output.decide({0.0, true, true, 0.0, false, false}), StopOutputKind::Brake, 20.0),
               "no handbrake subscriber: never enter the hold");
        output.decide({0.1, true, true, 0.0, false, true});
        expect(output.holding(), "subscriber present: hold");
        expect(is(output.decide({0.15, true, true, 0.0, false, false}), StopOutputKind::Brake, 20.0),
               "subscriber lost: leave the hold for the brake");
        expect(output.holdsAbandoned() == 1, "lost subscriber is counted");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        expect(is(output.decide({0.0, false, false, 0.0, false, true}), StopOutputKind::Brake, 20.0),
               "stale telemetry never enters the hold");
        expect(is(output.decide({0.1, true, true, nan_value, false, true}), StopOutputKind::Brake, 20.0),
               "non-finite ERPM never enters the hold");
        expect(is(output.decide({0.2, true, true, -150.0, false, true}), StopOutputKind::Hold, 15.0), "rolling back slowly enters the hold");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        output.decide({0.0, true, true, 0.0, false, true});
        expect(output.holding(), "fault test starts holding");
        expect(is(output.decide({0.1, true, true, 0.0, true, true}), StopOutputKind::Coast, 0.0), "VESC fault releases with 0A");
        expect(!output.holding() && output.erpmValid(), "VESC fault ends the hold");
        expect(is(output.decide({0.2, true, true, 900.0, true, true}), StopOutputKind::Coast, 0.0), "VESC fault never brakes");
        expect(is(output.decide({0.3, true, true, 0.0, false, true}), StopOutputKind::Hold, 15.0),
               "after the fault is cleared a fresh slow sample holds again");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        output.decide({0.0, true, true, 0.0, false, true});
        expect(is(output.decide({0.3, false, true, 1600.0, false, true}), StopOutputKind::Brake, 20.0),
               "a rejected but recent high reading (e.g. overspeed) ends an ineffective hold");
        output.decide({0.4, true, true, 0.0, false, true});
        expect(is(output.decide({0.8, false, false, 1600.0, false, true}), StopOutputKind::Hold, 15.0),
               "an old high reading does not end the hold");
        expect(is(output.decide({0.9, false, true, 0.0, false, true}), StopOutputKind::Hold, 15.0),
               "unhealthy telemetry keeps an existing hold");
    }
    {
        StopOutput output({20.0, 15.0, 200.0});
        expect(is(output.decide({0.0, false, true, 0.0, false, true}), StopOutputKind::Brake, 20.0),
               "a recent but rejected sample never enters the hold");
    }
    {
        StopOutput fixed({20.0, 15.0, 200.0});
        cruise(fixed, 0.0, 2.0, 27.0);
        expect(is(fixed.decide(still(2.1)), StopOutputKind::Hold, 15.0) && fixed.holdBasis() == HoldBasis::Fixed &&
               !fixed.holdLoadA() && !fixed.loadHoldEnabled(), "minimum 0: fixed hold current, drive current unused");
    }
    {
        StopOutput output(kLoad);
        expect(output.loadHoldEnabled() && output.holdBasis() == HoldBasis::None && !output.holdLoadA(),
               "load hold: nothing reported before a hold");
        output.observeDrive(0.0, 16.0, 1000.0);
        expect(is(output.decide(still(0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::Unknown &&
               !output.holdLoadA() && output.movedErev() == 0.0,
               "no drive yet (samples outside a drive ignored): hold the maximum");
        expect(string(holdBasisName(HoldBasis::Unknown)) == "unknown" && string(holdBasisName(HoldBasis::None)) == "none" &&
               string(holdBasisName(HoldBasis::SamePlace)) == "same_place" &&
               string(holdBasisName(HoldBasis::Unsettled)) == "unsettled", "basis names");
    }
    {
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 16.0);
        expect(abs(output.movedErev() - 2.0 * 1000.0 / 60.0) < 0.5, "movement counts ERPM over time (electrical revolutions)");
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, kGentle) && output.holdBasis() == HoldBasis::Load &&
               output.holdLoadA() && abs(*output.holdLoadA() - 10.0) < 1e-12 && output.movedErev() == 0.0,
               "settled climb: gain * (iq - rolling) + margin; movement restarts at the hold");
        t = cruise(output, t + 1.0, 2.0, 6.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && output.holdBasis() == HoldBasis::Load &&
               abs(*output.holdLoadA()) < 1e-12, "settled flat drive: the minimum");
        output.onDrive(1000.0);
        expect(output.holdBasis() == HoldBasis::None && !output.holdLoadA(), "drive clears the reported basis");
    }
    {
        StopOutput down(kLoad);
        double t = cruise(down, 0.0, 2.0, -4.0);
        expect(is(down.decide(still(t + 0.1)), StopOutputKind::Hold, kGentle),
               "downhill: the braking speed PID (iq opposite the command) gives the same gravity current");
        StopOutput reverse(kLoad);
        t = drive(reverse, 0.0, 2.0, -1000.0, -1000.0, -16.0);
        expect(is(reverse.decide(still(t + 0.1)), StopOutputKind::Hold, kGentle), "reverse climb: current and speed along the command");
        StopOutput steep(kLoad);
        t = cruise(steep, 0.0, 2.0, 27.0);
        expect(is(steep.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0), "14 deg climb or steeper: clamped to the maximum");
    }
    {
        // Restart samples still carry the stop output (avg_iq is the mean since the previous read).
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 27.0);
        output.decide(still(t + 0.1));
        t = drive(output, t + 1.0, 0.05, 1000.0, 1000.0, 30.0); // 3 samples: the 30 A handbrake mean
        t = cruise(output, t, 0.6, 6.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && output.holdBasis() == HoldBasis::Load,
               "the first restart samples (previous handbrake current) are not used");
        StopOutput braked(kLoad);
        t = drive(braked, 0.0, 0.05, 1000.0, 1000.0, -20.0); // brake current while still rolling (settled, steady)
        t = cruise(braked, t, 0.6, 6.0);
        expect(is(braked.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5), "a restart from the brake is not read as a slope");
    }
    {
        // Reaching a slope: speed falls behind the command while the PID raises the current.
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1)); // flat hold 12.5 A
        t = cruise(output, t + 1.0, 3.0, 6.0);
        for (double s = 0.0; s < 0.5; s += 0.02, t += 0.02) { output.onDrive(1000.0); output.observeDrive(t, 6.0 + 28.0 * s, 1000.0 - 1600.0 * s); }
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::Unsettled,
               "stop while reaching a slope (not settled, moved since the flat hold): the maximum, not the flat 12.5 A");
        // Restart from that hold: the PID ramps while the car hardly moves.
        for (double s = 0.0; s < 1.4; s += 0.02, t += 0.02) { output.onDrive(1000.0); output.observeDrive(t, 10.0 + 10.0 * s, 100.0 * s); }
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::SamePlace,
               "stop during a stalled hill start: the previous hold current on the same spot");
    }
    {
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1));
        t = drive(output, t + 1.0, 0.4, 1000.0, 600.0, 10.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 1.45 * 4.0 + 8.0) && output.holdBasis() == HoldBasis::SamePlace &&
               abs(*output.holdLoadA() - 4.0) < 1e-12,
               "short hop that never settled: the previous hold current, raised to what its own drive current needs");
        t = drive(output, t + 1.0, 0.6, 1000.0, 600.0, 6.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 1.45 * 4.0 + 8.0) && output.holdBasis() == HoldBasis::SamePlace,
               "a later lighter hop keeps the previous hold current");
        t = drive(output, t + 1.0, 0.6, 1000.0, 600.0, 6.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::Unsettled,
               "short hops add up: past 15 electrical revolutions since the last sized hold, the maximum");
    }
    {
        // 11-12 21:42: a stalled push (33.8 A at ~15 ERPM) right after a hold on the same spot.
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1));
        t = drive(output, t + 1.0, 1.0, 1000.0, 15.0, 33.8);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::SamePlace,
               "a stalled high-current push raises a same-place hold to what its current needs");
    }
    {
        // A brief brake-only pause on a climb must not reset the evidence that the car moved.
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1)); // flat hold 12.5 A
        t = cruise(output, t + 1.0, 5.0, 27.0);
        expect(is(output.decide({t, true, true, 800.0, false, true}), StopOutputKind::Brake, 20.0), "pause above 200 ERPM brakes");
        t = drive(output, t + 0.1, 0.5, 1000.0, 200.0, 12.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::Unsettled,
               "stop soon after a brief pause on a climb: the maximum, not the old flat hold");
    }
    {
        StopOutput output(kLoad);
        double t = drive(output, 0.0, 2.0, 1000.0, 500.0, 16.0);
        t = cruise(output, t, 0.2, 16.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::Unsettled,
               "settled for less than 0.3 s: not trusted");
        StopOutput crest(kLoad);
        t = cruise(crest, 0.0, 1.0, 27.0);
        t = cruise(crest, t, 0.5, 6.0);
        expect(is(crest.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0), "a settled climb within the last second still counts");
        StopOutput old(kLoad);
        t = cruise(old, 0.0, 1.0, 27.0);
        t = cruise(old, t, 1.5, 6.0);
        expect(is(old.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5), "a climb older than the window does not count");
    }
    {
        StopOutput spike(kLoad);
        double t = cruise(spike, 0.0, 1.0, 6.0);
        spike.observeDrive(t, 50.0, 1000.0);
        t = cruise(spike, t + 0.02, 0.3, 6.0);
        expect(is(spike.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5), "a single current spike is filtered by the median");
        StopOutput nan_iq(kLoad);
        t = drive(nan_iq, 0.0, 0.05, 1000.0, 1000.0, 16.0); // skipped
        nan_iq.observeDrive(t, nan_value, 1000.0);
        t = cruise(nan_iq, t + 0.02, 0.8, 16.0);
        expect(is(nan_iq.decide(still(t + 0.1)), StopOutputKind::Hold, kGentle), "a non-finite current sample is ignored");
        StopOutput late(kLoad);
        t = cruise(late, 0.0, 2.0, 6.0);
        late.observeDrive(t - 1.0, 6.0, 0.0);
        expect(is(late.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5), "an out-of-order sample is ignored");
    }
    {
        // 11-13 review: slowing while the current rises above rolling is a lower bound of the load and counts, also
        // after a steady overshoot plateau and in the last 0.1 s (no complete centred window).
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1)); // flat hold 12.5 A
        t += 1.0;
        for (int k = 0; k < 30; ++k, t += 0.02) { output.onDrive(1000.0); output.observeDrive(t, kRealHop[k][1], kRealHop[k][0]); }
        for (double s = 0.0; s <= 0.25 + 1e-9; s += 0.02, t += 0.02) { // the front axle reaches a slope
            output.onDrive(1000.0);
            output.observeDrive(t, 2.0 + 13.0 * s / 0.25, 1100.0 - 450.0 * s / 0.25);
        }
        const auto held = output.decide(still(t + 0.1));
        expect(held.current_a >= 20.0 && output.holdBasis() == HoldBasis::SamePlace && output.holdLoadA() &&
               *output.holdLoadA() >= 8.0,
               "a hop that ends slowing onto a slope under rising current sizes from it (not the flat plateau)");
        StopOutput gentle(kLoad);
        t = cruise(gentle, 0.0, 1.5, 6.0);
        for (double s = 0.0; s <= 0.3 + 1e-9; s += 0.02, t += 0.02) { // settled, but the load rises before the stop
            gentle.onDrive(1000.0);
            gentle.observeDrive(t, 6.0 + 10.0 * s / 0.3, 1000.0 - 220.0 * s / 0.3);
        }
        // End: 16 A at -730 ERPM/s, i.e. ~12.6 A of load above rolling with ~0.0035 A per ERPM/s of inertia. The lower
        // bound reaches 10.1 A only with the last 0.1 s (trailing windows) and the inertia correction (17.9 / 20.6 A
        // without either).
        expect(gentle.decide(still(t + 0.1)).current_a >= 22.0 && gentle.holdBasis() == HoldBasis::Load &&
               *gentle.holdLoadA() >= 10.0 && *gentle.holdLoadA() <= 12.6,
               "settled drive whose load rises in its last 0.3 s: the rising (slowing) samples count, as a lower bound");
    }
    {
        // Stop 0.1 s after the front axle meets a slope: the drive ends slowing hard (-3,500 ERPM/s) with the current
        // rising above rolling, too early for any usable evidence. It falls back to its launch current.
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1)); // flat hold 12.5 A
        t += 1.0;
        for (int k = 0; k < 30; ++k, t += 0.02) { output.onDrive(1000.0); output.observeDrive(t, kRealHop[k][1], kRealHop[k][0]); }
        for (double s = 0.0; s <= 0.1 + 1e-9; s += 0.02, t += 0.02) {
            output.onDrive(1000.0);
            output.observeDrive(t, 1.0 + 11.0 * s / 0.1, 1150.0 - 350.0 * s / 0.1);
        }
        expect(output.decide(still(t + 0.1)).current_a >= 25.0 && output.holdBasis() == HoldBasis::SamePlace,
               "a drive that ends slowing hard under rising current uses at least its launch current");
    }
    {
        // A 0.4 s restart on a slope (slow launch: 0 -> 450 ERPM at 33 A) hardly moves but is not a tiny creep.
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1));
        t = ramp(output, t + 1.0, 0.4, 0.0, 450.0, 33.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::SamePlace,
               "a 0.4 s slope restart that moved < 3 erev: sized from its launch current, not the flat hold");
        StopOutput creep(kLoad);
        t = cruise(creep, 0.0, 2.0, 6.0);
        creep.decide(still(t + 0.1));
        t = ramp(creep, t + 1.0, 0.26, 0.0, 600.0, 33.0);
        const auto first = creep.decide(still(t + 0.1));
        t = ramp(creep, t + 1.0, 0.26, 0.0, 600.0, 33.0);
        const auto second = creep.decide(still(t + 0.1));
        t = ramp(creep, t + 1.0, 0.26, 0.0, 600.0, 33.0);
        const auto third = creep.decide(still(t + 0.1));
        expect(is(first, StopOutputKind::Hold, 12.5) && is(second, StopOutputKind::Hold, 12.5) &&
               is(third, StopOutputKind::Hold, 30.0),
               "creeping 0.26 s hops (~1.4 erev each) add up: past 3 erev in total their launch current sizes the hold");
    }
    {
        // 11-12 21:42-like stalled push at 33.8 A with the reported speed swinging -155..341 (range > 300): not
        // steady, but its slowing windows are lower bounds. A steady flat stretch earlier in the drive must not hide it.
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1));
        t = cruise(output, t + 1.0, 0.6, 6.0);
        const double noise[] = {0, 150, -80, 341, 20, -155, 90, 10, 200, -40};
        for (int k = 0; k < 40; ++k, t += 0.02) { output.onDrive(1000.0); output.observeDrive(t, 33.8, noise[k % 10]); }
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::SamePlace,
               "noisy stalled push after a flat stretch: 30 A");
    }
    {
        // Each drive record restarts its launch evidence: hop A (30 A, ended braking at 800 ERPM, no hold) must not size
        // hop B (8 A).
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1));
        t = ramp(output, t + 1.0, 0.35, 0.0, 600.0, 30.0);
        expect(is(output.decide({t, true, true, 800.0, false, true}), StopOutputKind::Brake, 20.0), "hop A ends braking");
        t = ramp(output, t + 0.5, 0.35, 0.0, 600.0, 8.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && output.holdBasis() == HoldBasis::SamePlace,
               "hop B is sized from its own launch only");
    }
    {
        // Real kp 0.010 ripple: flat cruise reads as flat; the same speed trace 27 A higher reads as a 14 deg climb.
        StopOutput flat(kLoad);
        double t = replay(flat, 0.0, kRealCruise);
        expect(is(flat.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && flat.holdBasis() == HoldBasis::Load &&
               *flat.holdLoadA() < 3.2, "real flat cruise ripple: 12.5 A");
        StopOutput climb(kLoad);
        t = replay(climb, 0.0, kRealCruise, 27.0);
        expect(is(climb.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && climb.holdBasis() == HoldBasis::Load,
               "the same ripple on a 14 deg climb: 30 A");
        StopOutput jitter(kLoad);
        t = 0.0;
        for (int k = 0; k < 100; ++k, t += 0.02) {
            jitter.onDrive(1000.0);
            jitter.observeDrive(t + 0.002 * (k % 3 - 1), 16.0, 1000.0);
        }
        expect(is(jitter.decide(still(t + 0.1)), StopOutputKind::Hold, kGentle), "telemetry jitter (+-2 ms): same result");
    }
    {
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 16.0);
        output.decide(still(t + 0.1));
        expect(is(output.decide({t + 0.4, true, true, 900.0, false, true}), StopOutputKind::Brake, 20.0),
               "load hold reading high falls back to the brake");
        expect(is(output.decide(still(t + 0.5)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::Escalated,
               "re-entry after a hold read high uses the maximum");
        expect(is(output.decide({t + 0.6, true, true, 0.0, false, false}), StopOutputKind::Brake, 20.0),
               "losing the subscriber leaves the escalated hold");
        t = cruise(output, t + 1.0, 2.0, 6.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5), "a new drive clears the escalation");
        expect(is(output.decide({t + 0.2, true, true, 0.0, true, true}), StopOutputKind::Coast, 0.0) &&
               is(output.decide(still(t + 0.3)), StopOutputKind::Hold, 12.5),
               "re-entry after a VESC fault keeps the load-sized current");
    }
    {
        // 11-13 flat launch with kp 0.010: +4,300 ERPM/s at ~21 A, overshoot to 1300, back at -1,000 ERPM/s at ~3 A.
        StopOutput output(kLoad);
        double t = ramp(output, 0.0, 0.3, 0.0, 1300.0, 21.0);
        t = ramp(output, t, 0.3, 1300.0, 1000.0, 3.0);
        t = cruise(output, t, 0.6, 5.5);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && output.holdBasis() == HoldBasis::Load &&
               *output.holdLoadA() < 3.2,
               "flat launch: acceleration and overshoot-recovery current are not read as slope (was ~28 A)");
        t = ramp(output, t + 1.0, 0.3, 0.0, 1300.0, 21.0);
        t = ramp(output, t, 0.25, 1300.0, 1050.0, 3.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && output.holdBasis() == HoldBasis::SamePlace,
               "short flat hop ending in the overshoot: steady samples give no extra load");
    }
    {
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 6.0);
        output.decide(still(t + 0.1)); // flat hold 12.5 A
        t = ramp(output, t + 1.0, 0.6, 0.0, 750.0, 33.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && output.holdBasis() == HoldBasis::SamePlace,
               "a hop that kept accelerating (no steady sample) and moved > 3 erev: sized from its launch current (onto a slope: 30 A)");
        StopOutput launch(kLoad);
        t = cruise(launch, 0.0, 2.0, 6.0);
        launch.decide(still(t + 0.1)); // flat hold 12.5 A
        t += 1.0;
        t = replay(launch, t, kRealHop);
        const auto launched = launch.decide(still(t + 0.1));
        expect(launched.kind == StopOutputKind::Hold && launched.current_a < 16.0 && launch.holdLoadA() && *launch.holdLoadA() < 5.5,
               "the launch speed jump with 20-27 A is not a steady (stalled-push) sample");
        StopOutput tiny(kLoad);
        t = cruise(tiny, 0.0, 2.0, 6.0);
        tiny.decide(still(t + 0.1));
        t = ramp(tiny, t + 1.0, 0.2, 0.0, 400.0, 20.0);
        expect(is(tiny.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && tiny.holdBasis() == HoldBasis::SamePlace,
               "a tiny creep (<= 3 erev, < 0.28 s) with no usable sample keeps the previous hold");
        StopOutput climb(kLoad);
        for (t = 0.0; t <= 2.0 + 1e-9; t += 0.02) {
            climb.onDrive(1000.0);
            climb.observeDrive(t, 33.0 + 3.0 * sin(6.283185307 * t), 1000.0 + 70.0 * sin(6.283185307 * t));
        }
        expect(is(climb.decide(still(t + 0.1)), StopOutputKind::Hold, 30.0) && climb.holdBasis() == HoldBasis::Load,
               "steady climb with the kp 0.010 speed ripple (+-70 ERPM at 1 Hz) counts as steady");
    }
    {
        StopOutput output(kLoad);
        double t = cruise(output, 0.0, 2.0, 16.0);
        t = drive(output, t, 0.6, -1000.0, -1000.0, -6.0);
        expect(is(output.decide(still(t + 0.1)), StopOutputKind::Hold, 12.5) && output.holdBasis() == HoldBasis::Load &&
               abs(*output.holdLoadA()) < 1e-12,
               "a direction change starts a new drive record (old forward samples and direction dropped)");
    }
    expect(rejected({-1.0, 0.0, 200.0}) && rejected({30.5, 0.0, 200.0}) && rejected({nan_value, 0.0, 200.0}),
           "invalid brake current rejected");
    expect(rejected({0.0, -1.0, 200.0}) && rejected({0.0, 30.5, 200.0}) && rejected({0.0, nan_value, 200.0}),
           "invalid hold current rejected");
    expect(rejected({0.0, 0.0, -1.0}) && rejected({0.0, 0.0, 2000.5}) && rejected({0.0, 0.0, nan_value}),
           "invalid hold threshold rejected");
    expect(rejected({0.0, 30.0, 200.0, 30.5}) && rejected({0.0, 0.0, 200.0, 10.0}) && rejected({0.0, 30.0, 200.0, -1.0}),
           "load hold minimum must be within 0..stop_handbrake_current_a");
    expect(rejected({0.0, 30.0, 200.0, 10.0, -0.1}) && rejected({0.0, 30.0, 200.0, 10.0, 5.1}) &&
           rejected({0.0, 30.0, 200.0, 10.0, nan_value}), "invalid load gain rejected");
    expect(rejected({0.0, 30.0, 200.0, 10.0, 1.0, 30.5}) && rejected({0.0, 30.0, 200.0, 10.0, 1.0, -1.0}) &&
           rejected({0.0, 30.0, 200.0, 10.0, 1.0, 6.0, 30.5}) && rejected({0.0, 30.0, 200.0, 10.0, 1.0, 6.0, -1.0}),
           "invalid rolling current or margin rejected");
    expect(!rejected({30.0, 30.0, 2000.0}) && !rejected({0.0, 0.0, 0.0}) && !rejected({30.0, 30.0, 2000.0, 30.0, 5.0, 30.0, 30.0}),
           "range limits accepted");
    cout << checks << " checks, " << failures << " failures\n";
    return failures ? 1 : 0;
}

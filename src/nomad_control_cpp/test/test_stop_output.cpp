// Stop output policy without ROS: coast/brake/hold selection, hold latch and its fallback,
// VESC fault release.
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
        output.onDrive();
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
    expect(rejected({-1.0, 0.0, 200.0}) && rejected({30.5, 0.0, 200.0}) && rejected({nan_value, 0.0, 200.0}),
           "invalid brake current rejected");
    expect(rejected({0.0, -1.0, 200.0}) && rejected({0.0, 30.5, 200.0}) && rejected({0.0, nan_value, 200.0}),
           "invalid hold current rejected");
    expect(rejected({0.0, 0.0, -1.0}) && rejected({0.0, 0.0, 2000.5}) && rejected({0.0, 0.0, nan_value}),
           "invalid hold threshold rejected");
    expect(!rejected({30.0, 30.0, 2000.0}) && !rejected({0.0, 0.0, 0.0}), "range limits accepted");
    cout << checks << " checks, " << failures << " failures\n";
    return failures ? 1 : 0;
}

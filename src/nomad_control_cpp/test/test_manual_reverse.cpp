// 2026-11-15 GUI MANUAL reverse (manual_reverse_max_erpm). ROS·장치 없이 raw 상태 머신만 합성 시험합니다.
#include "nomad_control_cpp/raw_trial.hpp"
#include <cmath>
#include <iostream>
using namespace std;

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; cerr << "FAIL: " << name << '\n'; }
}
nomad::RawOptions options(double reverse) {
    nomad::RawOptions o;
    o.enabled = o.setup_acknowledged = o.hardware_timeout_verified = true;
    o.servo_min = 0.0; o.servo_max = 1.0; o.servo_center = 0.5;
    o.manual_reverse_max_erpm = reverse;
    return o;
}
struct Rig {
    nomad::RawTrialController c;
    double steady = 10.0, ros = 100.0, measured = 0.0;
    explicit Rig(double reverse) : c(options(reverse)) {}
    void feed(double dt = 0.05) { steady += dt; ros += dt; c.observe({ros, measured, 0.0, 0.0, 16.3, 0}, steady, ros); }
    void prime() { for (int i = 0; i < 13; ++i) feed(); }
    bool startManual() { return c.start(nomad::RawAxis::Manual, 0.0, 0.5, 0.0, steady, ros, true); }
    bool gui(double erpm) { return c.update(nomad::RawAxis::Manual, erpm, 0.5, steady, ros, 0.0, true); }
    bool planner(double erpm) { return c.update(nomad::RawAxis::Manual, erpm, 0.5, steady, ros, 0.0, false); }
    nomad::RawOutput step(double erpm, bool via_gui = true) {
        feed();
        if (via_gui) gui(erpm); else planner(erpm);
        return c.tick(steady, ros, true);
    }
};

void testDefaultForwardOnly() {
    Rig r(0.0); r.prime();
    expect(r.startManual(), "manual start at 0");
    r.feed();
    expect(!r.gui(-300.0), "reverse refused when manual_reverse_max_erpm = 0 (previous behaviour)");
    expect(r.c.state() == nomad::TrialState::Fault, "invalid target still latches like before");
}

void testGuiReverseRampsAndLimit() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "manual start");
    nomad::RawOutput out{};
    for (int i = 0; i < 4; ++i) out = r.step(-600.0);
    expect(out.kind == nomad::OutputKind::Drive && out.motor_erpm < 0.0 && out.motor_erpm > -600.0,
           "reverse ramps from 0 toward the target (not a jump)");
    for (int i = 0; i < 20; ++i) { r.measured = r.c.commandedMotorErpm(); out = r.step(-600.0); }
    expect(std::abs(out.motor_erpm + 600.0) < 1e-6, "reverse reaches the target");
    r.feed();
    expect(!r.gui(-900.0), "reverse beyond manual_reverse_max_erpm refused");
}

void testPlannerStaysForwardOnly() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start");
    r.feed();
    expect(!r.planner(-200.0), "planning path (allow_reverse=false) never reverses even when the option is set");
}

void testDirectionChangeStopsFirst() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start");
    nomad::RawOutput out{};
    for (int i = 0; i < 30; ++i) { r.measured = r.c.commandedMotorErpm(); out = r.step(700.0); }
    expect(out.motor_erpm > 600.0, "forward running");
    r.measured = 650.0;
    out = r.step(-500.0);
    expect(out.motor_erpm == 0.0, "forward -> reverse while rolling forward releases to 0 (stop output), no drive through zero");
    out = r.step(-500.0);
    expect(out.motor_erpm == 0.0, "still released while the wheel turns forward");
    r.measured = 0.0;
    for (int i = 0; i < 2; ++i) out = r.step(-500.0);
    expect(out.motor_erpm < 0.0, "reverse starts once the wheel is nearly still");
    r.measured = -450.0;
    out = r.step(500.0);
    expect(out.motor_erpm == 0.0, "reverse -> forward also stops first");
}

void testForwardUnchanged() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start");
    auto out = r.step(1000.0);
    const double first = out.motor_erpm;
    expect(first > 0.0 && first <= 1200.0 * 0.05 * 2 + 1e-6, "forward still ramps at motor_ramp_erpm_s");
    for (int i = 0; i < 40; ++i) { r.measured = r.c.commandedMotorErpm(); out = r.step(1000.0); }
    r.measured = 1000.0;
    out = r.step(400.0);
    expect(out.motor_erpm < 1000.0 && out.motor_erpm >= 400.0, "forward slow-down uses the deceleration ramp");
    out = r.step(0.0);
    expect(out.motor_erpm == 0.0, "zero still releases immediately");
}

void testOptionBounds() {
    bool threw = false;
    try { nomad::RawTrialController c(options(-1.0)); } catch (const std::exception&) { threw = true; }
    expect(threw, "negative reverse limit rejected");
    threw = false;
    try { nomad::RawTrialController c(options(5000.0)); } catch (const std::exception&) { threw = true; }
    expect(threw, "reverse limit above motor_max_erpm rejected");
}
} // namespace

int main() {
    testDefaultForwardOnly();
    testGuiReverseRampsAndLimit();
    testPlannerStaysForwardOnly();
    testDirectionChangeStopsFirst();
    testForwardUnchanged();
    testOptionBounds();
    cout << "manual reverse tests: " << checks - failures << "/" << checks << " passed\n";
    return failures ? 1 : 0;
}

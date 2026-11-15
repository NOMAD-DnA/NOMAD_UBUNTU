// 2026-11-15 GUI MANUAL reverse (manual_reverse_max_erpm). ROS·장치 없이 raw 상태 머신만 합성 시험합니다.
// 11-15 review: a reversal is the operator's (target against the last drive direction); a measured roll-back or a
// one-sample jump of the reported speed with a same-direction target keeps driving, and the wait is bounded.
#include "nomad_control_cpp/raw_trial.hpp"
#include <cmath>
#include <iostream>
#include <string>
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
    o.stall_watchdog_enabled = true; // as commissioning.yaml
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
    // Drive toward erpm with the measured speed following the command.
    nomad::RawOutput cruise(double erpm, int steps) {
        nomad::RawOutput out{};
        for (int i = 0; i < steps; ++i) { measured = c.commandedMotorErpm(); out = step(erpm); }
        return out;
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
    expect(abs(out.motor_erpm + 600.0) < 1e-6, "reverse reaches the target");
    r.feed();
    expect(r.gui(-800.0), "reverse at exactly manual_reverse_max_erpm accepted");
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
    expect(r.c.reversalWaitActive(), "the reversal wait is reported");
    out = r.step(-500.0);
    expect(out.motor_erpm == 0.0, "still released while the wheel turns forward");
    r.measured = 0.0;
    for (int i = 0; i < 2; ++i) out = r.step(-500.0);
    expect(out.motor_erpm < 0.0, "reverse starts once the wheel is nearly still");
    expect(!r.c.reversalWaitActive() && r.c.reversalWaitEnd() == "still", "the reversal wait ends (still)");
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

// 11-15 review blocker: a forward hill start that rolls back must keep driving forward (the speed PID pushes up to
// its current limit), with reverse off and on. The first version released to the stop output on measured < -50.
void testForwardWhileRollingBackKeepsDriving() {
    for (const double reverse : {0.0, 800.0}) {
        Rig r(reverse); r.prime();
        expect(r.startManual(), "start (roll-back)");
        r.measured = -600.0;
        bool always_drive = true;
        nomad::RawOutput out{};
        for (int i = 0; i < 30; ++i) {
            out = r.step(800.0);
            always_drive = always_drive && out.kind == nomad::OutputKind::Drive && out.motor_erpm > 0.0;
        }
        expect(always_drive, "forward target while rolling back at -600 drives forward every tick");
        expect(abs(out.motor_erpm - 800.0) < 1e-6, "and reaches the forward target (no restart from 0)");
        expect(r.c.state() == nomad::TrialState::Running && !r.c.reversalWaitActive(), "no wait, no fault");
        r.measured = -150.0;
        out = r.step(800.0);
        expect(out.motor_erpm == 800.0, "a slow roll-back (-150) does not release either");
    }
}

// Mirror case: backing up a slope, the car rolls forward under a reverse target.
void testReverseWhileRollingForwardKeepsDriving() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (reverse uphill)");
    r.cruise(-500.0, 20);
    r.measured = 300.0;
    bool always_drive = true;
    for (int i = 0; i < 10; ++i) always_drive = always_drive && r.step(-500.0).motor_erpm < 0.0;
    expect(always_drive, "reverse target while rolling forward keeps the reverse drive");
}

// One-sample jumps of the reported speed (the handbrake handover, HFI noise) are not a reversal.
void testOneSampleJumpKeepsDriving() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (jump)");
    auto out = r.cruise(1000.0, 5);
    const double before = out.motor_erpm;
    r.measured = -256.0;
    out = r.step(1000.0);
    expect(out.motor_erpm > before, "forward launch: a -256 ERPM sample does not cut the ramp");
    r.cruise(0.0, 1);
    r.measured = 0.0;
    for (int i = 0; i < 3; ++i) r.step(0.0);
    out = r.cruise(-600.0, 3);
    const double reverse_before = out.motor_erpm;
    expect(reverse_before < 0.0, "reverse launch after a stop");
    r.measured = 208.0;
    out = r.step(-600.0);
    expect(out.motor_erpm < reverse_before, "reverse launch: a +208 ERPM sample does not cut the ramp");
}

// A reversal whose wheel never gets still (a slope the 20 A brake cannot hold) waits at most kReversalWaitMaxS.
void testReversalWaitIsBounded() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (bounded)");
    r.cruise(700.0, 30);
    r.measured = 650.0;
    const double begin = r.steady + 0.05;
    nomad::RawOutput out{};
    double first_drive_at = -1.0;
    for (int i = 0; i < 20 && first_drive_at < 0.0; ++i) {
        out = r.step(-500.0);
        if (out.motor_erpm < 0.0) first_drive_at = r.steady - begin;
        else expect(out.motor_erpm == 0.0, "released while waiting");
    }
    expect(first_drive_at >= nomad::RawTrialController::kReversalWaitMaxS - 1e-6 &&
           first_drive_at <= nomad::RawTrialController::kReversalWaitMaxS + 1e-6,
           "the new direction drives from 0 after kReversalWaitMaxS (not a tick later)");
    expect(r.c.reversalWaitEnd() == "timeout", "the wait end is reported as a timeout");
    expect(out.motor_erpm > -100.0, "starting from 0, not a jump");
    expect(r.c.state() == nomad::TrialState::Running, "no fault");
}

// The last drive direction is kept across a key release; a reversal right after it still waits for the wheel.
void testReversalAfterKeyRelease() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (release)");
    r.cruise(700.0, 30);
    r.measured = 650.0;
    expect(r.step(0.0).motor_erpm == 0.0, "key released");
    r.measured = 600.0;
    expect(r.step(-500.0).motor_erpm == 0.0, "reverse pressed while still rolling forward waits");
    r.measured = 0.0; // handbrake hold: the reported speed is 0
    expect(r.step(-500.0).motor_erpm < 0.0, "then reverses from 0");
    // Released again and the car rolls backward: forward after a reverse drive waits only while it rolls backward.
    r.measured = 0.0;
    expect(r.step(0.0).motor_erpm == 0.0, "released");
    r.measured = 400.0; // pushed forward (e.g. downhill) while released
    expect(r.step(500.0).motor_erpm > 0.0, "forward while already rolling forward needs no wait");
}

// The wheel does not turn the old way (blocked, creeping within stationary_erpm, or already rolling the new way):
// no wait, and the command restarts from 0 instead of decelerating the old way first.
void testReversalWithWheelStillStartsFromZero() {
    for (const double measured : {0.0, 50.0, 30.0, -600.0}) {
        Rig r(800.0); r.prime();
        expect(r.startManual(), "start (still reversal)");
        r.cruise(700.0, 30);
        r.measured = measured;
        const auto out = r.step(-500.0);
        expect(out.motor_erpm < 0.0 && out.motor_erpm >= -1200.0 * 0.05 - 1e-6,
               "forward -> reverse with the wheel not turning forward: reverse from 0 at once");
        expect(!r.c.reversalWaitActive(), "no reversal wait when the wheel does not turn the old way");
    }
}

// 11-15 second review: on a slope the 20 A brake cannot hold, the old-way speed rises; the wait ends at once so the
// speed PID (which brakes harder) takes over, instead of running on for the whole kReversalWaitMaxS.
void testReversalEndsWhenBrakeLoses() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (brake losing)");
    r.cruise(700.0, 30);
    r.measured = 650.0;
    expect(r.step(-500.0).motor_erpm == 0.0 && r.c.reversalWaitActive(), "reversal waits on the brake");
    r.measured = 600.0;
    expect(r.step(-500.0).motor_erpm == 0.0, "the brake is winning: still waiting");
    r.measured = 700.0; // 100 above the lowest (600): not yet losing
    expect(r.step(-500.0).motor_erpm == 0.0 && r.c.reversalWaitActive(), "a rise up to kReversalLosingErpm keeps waiting");
    r.measured = 710.0;
    const auto out = r.step(-500.0);
    expect(out.motor_erpm < 0.0 && !r.c.reversalWaitActive() && r.c.reversalWaitEnd() == "brake losing",
           "the old-way speed rose more than kReversalLosingErpm: the new direction drives from 0");
    r.measured = 900.0;
    expect(r.step(-500.0).motor_erpm < out.motor_erpm && !r.c.reversalWaitActive(), "and it does not wait again");
}

void testReversalWaitReleasedAndStopped() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (released)");
    r.cruise(700.0, 30);
    r.measured = 650.0;
    r.step(-500.0);
    expect(r.c.reversalWaitActive(), "waiting");
    expect(r.step(0.0).motor_erpm == 0.0 && !r.c.reversalWaitActive() && r.c.reversalWaitEnd() == "released",
           "key released during the wait");
    r.step(-500.0);
    expect(r.c.reversalWaitActive(), "pressed again while still rolling forward: waits again");
    r.c.stop("operator STOP");
    expect(!r.c.reversalWaitActive() && r.c.reversalWaitEnd() == "stopped", "STOP clears the wait");
}

// The wait starts at the press, not at an earlier key release.
void testReleaseLongerThanWaitStillWaits() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (long release)");
    r.cruise(700.0, 30);
    r.measured = 600.0;
    for (int i = 0; i < 20; ++i) r.step(0.0);
    const auto out = r.step(-500.0);
    expect(out.motor_erpm == 0.0 && r.c.reversalWaitActive(), "reverse after a 1 s release while rolling forward waits");
}

// A blocked wheel in reverse trips the stall watchdog like forward (|ERPM| checks).
void testReverseStallWatchdog() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (reverse stall)");
    r.measured = 0.0;
    nomad::RawOutput out{};
    for (int i = 0; i < 45 && r.c.state() == nomad::TrialState::Running; ++i) out = r.step(-600.0);
    expect(r.c.state() == nomad::TrialState::Fault && r.c.reason().find("stalled") != string::npos,
           "reverse without rotation trips the stall watchdog after stall_timeout_s");
    expect(out.kind == nomad::OutputKind::Release, "and releases the motor");
}

// A new session forgets the last drive direction: its first press never waits.
void testNewSessionForgetsDirection() {
    Rig r(800.0); r.prime();
    expect(r.startManual(), "start (session 1)");
    r.cruise(-500.0, 20);
    r.c.stop("operator STOP");
    r.measured = 0.0;
    r.prime();
    expect(r.c.reset(r.steady, r.ros) && r.startManual(), "session 2 starts");
    r.measured = -600.0;
    const auto out = r.step(800.0);
    expect(out.motor_erpm > 0.0 && !r.c.reversalWaitActive(), "new session: forward while rolling back drives at once");
}

// The fixed-duration MOTOR bench trial never reverses and ignores the measured sign as before.
void testMotorAxisUnaffected() {
    Rig r(800.0); r.prime();
    expect(r.c.start(nomad::RawAxis::Motor, 600.0, 0.5, 2.0, r.steady, r.ros, true), "motor trial start");
    nomad::RawOutput out{};
    for (int i = 0; i < 5; ++i) { r.measured = r.c.commandedMotorErpm(); r.feed(); out = r.c.tick(r.steady, r.ros, true); }
    const double before = out.motor_erpm;
    r.measured = -120.0;
    r.feed();
    out = r.c.tick(r.steady, r.ros, true);
    expect(out.motor_erpm > before, "a negative sample does not release the MOTOR trial");
}

void testOptionBounds() {
    bool threw = false;
    try { nomad::RawTrialController c(options(-1.0)); } catch (const exception&) { threw = true; }
    expect(threw, "negative reverse limit rejected");
    threw = false;
    try { nomad::RawTrialController c(options(5000.0)); } catch (const exception&) { threw = true; }
    expect(threw, "reverse limit above motor_max_erpm rejected");
}
} // namespace

int main() {
    testDefaultForwardOnly();
    testGuiReverseRampsAndLimit();
    testPlannerStaysForwardOnly();
    testDirectionChangeStopsFirst();
    testForwardUnchanged();
    testForwardWhileRollingBackKeepsDriving();
    testReverseWhileRollingForwardKeepsDriving();
    testOneSampleJumpKeepsDriving();
    testReversalWaitIsBounded();
    testReversalAfterKeyRelease();
    testMotorAxisUnaffected();
    testReversalWithWheelStillStartsFromZero();
    testReversalEndsWhenBrakeLoses();
    testReversalWaitReleasedAndStopped();
    testReleaseLongerThanWaitStillWaits();
    testReverseStallWatchdog();
    testNewSessionForgetsDirection();
    testOptionBounds();
    cout << "manual reverse tests: " << checks - failures << "/" << checks << " passed\n";
    return failures ? 1 : 0;
}

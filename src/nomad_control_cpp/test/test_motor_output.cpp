// Motor output stage without ROS (2026-11-20 refactor 5b, motor_output.hpp): which topic and value each output call
// sends, the drive start with the stop output state before it, the stop output fallbacks, and the one-time log flags.
#include "nomad_control_cpp/motor_output.hpp"
#include <cmath>
#include <iostream>
#include <optional>
using namespace std;
using namespace nomad;

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; cerr << "FAIL: " << name << '\n'; }
}
bool near(double a, double b) { return abs(a - b) < 1e-9; }
bool is(const MotorCommand& c, MotorTopic topic, double value) { return c.topic == topic && near(c.value, value); }

// The deployed stop output: brake 20 A above 200 ERPM, handbrake 12.5..30 A sized from the drive load.
StopOutputConfig deployed() {
    StopOutputConfig c;
    c.brake_current_a = 20.0;
    c.hold_current_a = 30.0;
    c.hold_below_erpm = 200.0;
    c.hold_min_current_a = 12.5;
    return c;
}

// The node's StopOutput inputs; counts how often the stage asked for them.
struct Inputs {
    StopOutputInput input{0.0, true, true, 0.0, false, true};
    int calls = 0;
    StopOutputInput operator()() { ++calls; return input; }
    Inputs& at(double t, double erpm) { input.steady_now = t; input.erpm = erpm; return *this; }
};

void driveAndRelease() {
    StopOutput stop(deployed());
    MotorOutputStage stage(stop);
    stage.setStopTopics(true, true);
    Inputs in;
    auto c = stage.output(OutputKind::Drive, 1000.0, 0.52, 1.0, true, in.at(1.0, 0.0));
    expect(is(c, MotorTopic::Speed, 1000.0) && c.servo && near(*c.servo, 0.52) && stage.driving() && in.calls == 0,
           "a drive: the speed topic, the servo first; no stop output inputs needed");
    expect(c.drive_started && !c.drive_started->kind && near(c.drive_started->hold_s, 0.0) &&
               near(c.drive_started->stop_a, 0.0),
           "the first drive starts one, from no stop output yet");
    c = stage.output(OutputKind::Drive, 1000.0, nullopt, 1.02, true, in);
    expect(is(c, MotorTopic::Speed, 1000.0) && !c.servo && !c.drive_started, "the next drive tick does not start one again");

    // Release while still fast: the brake current.
    c = stage.output(OutputKind::Release, 0.0, 0.5, 1.04, true, in.at(1.04, 900.0));
    expect(is(c, MotorTopic::Brake, 20.0) && !c.servo && !stage.driving() && in.calls == 1 && !c.hold_started,
           "a release above 200 ERPM: the brake current, no servo, driving ends");
    // At standstill: the handbrake, logged once.
    c = stage.release(true, in.at(1.30, 0.0));
    expect(c.topic == MotorTopic::Handbrake && c.hold_started && near(c.hold_a, c.value) && stop.holding() &&
               c.value >= 12.5 && c.value <= 30.0,
           "at or below 200 ERPM: the handbrake; hold_started with its current");
    const double hold_a = c.value;
    c = stage.release(true, in.at(1.32, 0.0));
    expect(is(c, MotorTopic::Handbrake, hold_a) && !c.hold_started, "the hold repeats every tick, logged only once");

    // A restart from the hold: the [RESTART_TRACE] start figures are the hold's, taken before StopOutput::onDrive.
    c = stage.output(OutputKind::Drive, 1000.0, 0.5, 2.80, true, in);
    expect(c.drive_started && c.drive_started->kind == StopOutputKind::Hold && near(c.drive_started->stop_a, hold_a) &&
               near(c.drive_started->hold_s, 2.80 - 1.30) && !stop.holding() && is(c, MotorTopic::Speed, 1000.0),
           "a restart from the hold: from=hold, hold_s 1.5, stop_a the hold current; the hold then ends");

    // Drive with 0 ERPM is a release, with the servo.
    c = stage.output(OutputKind::Drive, 0.0, 0.48, 2.82, true, in.at(2.82, 950.0));
    expect(is(c, MotorTopic::Brake, 20.0) && c.servo && near(*c.servo, 0.48) && !stage.driving(),
           "a drive with 0 ERPM: the stop output, and the servo first");
    // A restart from the brake: from=brake, stop_a 20, no hold time.
    c = stage.output(OutputKind::Drive, 1000.0, nullopt, 3.0, true, in);
    expect(c.drive_started && c.drive_started->kind == StopOutputKind::Brake && near(c.drive_started->stop_a, 20.0) &&
               near(c.drive_started->hold_s, 0.0),
           "a restart from the brake: from=brake, stop_a 20 A, hold_s 0");
    // No output kind: nothing, and the driving state stays.
    const int calls = in.calls;
    c = stage.output(OutputKind::None, 0.0, 0.5, 3.02, true, in);
    expect(c.topic == MotorTopic::None && !c.servo && !c.drive_started && stage.driving() && in.calls == calls,
           "OutputKind::None: nothing is sent and the drive goes on");
}

void notOwned() {
    StopOutput stop(deployed());
    MotorOutputStage stage(stop);
    stage.setStopTopics(true, true);
    Inputs in;
    stage.output(OutputKind::Drive, 1000.0, nullopt, 1.0, true, in);
    const auto c = stage.release(false, in.at(1.02, 0.0));
    expect(c.topic == MotorTopic::None && in.calls == 0 && !stage.driving() && !stop.holding(),
           "not owning the output (or no current publisher): nothing is sent, the inputs are not built, driving ends");
    const auto d = stage.output(OutputKind::Release, 0.0, nullopt, 1.04, false, in);
    expect(d.topic == MotorTopic::None && in.calls == 0, "the same through output()");
}

void fallbacks() {
    // No brake publisher: a brake decision goes out as 0 A on the current topic.
    StopOutput a(deployed());
    MotorOutputStage no_brake(a);
    no_brake.setStopTopics(false, true);
    Inputs in;
    expect(is(no_brake.release(true, in.at(1.0, 900.0)), MotorTopic::Current, 0.0),
           "a brake without a brake publisher: 0 A on the current topic");
    // No handbrake publisher: a hold decision goes out as 0 A, but the chosen hold is still reported for the log.
    StopOutput b(deployed());
    MotorOutputStage no_hold(b);
    no_hold.setStopTopics(true, false);
    const auto c = no_hold.release(true, in.at(1.0, 0.0));
    expect(is(c, MotorTopic::Current, 0.0) && c.hold_started && c.hold_a >= 12.5,
           "a hold without a handbrake publisher: 0 A, hold_started with the chosen current (as before 11-20)");
    // Unhealthy telemetry: no hold entry, the brake instead.
    StopOutput d(deployed());
    MotorOutputStage stale(d);
    stale.setStopTopics(true, true);
    Inputs old;
    old.input.telemetry_healthy = false;
    expect(stale.release(true, old.at(1.0, 0.0)).topic == MotorTopic::Brake, "stale telemetry: the brake, not the hold");
}

void vescFault() {
    StopOutput stop(deployed());
    MotorOutputStage stage(stop);
    stage.setStopTopics(true, true);
    Inputs in;
    in.input.vesc_fault = true;
    auto c = stage.release(true, in.at(1.0, 900.0));
    expect(is(c, MotorTopic::Current, 0.0) && c.fault_release_started, "a latched VESC fault: 0 A, warn once");
    c = stage.release(true, in.at(1.02, 900.0));
    expect(is(c, MotorTopic::Current, 0.0) && !c.fault_release_started, "not warned again while it stays");
    in.input.vesc_fault = false;
    stage.release(true, in.at(1.04, 900.0));
    in.input.vesc_fault = true;
    expect(stage.release(true, in.at(1.06, 900.0)).fault_release_started, "a new fault after a clear one warns again");
    // Not owned: the warning state does not change (as before: the node returned before it).
    StopOutput s2(deployed());
    MotorOutputStage other(s2);
    other.setStopTopics(true, true);
    Inputs f;
    f.input.vesc_fault = true;
    other.release(false, f);
    expect(other.release(true, f.at(1.0, 0.0)).fault_release_started, "a release while not owning does not use up the warning");
}

// observeTelemetry feeds the load-sized hold only while driving.
void driveLoad() {
    const auto holdAfter = [](bool observe_while_driving) {
        StopOutput stop(deployed());
        MotorOutputStage stage(stop);
        stage.setStopTopics(true, true);
        Inputs in;
        double t = 0.0;
        if (!observe_while_driving) stage.release(true, in.at(t, 1000.0));
        for (; t < 3.0; t += 0.02) {
            if (observe_while_driving) stage.output(OutputKind::Drive, 1000.0, nullopt, t, true, in);
            else stop.onDrive(1000.0);  // the drive happens, but the stage is not driving: samples are dropped
            stage.observeTelemetry(t, 6.0, 1000.0);
        }
        return stage.release(true, in.at(t, 0.0));
    };
    const auto settled = holdAfter(true), unseen = holdAfter(false);
    expect(settled.topic == MotorTopic::Handbrake && near(settled.value, 12.5) && settled.hold_started &&
               settled.moved_erev > 40.0,
           "3 s at 1000 ERPM and 6 A observed while driving: a flat hold, 12.5 A; moved_erev read before decide (~50)");
    expect(unseen.topic == MotorTopic::Handbrake && near(unseen.value, 30.0),
           "the same samples while not driving are not used: no load known, 30 A");
}
// A hold that reads high is given up (brake) and entered again in the same stop: each new hold is logged.
void holdGivenUp() {
    StopOutput stop(deployed());
    MotorOutputStage stage(stop);
    stage.setStopTopics(true, true);
    Inputs in;
    double t = 0.0;
    for (; t < 3.0; t += 0.02) {
        stage.output(OutputKind::Drive, 1000.0, nullopt, t, true, in);
        stage.observeTelemetry(t, 6.0, 1000.0);
    }
    auto c = stage.release(true, in.at(t, 0.0));
    expect(c.topic == MotorTopic::Handbrake && c.hold_started, "the first hold of the stop");
    c = stage.release(true, in.at(t + 0.2, 900.0));
    expect(is(c, MotorTopic::Brake, 20.0) && !c.hold_started, "a hold reading 900 ERPM is given up: the brake");
    c = stage.release(true, in.at(t + 0.4, 0.0));
    expect(is(c, MotorTopic::Handbrake, 30.0) && c.hold_started && near(c.hold_a, 30.0),
           "entered again in the same stop: 30 A, logged again");
}
}  // namespace

int main() {
    driveAndRelease();
    notOwned();
    fallbacks();
    vescFault();
    driveLoad();
    holdGivenUp();
    cout << checks << " motor output checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

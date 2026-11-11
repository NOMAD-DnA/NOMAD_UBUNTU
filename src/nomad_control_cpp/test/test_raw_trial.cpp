#include "nomad_control_cpp/raw_trial.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}

nomad::RawOptions enabled() {
    nomad::RawOptions options;
    options.enabled = options.setup_acknowledged = options.hardware_timeout_verified = true;
    return options;
}

// 시간과 telemetry만 합성합니다. ROS·시리얼 장치·실제 차량과 연결하지 않습니다.
struct Rig {
    nomad::RawTrialController controller;
    double steady = 10.0, ros = 100.0;
    explicit Rig(nomad::RawOptions options = enabled()) : controller(options) {}
    void feed(double dt = 0.05, double erpm = 0.0) {
        steady += dt; ros += dt;
        controller.observe({ros, erpm, 0.0, 0.0, 16.3, 0}, steady, ros);
    }
    void prime() { for (int i = 0; i < 13; ++i) feed(); }
    bool start(nomad::RawAxis axis = nomad::RawAxis::Motor, double erpm = 1000.0,
               double servo = 0.50, double duration = 2.0, bool exclusive = true) {
        return controller.start(axis, erpm, servo, duration, steady, ros, exclusive);
    }
    bool update(nomad::RawAxis axis, double erpm, double servo) {
        return controller.update(axis, erpm, servo, steady, ros);
    }
    nomad::RawOutput tick(bool exclusive = true) { return controller.tick(steady, ros, exclusive); }
};

void testStartupAndGates() {
    Rig monitor(nomad::RawOptions{}); monitor.prime();
    expect(monitor.controller.state() == nomad::TrialState::Monitor, "disabled raw mode is MONITOR");
    expect(monitor.controller.healthy(monitor.steady, monitor.ros), "disabled mode may observe raw telemetry");
    expect(!monitor.start(), "monitor cannot start raw output");
    monitor.controller.stop("operator");
    const auto untouched = monitor.tick();
    expect(untouched.kind == nomad::OutputKind::None && !untouched.servo_position,
           "monitor never emits motor or servo output, including stop");
    expect(!monitor.controller.reset(monitor.steady, monitor.ros), "reset does not enable monitor");
    for (int gate = 0; gate < 2; ++gate) {
        auto options = enabled();
        if (gate == 0) options.setup_acknowledged = false;
        else options.hardware_timeout_verified = false;
        Rig r(options); r.prime();
        expect(!r.start(), "raw setup and firmware gates required");
        expect(r.tick().kind == nomad::OutputKind::None, "failed gate emits no output");
    }
    Rig r;
    expect(r.tick().kind == nomad::OutputKind::None, "enabled raw boot does not acquire output");
    expect(!r.start(), "no telemetry cannot start");
    r.feed();
    expect(!r.start(), "one stationary sample insufficient");
    r.prime();
    expect(!r.start(nomad::RawAxis::Motor, 1000.0, 0.5, 2.0, false), "exclusive driver required");
    expect(r.start(), "confirmed stationary raw trial starts");
    expect(!r.start(nomad::RawAxis::Steering, 0, 0.51, 3.0), "repeat start cannot replace axis or deadline");
    expect(!r.controller.reset(r.steady, r.ros), "running trial cannot reset");
}

void testMotorAxisAndRamp() {
    Rig r; r.prime();
    expect(r.start(nomad::RawAxis::Motor, 1850, 0.51, 3.0), "motor start at candidate ERPM cap");
    double previous = 0.0;
    for (int i = 0; i < 40; ++i) {
        r.feed();
        expect(r.update(nomad::RawAxis::Motor, 1850, 0.49), "motor heartbeat with unused servo remains valid");
        const auto output = r.tick();
        expect(output.kind == nomad::OutputKind::Drive && !output.servo_position,
               "motor axis NEVER emits a servo command");
        expect(output.motor_erpm >= previous && output.motor_erpm <= 1850 &&
               output.motor_erpm - previous <= 60.0 + 1e-8, "ERPM increase respects ramp and cap");
        previous = output.motor_erpm;
    }
    expect(std::abs(previous - 1850) < 1e-8, "motor ramp eventually reaches requested ERPM");
    r.feed(); r.update(nomad::RawAxis::Motor, 1000, 0.5);
    const auto reduced = r.tick();
    expect(std::abs(previous - reduced.motor_erpm - 120.0) < 1e-8,
           "positive lower target uses separate faster deceleration ramp");
    r.feed(); r.update(nomad::RawAxis::Motor, 0, 0.5);
    expect(r.tick().motor_erpm == 0.0 && r.controller.targetMotorErpm() == 0.0,
           "zero target releases motor on next control tick without speed ramp");
    r.controller.stop("operator stop");
    const auto stopped = r.tick();
    expect(stopped.kind == nomad::OutputKind::Release && stopped.motor_erpm == 0 && !stopped.servo_position,
           "operator stop bypasses ramp and emits only motor release");
    expect(!r.controller.manualActive() && !r.start(), "operator stop stays latched");
}

void testSteeringAxisAndContinuity() {
    Rig r; r.prime();
    expect(!r.start(nomad::RawAxis::Steering, 1, 0.51), "steering start rejects ANY nonzero motor target");
    expect(!r.controller.ownsOutput(), "rejected steering start does not acquire output");
    expect(r.start(nomad::RawAxis::Steering, 0, 0.51), "steering-only raw start");
    double previous = 0.5;
    for (int i = 0; i < 12; ++i) {
        r.feed();
        expect(r.update(nomad::RawAxis::Steering, 0, 0.51), "steering heartbeat");
        const auto output = r.tick();
        expect(output.kind == nomad::OutputKind::Drive && output.motor_erpm == 0.0 && output.servo_position,
               "steering axis NEVER emits nonzero motor target");
        if (output.servo_position) {
            expect(*output.servo_position >= previous && *output.servo_position <= 0.51 &&
                   *output.servo_position - previous <= 0.001 + 1e-9, "servo candidate range and slew bounded");
            previous = *output.servo_position;
        }
    }
    expect(std::abs(previous - 0.51) < 1e-9, "steering target reached as command, not measured angle");
    r.controller.stop("released hold");
    expect(!r.tick().servo_position, "release does not move servo back to center");
    expect(r.controller.reset(r.steady, r.ros), "stationary steering trial may reset");
    expect(r.tick().kind == nomad::OutputKind::Release && !r.tick().servo_position,
           "reset continues motor release without servo output");
    expect(r.start(nomad::RawAxis::Motor, 0, 0.49), "intervening motor trial accepted");
    r.feed();
    expect(!r.tick().servo_position, "motor session does not change last servo command");
    r.controller.stop("motor finished");
    r.controller.reset(r.steady, r.ros);
    expect(r.start(nomad::RawAxis::Steering, 0, 0.49), "opposite steering trial after explicit reset");
    r.feed();
    const auto next = r.tick();
    expect(next.servo_position && std::abs(*next.servo_position - 0.509) < 1e-9,
           "new steering trial slews from last command, not assumed center");
}

void testExpandedSteeringEnvelope() {
    auto options = enabled();
    expect(options.servo_min == 0.49 && options.servo_max == 0.51,
           "default profile keeps narrow steering range");
    options.servo_min = 0.45;
    options.servo_max = 0.55;
    for (const double target : {0.45, 0.55}) {
        Rig r(options); r.prime();
        expect(r.start(nomad::RawAxis::Steering, 0, target, 3.0),
               "explicit expanded profile accepts either endpoint");
        double previous = 0.50;
        bool bounded = true;
        for (int i = 0; i < 52; ++i) {
            r.feed();
            const bool renewed = r.update(nomad::RawAxis::Steering, 0, target);
            const auto output = r.tick();
            bounded = bounded && renewed && output.kind == nomad::OutputKind::Drive &&
                output.motor_erpm == 0.0 && output.servo_position.has_value();
            if (output.servo_position) {
                const double position = *output.servo_position;
                bounded = bounded && position >= 0.45 && position <= 0.55 &&
                    std::abs(position - previous) <= 0.001 + 1e-9;
                previous = position;
            }
            if (i == 39)
                expect(std::abs(previous - target) > 0.009,
                       "expanded endpoint cannot be reached in two seconds at unchanged slew");
        }
        expect(bounded, "expanded steering preserves slew, configured range and motor isolation");
        expect(std::abs(previous - target) < 1e-9,
               "expanded endpoint reached as a command within three seconds");
        for (int i = 0; i < 9; ++i) {
            r.feed();
            r.update(nomad::RawAxis::Steering, 0, target);
            r.tick();
        }
        const auto stopped = r.tick();
        expect(r.controller.state() == nomad::TrialState::Stopped &&
               stopped.kind == nomad::OutputKind::Release && !stopped.servo_position,
               "expanded profile still stops at three seconds without servo recentering");
    }
    for (const double target : {0.449, 0.551}) {
        Rig r(options); r.prime();
        expect(!r.start(nomad::RawAxis::Steering, 0, target),
               "expanded profile rejects start outside outer bounds");
        expect(r.tick().kind == nomad::OutputKind::None,
               "rejected expanded target does not acquire output");
        expect(r.start(nomad::RawAxis::Steering, 0, 0.50), "expanded profile starts at center");
        r.feed();
        expect(!r.update(nomad::RawAxis::Steering, 0, target),
               "expanded profile rejects out-of-range updates instead of clipping");
        const auto fault = r.tick();
        expect(r.controller.state() == nomad::TrialState::Fault &&
               fault.kind == nomad::OutputKind::Release && !fault.servo_position,
               "out-of-range expanded update stops without further servo commands");
    }
}

void testExplicitOuterSteeringEnvelope() {
    auto options = enabled();
    expect(options.servo_min == 0.49 && options.servo_max == 0.51 && options.servo_rate_s == 0.02,
           "outer envelope does not change narrow defaults or default slew");
    options.servo_min = 0.40;
    options.servo_max = 0.60;
    options.servo_rate_s = 0.04;
    // 독립된 합성 시험에서 중립 후보부터 각 끝값을 확인합니다. 실차 전체 범위 스윕이 아닙니다.
    for (const double target : {0.40, 0.60}) {
        Rig r(options); r.prime();
        expect(!r.start(nomad::RawAxis::Steering, 0.01, target, 3.0),
               "outer steering profile still rejects nonzero motor target");
        expect(r.start(nomad::RawAxis::Steering, 0, target, 3.0),
               "explicit outer profile accepts endpoint at configured slew");
        double previous = 0.50;
        bool bounded = true;
        for (int i = 1; i <= 50; ++i) {
            r.feed();
            const bool renewed = r.update(nomad::RawAxis::Steering, 0, target);
            const auto output = r.tick();
            bounded = bounded && renewed && output.kind == nomad::OutputKind::Drive &&
                output.motor_erpm == 0.0 && output.servo_position.has_value();
            if (output.servo_position) {
                const double position = *output.servo_position;
                bounded = bounded && position >= 0.40 && position <= 0.60 &&
                    std::abs(position - previous) <= 0.002 + 1e-9 &&
                    std::abs(target - position) <= std::abs(target - previous) + 1e-9;
                previous = position;
            }
            if (i == 20)
                expect(std::abs(std::abs(previous - 0.50) - 0.04) < 1e-9 &&
                       std::abs(previous - target) > 0.059,
                       "one second moves at most .04 and does not reach outer endpoint");
            if (i == 49)
                expect(std::abs(previous - target) > 0.0019,
                       "outer endpoint is not reached before 2.5 seconds");
        }
        expect(bounded, "outer steering preserves motor zero, configured range and slew on every tick");
        expect(std::abs(previous - target) < 1e-9,
               "outer endpoint reached as command at 2.5 seconds, not measured angle");
        for (int i = 0; i < 11; ++i) {
            r.feed();
            r.update(nomad::RawAxis::Steering, 0, target);
            r.tick();
        }
        const auto stopped = r.tick();
        expect(r.controller.state() == nomad::TrialState::Stopped &&
               stopped.kind == nomad::OutputKind::Release && stopped.motor_erpm == 0.0 &&
               !stopped.servo_position && r.controller.remaining(r.steady) == 0.0,
               "outer endpoint updates cannot extend three-second deadline or recenter on stop");
    }
    for (const double target : {0.399, 0.601}) {
        Rig r(options); r.prime();
        expect(!r.start(nomad::RawAxis::Steering, 0, target),
               "outer profile rejects target beyond .40-.60");
        expect(r.tick().kind == nomad::OutputKind::None,
               "invalid outer target does not acquire output");
        expect(r.start(nomad::RawAxis::Steering, 0, 0.50), "outer profile starts at center");
        r.feed();
        expect(!r.update(nomad::RawAxis::Steering, 0, target),
               "outer profile faults on out-of-range update without clipping");
        const auto fault = r.tick();
        expect(r.controller.state() == nomad::TrialState::Fault &&
               fault.kind == nomad::OutputKind::Release && fault.motor_erpm == 0.0 && !fault.servo_position,
               "invalid outer update emits only zero-motor release");
    }
    for (int fault = 0; fault < 4; ++fault) {
        Rig r(options); r.prime();
        expect(r.start(nomad::RawAxis::Steering, 0, 0.60, 3.0), "outer profile starts for expiry check");
        if (fault == 0) {
            for (int i = 0; i < 8; ++i) { r.feed(); r.tick(); }
        } else if (fault == 1) {
            r.feed(0.21);
        } else if (fault == 2) {
            r.feed(0.36);
        } else {
            r.steady += 0.31; r.ros += 0.31;
        }
        expect(!r.update(nomad::RawAxis::Steering, 0, 0.60),
               "outer profile rejects renewal after heartbeat, tick or telemetry expiry");
        const auto output = r.tick();
        expect(r.controller.state() == nomad::TrialState::Fault &&
               output.kind == nomad::OutputKind::Release && output.motor_erpm == 0.0 && !output.servo_position,
               "outer profile expiry stops without additional servo output");
    }
}

void testDeadlinesAndLateUpdates() {
    Rig r; r.prime(); r.start(nomad::RawAxis::Motor, 1000, 0.5, 1.0);
    for (int i = 0; i < 18; ++i) {
        r.feed();
        expect(r.update(nomad::RawAxis::Motor, 900, 0.5), "updates accepted before fixed deadline");
        r.tick();
    }
    r.feed(0.15);
    expect(!r.update(nomad::RawAxis::Motor, 900, 0.5), "late update cannot extend total duration");
    expect(r.controller.state() == nomad::TrialState::Stopped && r.controller.remaining(r.steady) == 0.0,
           "deadline ends the session and remaining time");
    expect(r.tick().kind == nomad::OutputKind::Release && !r.start(), "deadline requires explicit reset");

    Rig heartbeat; heartbeat.prime(); heartbeat.start();
    for (int i = 0; i < 8; ++i) { heartbeat.feed(); heartbeat.tick(); }
    expect(heartbeat.controller.state() == nomad::TrialState::Fault, "GUI loss stops despite fresh telemetry and timer");
    expect(!heartbeat.update(nomad::RawAxis::Motor, 1000, 0.5), "late GUI message cannot revive fault");

    Rig delayed; delayed.prime(); delayed.start(); delayed.feed(0.36);
    expect(!delayed.update(nomad::RawAxis::Motor, 1000, 0.5), "heartbeat timeout checked BEFORE renewal and timer");
    expect(delayed.tick().kind == nomad::OutputKind::Release, "late renewal releases");

    Rig stalled; stalled.prime(); stalled.start(); stalled.feed(0.21);
    expect(!stalled.update(nomad::RawAxis::Motor, 1000, 0.5), "update cannot hide missed control tick");
    expect(stalled.tick().kind == nomad::OutputKind::Release, "missed tick releases");
}

void testAxisAndInvalidTargets() {
    Rig switched; switched.prime(); switched.start();
    expect(!switched.update(nomad::RawAxis::Steering, 0, 0.5), "axis switch during run refused");
    expect(switched.controller.state() == nomad::TrialState::Fault && !switched.tick().servo_position,
           "axis switch faults without touching servo");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int kind = 0; kind < 7; ++kind) {
        Rig r; r.prime(); r.start();
        const double erpm = kind == 0 ? -1 : kind == 1 ? 1850.1 : kind == 2 ? nan : 1000;
        const double servo = kind == 3 ? 0.48 : kind == 4 ? 0.52 : kind == 5 ? nan : 0.5;
        const auto axis = kind == 6 ? static_cast<nomad::RawAxis>(99) : nomad::RawAxis::Motor;
        expect(!r.update(axis, erpm, servo), "invalid raw target faults, not clipped");
        expect(r.tick().kind == nomad::OutputKind::Release, "invalid raw target releases motor");
    }
    Rig steering; steering.prime(); steering.start(nomad::RawAxis::Steering, 0, 0.5);
    expect(!steering.update(nomad::RawAxis::Steering, 0.01, 0.5), "steering update rejects nonzero ERPM");
}

void testTelemetryAndClockFaults() {
    for (int kind = 0; kind < 9; ++kind) {
        Rig r; r.prime(); r.start();
        r.steady += 0.05; r.ros += 0.05;
        nomad::TelemetrySample sample{r.ros, 0, 0, 0, 16.3, 0};
        if (kind == 0) sample.raw_erpm = 2501;
        if (kind == 1) sample.raw_erpm = -2501;
        if (kind == 2) sample.current_motor_a = 30.01;
        if (kind == 3) sample.current_input_a = -30.01;
        if (kind == 4) sample.voltage_v = 0;
        if (kind == 5) sample.fault_code = 1;
        if (kind == 6) sample.raw_erpm = std::numeric_limits<double>::infinity();
        if (kind == 7) sample.stamp_s -= 0.31;
        if (kind == 8) sample.stamp_s += 0.1;
        r.controller.observe(sample, r.steady, r.ros);
        expect(r.controller.state() == nomad::TrialState::Fault, "bad telemetry latches immediately before timer");
        const auto output = r.tick();
        expect(output.kind == nomad::OutputKind::Release && !output.servo_position,
               "telemetry fault never commands steering");
        expect(!r.controller.reset(r.steady, r.ros), "invalid telemetry cannot reset");
    }
    Rig absent; absent.prime(); absent.start(); absent.steady += 0.31; absent.ros += 0.31;
    expect(absent.tick().kind == nomad::OutputKind::Release, "lost telemetry stops");
    Rig owner; owner.prime(); owner.start(); owner.feed();
    expect(owner.tick(false).kind == nomad::OutputKind::Release, "lost exclusive driver stops");
    for (int kind = 0; kind < 3; ++kind) {
        Rig r; r.prime(); r.start();
        if (kind == 0) r.steady -= 0.01;
        if (kind == 1) r.ros -= 0.01;
        if (kind == 2) r.steady = std::numeric_limits<double>::quiet_NaN();
        expect(r.tick().kind == nomad::OutputKind::Release && r.controller.state() == nomad::TrialState::Fault,
               "invalid or reversed clock stops");
    }
}

void testStationaryAndReplays() {
    Rig moving; moving.prime(); moving.feed(0.05, 51);
    expect(!moving.start(), "moving ERPM prevents start");
    moving.feed();
    expect(!moving.start(), "stationary hold restarts after movement");
    moving.prime();
    expect(moving.start(), "fresh stationary hold permits start");

    Rig gap; gap.prime(); gap.feed(0.31);
    expect(!gap.start(), "telemetry gap resets stationary hold");

    Rig replay; replay.feed(); const double stamp = replay.ros;
    for (int i = 0; i < 14; ++i) {
        replay.steady += 0.05; replay.ros += 0.05;
        replay.controller.observe({stamp, 0, 0, 0, 16.3, 0}, replay.steady, replay.ros);
    }
    expect(!replay.controller.healthy(replay.steady, replay.ros) && !replay.start(),
           "duplicate telemetry cannot renew receipt freshness or prove stationary hold");
    replay.prime();
    expect(replay.start(), "new telemetry recovers before first trial");
    replay.controller.stop("operator", true);
    replay.feed(0.05, 100);
    expect(!replay.controller.reset(replay.steady, replay.ros), "moving raw feedback cannot reset");
    replay.prime();
    expect(replay.controller.reset(replay.steady, replay.ros), "new stationary hold allows explicit reset");
    expect(!replay.controller.manualActive() && replay.tick().kind == nomad::OutputKind::Release,
           "reset alone never resumes the motor");
}

void testOptionBounds() {
    const std::vector<std::function<void(nomad::RawOptions&)>> invalid = {
        [](auto& o) { o.motor_max_erpm = 1851; }, [](auto& o) { o.motor_max_erpm = 0; },
        [](auto& o) { o.overspeed_erpm = 2501; }, [](auto& o) { o.overspeed_erpm = o.motor_max_erpm; },
        [](auto& o) { o.servo_min = -0.001; }, [](auto& o) { o.servo_max = 1.001; },
        [](auto& o) { o.servo_center = 0.52; }, [](auto& o) { o.servo_min = o.servo_max; },
        [](auto& o) { o.max_duration_s = 3.01; }, [](auto& o) { o.max_duration_s = 0; },
        [](auto& o) { o.telemetry_timeout_s = 0.31; }, [](auto& o) { o.manual_timeout_s = 2.01; },
        [](auto& o) { o.max_tick_gap_s = 0.21; }, [](auto& o) { o.stationary_hold_s = 0.19; },
        [](auto& o) { o.stationary_erpm = 51; }, [](auto& o) { o.current_limit_a = 31; },
        [](auto& o) { o.motor_ramp_erpm_s = 1201; }, [](auto& o) { o.servo_rate_s = 0.0401; },
        [](auto& o) { o.motor_deceleration_erpm_s = 2401; },
        [](auto& o) { o.motor_deceleration_erpm_s = 0; },
        [](auto& o) { o.stall_timeout_s = 2.01; }, [](auto& o) { o.stall_timeout_s = 0.49; },
        [](auto& o) { o.stall_command_erpm = 101; },
        [](auto& o) { o.stall_feedback_erpm = o.stall_command_erpm + 1.0; },
        [](auto& o) { o.manual_servo_rate_s = 0.4001; }, [](auto& o) { o.manual_servo_rate_s = 0.0; },
        [](auto& o) { o.servo_center = std::numeric_limits<double>::quiet_NaN(); }
    };
    for (const auto& change : invalid) {
        auto options = enabled(); change(options); bool rejected = false;
        try { Rig invalid_rig(options); } catch (const std::invalid_argument&) { rejected = true; }
        expect(rejected, "unsafe or nonsensical commissioning option refused");
    }
    Rig r; r.prime();
    expect(!r.start(nomad::RawAxis::Motor, 1000, 0.5, 3.01), "requested duration cannot exceed configured cap");
    expect(!r.start(nomad::RawAxis::Motor, 1000, 0.5, 0), "zero duration is invalid");
}

// 실제 장치 없이 시간·heartbeat·두 축의 출력 제한과 연속 조작을 검사합니다.
void testContinuousManual() {
    auto options = enabled(); options.servo_min = 0.40; options.servo_max = 0.60;
    Rig r(options); r.prime();
    expect(!r.start(nomad::RawAxis::Manual, 1, 0.5, 0), "continuous START must start at zero ERPM");
    expect(!r.start(nomad::RawAxis::Manual, 0, 0.5, 1), "continuous START must explicitly select duration zero");
    expect(!r.start(nomad::RawAxis::Manual, 0, 0.5, std::numeric_limits<double>::quiet_NaN()),
           "continuous duration cannot be nonfinite");
    expect(!r.controller.ownsOutput(), "invalid continuous starts do not acquire output");
    expect(r.start(nomad::RawAxis::Manual, 0, 0.5, 0), "explicit continuous START accepted");
    const auto initial = r.tick();
    expect(initial.motor_erpm == 0 && initial.servo_position == 0.5, "continuous starts at zero motor target");
    double previous_erpm = 0.0, previous_servo = 0.5;
    bool valid = true, simultaneous = false, responsive = false;
    for (int i = 0; i < 100; ++i) {
        r.feed();
        const double erpm = i < 50 ? 600 : 0;
        const double servo = i < 50 ? 0.60 : 0.40;
        valid = r.update(nomad::RawAxis::Manual, erpm, servo) && valid;
        const auto output = r.tick();
        valid = valid && output.kind == nomad::OutputKind::Drive && output.servo_position &&
            output.motor_erpm >= 0 && output.motor_erpm <= 600 &&
            (erpm == 0 ? output.motor_erpm == 0.0 : std::abs(output.motor_erpm - previous_erpm) <= 60.0 + 1e-8);
        if (output.servo_position) {
            valid = valid && *output.servo_position >= 0.40 && *output.servo_position <= 0.60 &&
                std::abs(*output.servo_position - previous_servo) <= 0.020 + 1e-9;
            if (output.motor_erpm > 0 && *output.servo_position > 0.5) simultaneous = true;
            if (i == 59) responsive = std::abs(*output.servo_position - 0.40) < 1e-9;
            previous_servo = *output.servo_position;
        }
        previous_erpm = output.motor_erpm;
    }
    expect(valid && simultaneous, "continuous session updates bounded motor and servo outputs together");
    expect(responsive, "continuous full-range servo command sweep takes half a second");
    expect(r.controller.continuousManualActive() && r.controller.remaining(r.steady) == 0,
           "fresh continuous control stays RUNNING past the three-second trial limit");
    expect(r.controller.commandedMotorErpm() == 0 &&
           std::abs(r.controller.commandedServoPosition() - 0.4) < 1e-9,
           "reported commands match final outputs without claiming physical feedback");
    r.controller.stop("operator STOP");
    const auto stopped = r.tick();
    expect(stopped.kind == nomad::OutputKind::Release && stopped.motor_erpm == 0 && !stopped.servo_position,
           "continuous STOP releases motor without centering servo");
    expect(!r.update(nomad::RawAxis::Manual, 600, 0.6) && !r.controller.continuousManualActive(),
           "late continuous update cannot restart a stopped session");
    expect(r.controller.reset(r.steady, r.ros), "continuous STOP requires explicit stationary reset");
    expect(!r.controller.manualActive() && !r.tick().servo_position, "reset does not resume continuous outputs");
    expect(r.start(nomad::RawAxis::Manual, 0, 0.4, 0), "new continuous session starts at zero after reset");
    expect(r.tick().servo_position == 0.4, "new session preserves previous servo command");
}

void testContinuousFaults() {
    for (int fault = 0; fault < 6; ++fault) {
        Rig r; r.prime();
        expect(r.start(nomad::RawAxis::Manual, 0, 0.5, 0), "continuous fault fixture starts");
        if (fault == 0) {
            for (int i = 0; i < 8; ++i) { r.feed(); r.tick(); }
        } else if (fault == 1) {
            r.feed(0.21);
            expect(!r.update(nomad::RawAxis::Manual, 100, 0.5), "late update cannot conceal continuous timer stall");
        } else if (fault == 2) {
            r.steady += 0.31; r.ros += 0.31; r.tick();
        } else if (fault == 3) {
            r.feed(); r.tick(false);
        } else if (fault == 4) {
            r.feed(); r.update(nomad::RawAxis::Manual, 1851, 0.5);
        } else {
            r.feed(); r.update(nomad::RawAxis::Manual, 0, 0.52);
        }
        expect(r.controller.state() == nomad::TrialState::Fault, "continuous fault remains latched");
        r.feed();
        expect(!r.update(nomad::RawAxis::Manual, 0, 0.5), "fresh update cannot revive continuous fault");
        const auto output = r.tick();
        expect(output.kind == nomad::OutputKind::Release && !output.servo_position && output.motor_erpm == 0,
               "continuous fault emits only motor release");
    }
}

void testHalfRadianCandidateEnvelope() {
    auto options = enabled(); options.servo_min = 0.375; options.servo_max = 0.625;
    Rig r(options); r.prime();
    expect(r.start(nomad::RawAxis::Manual, 0, 0.5, 0), "half-radian candidate envelope accepts explicit manual start");
    for (const double target : {0.625, 0.375}) {
        bool bounded = true;
        for (int i = 0; i < 35; ++i) {
            r.feed(0.02); r.update(nomad::RawAxis::Manual, 0, target);
            const auto output = r.tick();
            bounded = bounded && output.servo_position && *output.servo_position >= 0.375 &&
                *output.servo_position <= 0.625 && output.motor_erpm == 0.0;
        }
        expect(bounded && std::abs(r.controller.commandedServoPosition() - target) < 1e-9,
               "expanded endpoint is reachable with bounded slew and motor released");
    }
    r.feed();
    expect(!r.update(nomad::RawAxis::Manual, 0, 0.626) && !r.tick().servo_position,
           "new envelope still faults beyond its explicit candidate limit");
}

// 다음 후보 범위만 허용하며 기존 좁은 설정도 여전히 자신의 한계를 적용합니다.
void testNextCandidateEnvelope() {
    auto options = enabled(); options.servo_min = 0.3625; options.servo_max = 0.6375;
    Rig r(options); r.prime();
    expect(r.start(nomad::RawAxis::Manual, 0, 0.5, 0), "0.55-radian candidate servo envelope accepts zero-motor start");
    for (const double target : {0.6375, 0.3625}) {
        bool bounded = true;
        double previous = r.controller.commandedServoPosition();
        for (int i = 0; i < 36; ++i) {
            r.feed(0.02);
            bounded = r.update(nomad::RawAxis::Manual, 0, target) && bounded;
            const auto output = r.tick();
            bounded = bounded && output.servo_position && *output.servo_position >= 0.3625 &&
                *output.servo_position <= 0.6375 && output.motor_erpm == 0.0;
            if (output.servo_position) {
                bounded = bounded && std::abs(*output.servo_position - previous) <= 0.008 + 1e-9;
                previous = *output.servo_position;
            }
        }
        expect(bounded && std::abs(r.controller.commandedServoPosition() - target) < 1e-9,
               "new candidate endpoint reaches exactly with unchanged slew limit and zero motor");
    }
    for (const double invalid_target : {0.3624, 0.6376}) {
        Rig invalid(options); invalid.prime(); invalid.start(nomad::RawAxis::Manual, 0, 0.5, 0);
        invalid.feed();
        expect(!invalid.update(nomad::RawAxis::Manual, 0, invalid_target) &&
               invalid.controller.state() == nomad::TrialState::Fault && !invalid.tick().servo_position,
               "both targets just outside next candidate bounds fault without servo output");
    }
}

// 정체 판정은 실제 출력 명령부터 시작하고 회전 확인·0 목표에서 재설정됩니다.
void testStallWatchdog() {
    auto options = enabled(); options.stall_watchdog_enabled = true; options.motor_ramp_erpm_s = 100.0;
    options.stall_command_erpm = 100.0;
    Rig r(options); r.prime(); r.start(nomad::RawAxis::Manual, 0, 0.5, 0);
    bool active = true;
    for (int i = 0; i < 58; ++i) {
        r.feed(); r.update(nomad::RawAxis::Manual, 200, 0.5); r.tick();
        active = active && r.controller.manualActive();
    }
    expect(active, "watchdog excludes initial acceleration below 100 ERPM from two-second stall budget");
    for (int i = 0; i < 5; ++i) { r.feed(); r.update(nomad::RawAxis::Manual, 200, 0.5); r.tick(); }
    expect(r.controller.state() == nomad::TrialState::Fault &&
           r.controller.reason().find("stalled") != std::string::npos,
           "two seconds of commanded rotation without feedback faults");
    expect(r.tick().kind == nomad::OutputKind::Release && !r.tick().servo_position &&
           !r.update(nomad::RawAxis::Manual, 0, 0.5), "stalled motor releases both channels and cannot be renewed");

    options.motor_ramp_erpm_s = 1200.0;
    Rig moving(options); moving.prime(); moving.start(nomad::RawAxis::Manual, 0, 0.5, 0);
    for (int cycle = 0; cycle < 3; ++cycle) {
        for (int i = 0; i < 28; ++i) {
            moving.feed(); moving.update(nomad::RawAxis::Manual, 200, 0.5); moving.tick();
        }
        moving.feed(0.05, 100); moving.update(nomad::RawAxis::Manual, 200, 0.5); moving.tick();
    }
    expect(moving.controller.manualActive(), "measured rotation resets an intermittent startup stall interval");
    moving.feed(); moving.update(nomad::RawAxis::Manual, 0, 0.5);
    expect(moving.tick().motor_erpm == 0.0, "zero target clears output and pending stall interval");
    for (int i = 0; i < 30; ++i) {
        moving.feed(); moving.update(nomad::RawAxis::Manual, 200, 0.5); moving.tick();
    }
    expect(moving.controller.manualActive(), "new positive target gets a fresh startup interval after zero");
    for (int i = 0; i < 80; ++i) {
        moving.feed(); moving.update(nomad::RawAxis::Manual, 50, 0.5); moving.tick();
    }
    expect(moving.controller.manualActive(), "target below configured watchdog command threshold is not a false stall");

    auto low_options = enabled(); low_options.stall_watchdog_enabled = true;
    Rig low(low_options); low.prime(); low.start(nomad::RawAxis::Manual, 0, 0.5, 0);
    for (int i = 0; i < 45; ++i) {
        low.feed(); low.update(nomad::RawAxis::Manual, 50, 0.5); low.tick();
    }
    expect(low.controller.state() == nomad::TrialState::Fault &&
           low.controller.reason().find("stalled") != std::string::npos,
           "default watchdog covers the lowest 50 ERPM GUI command without observed rotation");
}

// Wi-Fi 지연 허용과 명령 발행 시각 기준의 만료를 분리하여 검사합니다.
void testWifiIssuedTimeLease() {
    auto options = enabled(); options.manual_timeout_s = 2.0;
    expect(enabled().manual_timeout_s == 0.35, "default raw heartbeat lease remains strict 350ms");
    Rig r(options); r.prime();
    expect(r.start(nomad::RawAxis::Manual, 0, 0.5, 0), "Wi-Fi continuous manual starts explicitly at zero");
    for (int i = 0; i < 65; ++i) { r.feed(0.02); r.tick(); }
    const double issued_at = r.steady - 1.2;
    expect(r.controller.update(nomad::RawAxis::Manual, 0, 0.5, r.steady, r.ros, 1.2),
           "1.2-second delayed command accepted with fresh local telemetry and ticks");
    for (int i = 0; i < 39; ++i) { r.feed(0.02); r.tick(); }
    expect(r.controller.manualActive(), "issued-time lease remains active just before two seconds");
    for (int i = 0; i < 3 && r.controller.manualActive(); ++i) { r.feed(0.02); r.tick(); }
    expect(r.controller.state() == nomad::TrialState::Fault && r.steady - issued_at <= 2.020001 &&
           r.controller.reason() == "raw GUI command timeout",
           "delayed command cannot extend output beyond two seconds plus one tick from issue");
    expect(!r.controller.update(nomad::RawAxis::Manual, 0, 0.5, r.steady, r.ros, 0.0),
           "fresh or queued update cannot revive an expired Wi-Fi session");

    Rig delayed_start(options); delayed_start.prime();
    expect(delayed_start.controller.start(nomad::RawAxis::Manual, 0, 0.5, 0,
               delayed_start.steady, delayed_start.ros, true, 1.2), "delayed zero START uses issue-time lease too");
    for (int i = 0; i < 42; ++i) { delayed_start.feed(0.02); delayed_start.tick(); }
    expect(!delayed_start.controller.manualActive(), "delayed START receives only the unexpired lease remainder");

    for (const double age : {1.50001, -0.05001, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
        Rig invalid(options); invalid.prime();
        expect(!invalid.controller.start(nomad::RawAxis::Manual, 0, 0.5, 0,
                   invalid.steady, invalid.ros, true, age), "out-of-policy command ages cannot START");
        invalid.start(nomad::RawAxis::Manual, 0, 0.5, 0);
        expect(!invalid.controller.update(nomad::RawAxis::Manual, 0, 0.5, invalid.steady, invalid.ros, age),
               "out-of-policy command ages cannot UPDATE");
    }
    Rig monotonic(options); monotonic.prime(); monotonic.start(nomad::RawAxis::Manual, 0, 0.5, 0);
    for (int i = 0; i < 20; ++i) { monotonic.feed(0.02); monotonic.tick(); }
    monotonic.update(nomad::RawAxis::Manual, 0, 0.5);
    for (int i = 0; i < 5; ++i) { monotonic.feed(0.02); monotonic.tick(); }
    expect(monotonic.controller.update(nomad::RawAxis::Manual, 0, 0.5, monotonic.steady, monotonic.ros, 0.4),
           "older issue-time update may be accepted without extending the latest issue lease");
    for (int i = 0; i < 85; ++i) { monotonic.feed(0.02); monotonic.tick(); }
    expect(monotonic.controller.manualActive(), "accepted issue-time watermark never moves backward");
    for (int i = 0; i < 12; ++i) { monotonic.feed(0.02); monotonic.tick(); }
    expect(!monotonic.controller.manualActive(), "older queued update never renews lease from its receipt time");

    for (const auto axis : {nomad::RawAxis::Motor, nomad::RawAxis::Steering}) {
        Rig short_trial(options); short_trial.prime();
        expect(!short_trial.controller.start(axis, 0, 0.5, 2.0, short_trial.steady, short_trial.ros, true, 0.31),
               "short-axis command age remains limited to 300ms under Wi-Fi profile");
        short_trial.start(axis, 0, 0.5, 2.0);
        for (int i = 0; i < 18; ++i) { short_trial.feed(0.02); short_trial.tick(); }
        expect(!short_trial.controller.manualActive() && short_trial.controller.reason() == "raw GUI command timeout",
               "short-axis heartbeat lease remains 350ms under Wi-Fi profile");
    }
    Rig telemetry(options); telemetry.prime(); telemetry.start(nomad::RawAxis::Manual, 0, 0.5, 0);
    telemetry.steady += 0.31; telemetry.ros += 0.31; telemetry.tick();
    expect(telemetry.controller.reason() == "raw telemetry timeout or invalid sample",
           "Wi-Fi profile keeps local telemetry timeout at 300ms");
    Rig timer(options); timer.prime(); timer.start(nomad::RawAxis::Manual, 0, 0.5, 0);
    timer.feed(0.21); timer.tick();
    expect(timer.controller.reason() == "raw control timer stalled", "Wi-Fi profile keeps control tick gap at 200ms");
}
void testAdjustableSteeringRange() {
    auto direct_options=enabled();direct_options.servo_min=0;direct_options.servo_max=1;
    Rig direct(direct_options);direct.prime();
    expect(direct.start(nomad::RawAxis::Manual,0,.5,0),"direct servo manual starts at center");
    for(double value:{0.,.5,1.}){direct.feed();expect(direct.update(nomad::RawAxis::Manual,0,value),"direct 0 to 1 servo target accepted");expect(direct.tick().motor_erpm==0,"direct steering retains zero speed");}
    Rig r;r.prime();const auto before=r.controller.commandedServoPosition();
    expect(r.controller.setSteeringTestSpan(.5,r.steady,r.ros),"stationary operator may extend steering-only range to protocol bounds");
    expect(r.controller.steeringTestMin()==0&&r.controller.steeringTestMax()==1&&r.controller.commandedServoPosition()==before&&
        r.tick().kind==nomad::OutputKind::None,"changing range sends no output and never changes current command");
    for(double bad:{0.0,.501,std::numeric_limits<double>::quiet_NaN()})
        expect(!r.controller.setSteeringTestSpan(bad,r.steady,r.ros),"invalid span rejected");
    expect(!r.start(nomad::RawAxis::Manual,0,.8,0),"wide limit does not widen ordinary manual controls");
    expect(!r.start(nomad::RawAxis::Steering,1,.8,3),"wide steering trial still rejects nonzero motor");
    expect(r.start(nomad::RawAxis::Steering,0,.8,3),"wide steering-only target allowed after explicit limit");
    expect(!r.controller.setSteeringTestSpan(.4,r.steady,r.ros),"running trial cannot change limit");
    for(int i=0;i<20;++i){r.feed();expect(r.update(nomad::RawAxis::Steering,0,.8),"steering heartbeat accepted");
        const auto out=r.tick();expect(out.motor_erpm==0&&out.servo_position&&*out.servo_position<=before+.04*(i+1)*.05+1e-8,"expanded test retains motor zero and slow servo slew");}
    r.controller.stop("operator stop");
    expect(r.controller.outsideDriveRange(),"outside ordinary range reported");
    expect(!r.controller.setSteeringTestSpan(.005,r.steady,r.ros),"shrinking past current command rejected without centering");
    expect(r.controller.reset(r.steady,r.ros)&&!r.start(nomad::RawAxis::Manual,0,.5,0),"drive remains blocked until operator returns steering");
    expect(r.start(nomad::RawAxis::Steering,0,.5,3),"steering-only return remains available");
    r.controller.stop("operator stop");r.steady+=1;r.ros+=1;
    expect(!r.controller.setSteeringTestSpan(.5,r.steady,r.ros),"stale telemetry cannot change range");
}

void testPlanningContinuation() {
    Rig r; r.prime();
    expect(r.start(nomad::RawAxis::Manual, 0, .5, 0), "planning fixture starts at zero normally");
    r.feed(.05, 600);
    r.update(nomad::RawAxis::Manual, 930, .51); r.tick();
    const double servo_before = r.controller.commandedServoPosition();
    r.controller.stop("LPP pause");
    r.feed(.05, 600);
    expect(!r.controller.stationary(r.steady, r.ros) &&
           !r.controller.reset(r.steady, r.ros) &&
           !r.start(nomad::RawAxis::Manual, 0, .5, 0),
           "moving pause still rejects ordinary stationary reset and manual start");
    expect(r.controller.resumePlanning(930, .49, r.steady, r.ros, true),
           "authorized planning continuation accepts fresh moving telemetry");
    expect(r.controller.continuousManualActive() && r.controller.ownsOutput() &&
           r.controller.commandedMotorErpm() == 600 && r.controller.targetMotorErpm() == 930 &&
           r.controller.commandedServoPosition() == servo_before,
           "continuation seeds measured rotation and preserves last servo command");
    const auto immediate = r.tick();
    expect(immediate.kind == nomad::OutputKind::Drive && immediate.motor_erpm == 600 &&
           immediate.servo_position && *immediate.servo_position == servo_before,
           "first continuation tick adds no invented elapsed ramp or servo jump");
    r.feed(.05, 620); r.update(nomad::RawAxis::Manual, 930, .49);
    const auto next = r.tick();
    expect(std::abs(next.motor_erpm - 660) < 1e-8 && next.servo_position &&
           std::abs(*next.servo_position-servo_before) <= .020000001,
           "continued command retains original motor and manual servo slew bounds");
    r.controller.stop("LPP pause"); r.feed(.05, 1200);
    expect(r.controller.resumePlanning(500, .5, r.steady, r.ros, true) &&
           r.controller.commandedMotorErpm() == 500,
           "measured seed is clamped to the lower new target");
    r.controller.stop("LPP pause"); r.feed(.05, 400);
    expect(r.controller.resumePlanning(0, r.controller.commandedServoPosition(), r.steady, r.ros, true) &&
           r.controller.commandedMotorErpm() == 0 && r.tick().motor_erpm == 0,
           "zero continuation target remains motor release rather than coast feedback command");

    for (double measured : {-50., -1., 0.}) {
        r.controller.stop("LPP pause"); r.feed(.05, measured);
        expect(r.controller.resumePlanning(930, .5, r.steady, r.ros, true) &&
               r.controller.commandedMotorErpm() == 0,
               "small signed stationary feedback never seeds reverse output");
    }
    r.controller.stop("LPP pause"); r.feed(.05, -50.01);
    expect(!r.controller.resumePlanning(930, .5, r.steady, r.ros, true) &&
           r.controller.state() == nomad::TrialState::Stopped && r.controller.commandedMotorErpm() == 0,
           "actual reverse rotation beyond stationary threshold cannot resume planning");
}

void testPlanningContinuationGates() {
    for (int state = 0; state < 4; ++state) {
        Rig r(state == 0 ? nomad::RawOptions{} : enabled()); r.prime();
        if (state == 2) r.start(nomad::RawAxis::Manual, 0, .5, 0);
        if (state == 3) r.controller.stop("hardware fault", true);
        const auto before = r.controller.state();
        expect(!r.controller.resumePlanning(930, .5, r.steady, r.ros, true) &&
               r.controller.state() == before,
               "monitor idle running and fault cannot enter continuation");
    }
    for (int gate = 0; gate < 2; ++gate) {
        auto options = enabled();
        if (gate == 0) options.setup_acknowledged = false;
        else options.hardware_timeout_verified = false;
        Rig r(options); r.prime(); r.controller.stop("LPP pause");
        expect(!r.controller.resumePlanning(930, .5, r.steady, r.ros, true) && !r.controller.ownsOutput(),
               "continuation retains setup and hardware timeout prerequisites");
    }
    Rig missing; missing.controller.stop("LPP pause");
    expect(!missing.controller.resumePlanning(930, .5, missing.steady, missing.ros, true),
           "continuation needs actual telemetry");
    Rig r; r.prime(); r.controller.stop("LPP pause"); r.feed(.05, 400);
    expect(!r.controller.resumePlanning(930, .5, r.steady, r.ros, false),
           "continuation still requires exclusive driver");
    for (const auto& target : std::vector<std::pair<double,double>>{{-1.,.5},{1850.1,.5},
              {930.,.489},{930.,.511},{std::numeric_limits<double>::quiet_NaN(),.5},
              {930.,std::numeric_limits<double>::infinity()}}) {
        expect(!r.controller.resumePlanning(target.first, target.second, r.steady, r.ros, true) &&
               r.controller.state() == nomad::TrialState::Stopped && !r.controller.ownsOutput(),
               "invalid continuation targets never gain output ownership");
    }
    r.steady += .31; r.ros += .31;
    expect(!r.controller.resumePlanning(930, .5, r.steady, r.ros, true),
           "stale moving telemetry cannot resume");
    r.feed(.05, 400);
    r.controller.observe({r.ros+.01,400,30.01,0,16.3,0},r.steady+.01,r.ros+.01);
    r.steady += .01; r.ros += .01;
    expect(!r.controller.resumePlanning(930, .5, r.steady, r.ros, true),
           "overcurrent sample remains invalid for continuation");

    Rig steering; steering.prime();
    expect(steering.controller.setSteeringTestSpan(.5,steering.steady,steering.ros) &&
           steering.start(nomad::RawAxis::Steering,0,.8,3), "wide steering-only fixture starts");
    for (int i=0; i<20; ++i) { steering.feed(); steering.update(nomad::RawAxis::Steering,0,.8); steering.tick(); }
    steering.controller.stop("LPP pause"); steering.feed(.05,400);
    expect(steering.controller.outsideDriveRange() &&
           !steering.controller.resumePlanning(930,.5,steering.steady,steering.ros,true),
           "continuation cannot bypass return from steering-only outer range");

    Rig lease; lease.prime(); lease.controller.stop("LPP pause"); lease.feed(.05,400);
    expect(lease.controller.resumePlanning(930,.5,lease.steady,lease.ros,true), "continuation starts fresh lease");
    for(int i=0;i<8;++i){lease.feed(.05,400);lease.tick();}
    expect(lease.controller.state()==nomad::TrialState::Fault && lease.controller.reason()=="raw GUI command timeout",
           "continuation does not disable normal command lease expiry");
}

void testPlanningMinimumDriveOutput() {
    Rig manual; manual.prime();
    expect(manual.start(nomad::RawAxis::Manual,0,.5,0), "legacy manual floor fixture starts");
    manual.feed(); manual.update(nomad::RawAxis::Manual,550,.5);
    expect(std::abs(manual.tick().motor_erpm-60.)<1e-8,
           "original three-argument tick preserves manual low-RPM ramp");

    Rig r; r.prime();
    expect(r.start(nomad::RawAxis::Manual,0,.5,0), "planning minimum fixture starts normally");
    r.feed(); r.update(nomad::RawAxis::Manual,200,.5);
    auto output=r.controller.tick(r.steady,r.ros,true,550);
    expect(output.kind==nomad::OutputKind::Drive && output.motor_erpm==550 &&
           r.controller.commandedMotorErpm()==550,
           "first planning send and reported command skip directly to 550 ERPM");
    for(int i=0;i<5;++i) {
        r.feed(); r.update(nomad::RawAxis::Manual,200,.5);
        expect(r.controller.tick(r.steady,r.ros,true,550).motor_erpm==550,
               "positive low request retains continuous 550 ERPM floor");
    }
    double previous=550;
    for(int i=0;i<3;++i) {
        r.feed(); r.update(nomad::RawAxis::Manual,700,.5);
        output=r.controller.tick(r.steady,r.ros,true,550);
        expect(output.motor_erpm>=previous && output.motor_erpm<=700 &&
               output.motor_erpm-previous<=60.+1e-8,
               "above-floor acceleration retains original rise-rate bound");
        previous=output.motor_erpm;
    }
    expect(std::abs(previous-700)<1e-8,"higher 700 ERPM request remains 700 rather than being pinned to floor");

    r.feed(); r.update(nomad::RawAxis::Manual,650,.5);
    expect(std::abs(r.controller.tick(r.steady,r.ros,true,550).motor_erpm-650)<1e-8,
           "650 ERPM startup-assist request remains available above floor");
    r.feed(); r.update(nomad::RawAxis::Manual,550,.5);
    expect(r.controller.tick(r.steady,r.ros,true,550).motor_erpm==550,
           "startup-assist end returns to continuous 550 ERPM output");

    for(int i=0;i<3;++i) {
        r.feed(); r.update(nomad::RawAxis::Manual,200,.5);
        output=r.controller.tick(r.steady,r.ros,true,0);
    }
    expect(std::abs(output.motor_erpm-200)<1e-8 && r.controller.commandedMotorErpm()<550,
           "explicit zero floor preserves overspeed reduction below 550 ERPM");
    r.feed(); r.update(nomad::RawAxis::Manual,550,.5);
    expect(r.controller.tick(r.steady,r.ros,true,550).motor_erpm==550,
           "valid motion recovers its floor after overspeed reduction ends");
    r.feed(); r.update(nomad::RawAxis::Manual,0,.5);
    expect(r.controller.tick(r.steady,r.ros,true,550).motor_erpm==0 &&
           r.controller.commandedMotorErpm()==0,
           "zero target immediately remains zero despite configured planning floor");
    r.feed(); r.update(nomad::RawAxis::Manual,550,.5);
    r.controller.tick(r.steady,r.ros,true,550);
    r.controller.stop("operator STOP");
    output=r.controller.tick(r.steady,r.ros,true,550);
    expect(output.kind==nomad::OutputKind::Release && output.motor_erpm==0,
           "operator STOP releases output without reapplying floor");

    Rig ownership; ownership.prime(); ownership.start(nomad::RawAxis::Manual,0,.5,0);
    ownership.feed(); ownership.update(nomad::RawAxis::Manual,550,.5);
    ownership.controller.tick(ownership.steady,ownership.ros,true,550);
    ownership.feed(); ownership.update(nomad::RawAxis::Manual,550,.5);
    output=ownership.controller.tick(ownership.steady,ownership.ros,false,550);
    expect(output.kind==nomad::OutputKind::Release && output.motor_erpm==0 &&
           ownership.controller.state()==nomad::TrialState::Fault,
           "driver ownership loss faults and releases even with positive floor");

    for(double floor : {-1.,1850.1,std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()}) {
        Rig invalid; invalid.prime(); invalid.start(nomad::RawAxis::Manual,0,.5,0);
        invalid.feed(); invalid.update(nomad::RawAxis::Manual,550,.5);
        output=invalid.controller.tick(invalid.steady,invalid.ros,true,floor);
        expect(output.kind==nomad::OutputKind::Release && output.motor_erpm==0 &&
               invalid.controller.state()==nomad::TrialState::Fault &&
               invalid.controller.reason()=="invalid planning minimum drive ERPM",
               "invalid planning floor faults without nonzero motor output");
    }
}
void testPlanningTransientRecovery() {
    Rig r; r.prime(); r.controller.setPlanningRecovery(true);
    expect(r.start(nomad::RawAxis::Manual,0,.5,0),"headless transient fixture starts");
    r.tick();r.feed(.02);r.update(nomad::RawAxis::Manual,1000,.5);r.tick();
    r.steady+=.02;r.ros+=.02;
    r.controller.observe({r.ros-.31,0,0,0,16.3,0},r.steady,r.ros);
    expect(r.controller.state()==nomad::TrialState::Stopped&&r.controller.planningWaitActive(),"old telemetry becomes headless wait without fault");
    expect(r.tick().kind==nomad::OutputKind::Release&&r.controller.commandedMotorErpm()==0,"transient wait releases output");
    r.feed(.02);
    expect(r.controller.resumePlanning(1000,.5,r.steady,r.ros,true),"fresh telemetry and fresh planner command resume without reset");
    r.tick();r.feed(.02);
    expect(r.controller.pausePlanning("module unavailable",r.steady,r.ros),"normal module pause remains recoverable");
    r.steady+=.02;r.ros+=.02;r.controller.observe({r.ros,0,0,0,16.3,1},r.steady,r.ros);
    expect(r.controller.state()==nomad::TrialState::Fault&&!r.controller.planningWaitActive(),"real VESC fault during wait stays latched");
    r.feed(.02);
    expect(!r.controller.resumePlanning(1000,.5,r.steady,r.ros,true),"fresh telemetry cannot clear physical fault");
    Rig timer;timer.prime();timer.controller.setPlanningRecovery(true);
    expect(timer.start(nomad::RawAxis::Manual,0,.5,0),"planning timer fixture starts");
    timer.tick();timer.feed(.21);
    expect(!timer.controller.pausePlanning("module unavailable",timer.steady,timer.ros)&&timer.controller.state()==nomad::TrialState::Fault,"real control timer stall still latches on pause");
    Rig gap;gap.prime();gap.controller.setPlanningRecovery(true);
    expect(gap.start(nomad::RawAxis::Manual,0,.5,0),"telemetry gap fixture starts");gap.tick();
    for(int j=0;j<7;++j){gap.steady+=.05;gap.ros+=.05;gap.controller.update(nomad::RawAxis::Manual,1000,.5,gap.steady,gap.ros);gap.tick();}
    expect(gap.controller.state()==nomad::TrialState::Stopped&&gap.controller.planningWaitActive(),"missing telemetry releases to wait in headless mode");
    gap.feed(.02);
    expect(gap.controller.resumePlanning(1000,.5,gap.steady,gap.ros,true),"telemetry gap recovers without resetting controller");
}
} // namespace

int main() {
    testPlanningTransientRecovery();
    testStartupAndGates();
    testMotorAxisAndRamp();
    testSteeringAxisAndContinuity();
    testExpandedSteeringEnvelope();
    testExplicitOuterSteeringEnvelope();
    testDeadlinesAndLateUpdates();
    testAxisAndInvalidTargets();
    testTelemetryAndClockFaults();
    testStationaryAndReplays();
    testOptionBounds();
    testContinuousManual();
    testContinuousFaults();
    testHalfRadianCandidateEnvelope();
    testNextCandidateEnvelope();
    testStallWatchdog();
    testWifiIssuedTimeLease();
    testAdjustableSteeringRange();
    testPlanningContinuation();
    testPlanningContinuationGates();
    testPlanningMinimumDriveOutput();
    std::cout << checks << " raw commissioning checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

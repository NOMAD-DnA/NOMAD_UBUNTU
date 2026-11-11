#include "nomad_control_cpp/trial_controller.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}
nomad::Calibration calibration() {
    return {6000.0, 0.0, -0.6, 0.52, 1, -1, 0.0, 0.4, -0.2, 0.2, 0.4, 0.64, -3000.0, 3000.0};
}
nomad::TrialOptions enabled() {
    nomad::TrialOptions o;
    o.actuation_enabled = o.calibration_verified = o.hardware_timeout_verified = true;
    o.target_speed_mps = 0.2;
    o.duration_s = 1.0;
    return o;
}

// 실제 시간·ROS 없이 telemetry, 타이머, 장애 시점을 정확하게 재현합니다.
struct Rig {
    nomad::TrialController controller;
    double wall = 10.0, ros = 100.0;
    explicit Rig(nomad::TrialOptions o = enabled()) : controller(calibration(), o) {}
    void feed(double dt = 0.05, double erpm = 0.0, int fault = 0, double current = 0.0) {
        wall += dt; ros += dt;
        controller.observe({ros, erpm, current, 0.0, 12.0, fault}, wall, ros);
    }
    void prime() { for (int i = 0; i < 13; ++i) feed(); }
    bool start(bool exclusive = true) { return controller.start(wall, ros, exclusive); }
    nomad::TrialOutput tick(bool exclusive = true) { return controller.tick(wall, ros, exclusive); }
};

void testMonitorAndGates() {
    Rig monitor(nomad::TrialOptions{});
    monitor.prime();
    expect(monitor.controller.healthy(monitor.wall, monitor.ros), "monitor consumes telemetry");
    expect(!monitor.start(), "monitor rejects start");
    monitor.controller.stop("test");
    expect(monitor.tick().kind == nomad::OutputKind::None, "monitor never emits even stop output");
    for (int gate = 0; gate < 2; ++gate) {
        auto o = enabled();
        if (gate == 0) o.calibration_verified = false;
        else o.hardware_timeout_verified = false;
        Rig r(o); r.prime();
        expect(!r.start(), "verification gate");
        expect(r.tick().kind == nomad::OutputKind::None, "refused start has no output");
    }
    Rig r;
    expect(!r.start(), "no telemetry refuses start");
    r.feed();
    expect(!r.start(), "one sample cannot prove stationary hold");
    r.prime();
    expect(!r.start(false), "competing or absent driver refuses start");
    expect(r.start(), "healthy exclusive stationary start");
    expect(!r.start(), "repeat start cannot extend trial");
}

void testTimedTrialAndStop() {
    Rig r; r.prime(); expect(r.start(), "timed trial start");
    const double start = r.wall;
    bool moved = false;
    double previous_erpm = 0.0;
    while (r.wall - start < 1.1) {
        r.feed();
        const auto output = r.tick();
        if (output.kind == nomad::OutputKind::Drive) {
            expect(output.motor_erpm - previous_erpm <= 60.0 + 1e-6, "acceleration bounded");
            moved = moved || output.motor_erpm > 0.0;
            previous_erpm = output.motor_erpm;
        }
    }
    expect(moved, "trial reaches nonzero command");
    expect(r.controller.state() == nomad::TrialState::Stopped, "duration stops trial");
    expect(r.tick().kind == nomad::OutputKind::Release, "timeout sends release");
    expect(!r.start(), "timeout does not automatically rearm");
    expect(r.controller.reset(r.wall, r.ros), "stationary explicit reset");
    expect(r.tick().kind == nomad::OutputKind::Release, "reset keeps release until next start");
    expect(r.start(), "new explicit start after reset");
    r.controller.stop("operator");
    expect(r.tick().kind == nomad::OutputKind::Release, "operator stop releases immediately");
    expect(!r.start(), "operator stop remains latched");
}

void testFaults() {
    for (int fault = 0; fault < 8; ++fault) {
        Rig r; r.prime(); r.start();
        if (fault == 0) { r.wall += 0.31; r.ros += 0.31; }
        if (fault == 1) r.feed(0.05, 0.0, 4);
        if (fault == 2) r.feed(0.05, 0.0, 0, 31.0);
        if (fault == 3) r.feed(0.05, std::numeric_limits<double>::quiet_NaN());
        if (fault == 4) { r.wall += 0.21; r.ros += 0.21; r.feed(0.0); }
        if (fault == 5) { r.wall += 0.05; r.ros -= 1.0; }
        if (fault == 6) { r.wall -= 0.1; }
        const auto output = r.tick(fault != 7);
        expect(output.kind == nomad::OutputKind::Release, "fault sends release");
        expect(r.controller.state() == nomad::TrialState::Fault, "fault is latched");
        expect(!r.start(), "fault cannot restart directly");
    }
}

void testFreshnessAndStationary() {
    Rig r; r.prime();
    const double old_stamp = r.ros;
    for (int i = 0; i < 8; ++i) {
        r.wall += 0.05; r.ros += 0.05;
        r.controller.observe({old_stamp, 0.0, 0.0, 0.0, 12.0, 0}, r.wall, r.ros);
    }
    expect(!r.start(), "repeated telemetry stamp cannot renew lease");
    r.prime();
    r.feed(0.05, -100.0);
    expect(!r.start(), "moving vehicle cannot start");
    r.feed();
    expect(!r.start(), "stationary hold resets after movement");
    r.prime();
    r.controller.observe({r.ros + 1.0, 0.0, 0.0, 0.0, 12.0, 0}, r.wall, r.ros);
    expect(!r.start(), "future timestamp rejected");
    r.prime();
    r.wall += 0.4; r.ros += 0.4; r.feed(0.0);
    expect(!r.start(), "telemetry gap resets stationary hold");
}

void testSteeringAndOptions() {
    auto o = enabled(); o.target_steering_rad = 0.15; o.duration_s = 2.0;
    Rig r(o); r.prime(); r.start();
    const double started = r.wall;
    bool moved = false;
    for (int i = 0; i < 22; ++i) {
        r.feed();
        const auto output = r.tick();
        if (r.wall - started < 0.75) expect(output.motor_erpm == 0.0, "motor waits for steering ramp and settle");
        moved = moved || output.motor_erpm > 0.0;
    }
    expect(moved, "motor may move after steering preparation");
    for (int kind = 0; kind < 5; ++kind) {
        auto bad = enabled();
        if (kind == 0) bad.duration_s = 5.01;
        if (kind == 1) bad.duration_s = std::numeric_limits<double>::infinity();
        if (kind == 2) bad.telemetry_timeout_s = 10.0;
        if (kind == 3) bad.acceleration_mps2 = 0.0;
        if (kind == 4) bad.target_speed_mps = 0.9;
        bool rejected = false;
        try { Rig invalid(bad); } catch (const std::invalid_argument&) { rejected = true; }
        expect(rejected, "invalid trial options rejected");
    }
}
void testManualLease() {
    Rig r; r.prime();
    expect(!r.controller.startManual(0.1, 0, 5.1, r.wall, r.ros, true), "manual duration cap");
    expect(r.controller.startManual(0.1, 0, 1.0, r.wall, r.ros, true), "manual start");
    const double start = r.wall;
    while (r.wall - start < 0.95) {
        r.feed();
        expect(r.controller.updateManual(0.15, 0.05, r.wall, r.ros), "fresh manual update");
        r.tick();
    }
    r.feed(0.1);
    expect(!r.controller.updateManual(0.15, 0, r.wall, r.ros), "updates never extend deadline");
    expect(r.tick().kind == nomad::OutputKind::Release, "manual deadline releases");
    expect(r.controller.reset(r.wall, r.ros), "manual reset");
    expect(r.controller.startManual(0.1, 0, 2.0, r.wall, r.ros, true), "new manual trial");
    for (int i = 0; i < 8; ++i) { r.feed(); r.tick(); }
    expect(r.controller.state() == nomad::TrialState::Fault, "missing GUI heartbeat latches fault");
    expect(!r.controller.updateManual(0.1, 0, r.wall, r.ros), "late update cannot revive fault");
    r.controller.reset(r.wall, r.ros);
    r.controller.startManual(0.1, 0, 2.0, r.wall, r.ros, true);
    r.feed(0.36);
    expect(!r.controller.updateManual(0.1, 0, r.wall, r.ros), "late update rejected before next timer");
    r.controller.reset(r.wall, r.ros);
    r.controller.startManual(0.1, 0, 2.0, r.wall, r.ros, true);
    expect(!r.controller.updateManual(99, 0, r.wall, r.ros), "invalid manual target rejected");
    expect(r.tick().kind == nomad::OutputKind::Release, "invalid target releases");
    Rig monitor(nomad::TrialOptions{}); monitor.prime();
    expect(!monitor.controller.startManual(0.1, 0, 1, monitor.wall, monitor.ros, true), "manual cannot bypass monitor");
}

// 정지 시 서보를 유지한 뒤 재시작해도 마지막 명령에서 변화율 제한을 이어가야 합니다.
void testSteeringRestartContinuity() {
    auto options = enabled();
    options.target_speed_mps = 0.0;
    options.target_steering_rad = 0.15;
    options.duration_s = 2.0;
    Rig r(options); r.prime();
    expect(r.start(), "steering-only initial trial starts");
    nomad::TrialOutput last;
    for (int i = 0; i < 12; ++i) { r.feed(); last = r.tick(); }
    expect(last.kind == nomad::OutputKind::Drive && last.motor_erpm == 0.0,
           "steering trial has no motor drive");
    const auto first_target = nomad::convertTarget(0.0, 0.15, calibration());
    expect(std::abs(last.servo_position - first_target.value->servo_position) < 1e-9,
           "first trial reaches steering target");
    r.controller.stop("operator stop");
    r.feed();
    expect(r.tick().kind == nomad::OutputKind::Release, "stopped trial releases motor");
    expect(r.controller.reset(r.wall, r.ros), "steering trial resets while stationary");
    expect(r.controller.startManual(0.0, -0.15, 2.0, r.wall, r.ros, true),
           "opposite steering trial starts after reset");

    const double max_servo_step = std::abs(calibration().steering_gain) * options.steering_rate_radps * 0.05;
    for (int i = 0; i < 8; ++i) {
        r.feed();
        expect(r.controller.updateManual(0.0, -0.15, r.wall, r.ros), "restart heartbeat accepted");
        const auto next = r.tick();
        expect(next.kind == nomad::OutputKind::Drive && next.motor_erpm == 0.0,
               "restarted steering trial has no motor drive");
        expect(std::abs(next.servo_position - last.servo_position) <= max_servo_step + 1e-9,
               "restart preserves servo slew limit from the last command");
        expect(next.servo_position > last.servo_position, "restart slews toward opposite target");
        last = next;
    }
}
} // namespace

int main() {
    {
        Rig limited; limited.prime();
        limited.controller.setStabilityLimits(.01,.05);
        expect(limited.start(),"stability-limited trial starts");
        double prior=0;
        for(int i=0;i<16;++i) {
            limited.feed(); const auto output=limited.tick();
            expect(output.motor_erpm<=60.0+1e-7,"stability speed cap reaches actuator mapping");
            expect(output.motor_erpm-prior<=15.0+1e-7,"stability acceleration cap reaches actuator mapping");
            prior=output.motor_erpm;
        }
        bool invalid=false;
        try { limited.controller.setStabilityLimits(std::numeric_limits<double>::quiet_NaN(),.1); }
        catch(const std::invalid_argument&) { invalid=true; }
        expect(invalid,"nonfinite stability limit rejected");
    }
    testMonitorAndGates();
    testTimedTrialAndStop();
    testFaults();
    testFreshnessAndStationary();
    testSteeringAndOptions();
    testManualLease();
    testSteeringRestartContinuity();
    std::cout << checks << " trial checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

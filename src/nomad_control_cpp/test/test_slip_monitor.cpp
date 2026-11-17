// 2026-11-17 slip / stuck detection (4a, display only). ROS-free.
#include "nomad_control_cpp/slip_monitor.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace std;
using namespace nomad;
static int checks = 0;
static void expect(bool v, const char* why) { ++checks; if (!v) throw runtime_error(why); }

// 50 Hz telemetry, 20 Hz pose, 50 Hz control tick. The body moves along +x at body_mps (negative = backward).
struct Run {
    SlipMonitor m;
    double t = 0.0, x = 0.0, next_pose = 0.0;
    bool status = true, poses = true;
    double command = 1000.0;      // commanded ERPM while driving
    bool spin = false;            // telemetry ERPM swings 0..erpm (sensorless spin on leaves, 11-16)
    int events = 0;
    optional<SlipEvent> last;
    void step(double seconds, double body_mps, double erpm, double iq, bool driving, bool holding = false) {
        for (double end = t + seconds; t < end - 1e-9;) {
            t += 0.02;
            x += body_mps * 0.02;
            m.onTelemetry(t, spin ? (static_cast<int>(t * 50.0 + 0.5) % 2 ? erpm : 0.0) : erpm, iq);
            if (poses && t >= next_pose - 1e-9) { m.onPose(t, x, 0.0, 0.0, t); next_pose = t + 0.05; }
            m.onStatus(status, t);
            auto e = m.tick(t, driving ? command : 0.0, holding);
            if (e) { ++events; last = e; }
        }
    }
};

static void stuck() {
    Run r;
    r.step(1.0, 0.27, 1000, 5, true);
    expect(r.m.state() == SlipState::Ok && abs(*r.m.bodySpeed() - 0.27) < 0.01, "normal drive: ok, body 0.27 m/s");
    r.step(0.9, 0.0, 800, 30, true);   // wheels spin, body still
    expect(r.m.state() == SlipState::Ok, "not stuck before 1 s (plus the speed window)");
    r.step(0.8, 0.0, 800, 30, true);
    expect(r.m.state() == SlipState::Stuck && r.events == 1 && r.last->state == SlipState::Stuck &&
           r.last->iq_a > 25 && r.last->erpm > 700, "wheels at 800 ERPM / 30 A with the body still: stuck");
    r.step(1.0, 0.25, 1000, 8, true);
    expect(r.m.state() == SlipState::Ok && r.events == 2 && r.last->ended == SlipState::Stuck && r.last->duration_s > 0.5,
           "moving again ends the stuck event");
    Run low;
    low.step(1.0, 0.27, 1000, 5, true);
    low.step(3.0, 0.0, 1000, 8, true);  // body still but low current: a stop under way / blocked lightly, not spinning
    expect(low.m.state() == SlipState::Ok && low.events == 0, "low current with the body still is not stuck");
    // 11-16 Umyeonsan: the measured ERPM swung 0-1000 while the wheels spun at 25-49 A (review 11-17: a mean-ERPM
    // gate missed every replayed case).
    Run swing;
    swing.step(1.0, 0.27, 1000, 5, true);
    swing.spin = true;
    swing.step(2.0, 0.0, 1000, 35, true);
    expect(swing.m.state() == SlipState::Stuck && swing.last->erpm < 700, "ERPM swinging 0-1000 at 35 A with the body still: stuck");
    Run blocked;  // a hard block: wheels near 0 ERPM at high current, commanded forward
    blocked.step(1.0, 0.27, 1000, 5, true);
    blocked.step(2.0, 0.0, 20, 40, true);
    expect(blocked.m.state() == SlipState::Stuck && blocked.last->erpm < 100, "blocked push (ERPM ~0, 40 A): stuck, ERPM reported");
    Run idle;
    idle.step(3.0, 0.0, 800, 30, false);  // not driving (e.g. manual bench numbers): never stuck
    expect(idle.m.state() == SlipState::Ok && idle.events == 0, "no stuck event while not driving forward");
}

static void backslide() {
    Run r;
    r.step(1.0, 0.27, 1000, 6, true);      // forward reference
    r.step(1.0, 0.0, 0, 0, false, true);   // stopped, holding
    expect(r.m.state() == SlipState::Ok, "holding still: ok");
    r.step(0.8, -0.10, 0, 0, false, true); // the hold slips: body backward at 0.1 m/s (0.5 s window + 0.3 s)
    expect(r.m.state() == SlipState::BackSlide && r.last->state == SlipState::BackSlide, "backward 0.1 m/s while holding");
    r.step(1.0, -0.10, 0, 0, false, true);
    r.step(1.0, 0.0, 0, 0, false, true);
    expect(r.m.state() == SlipState::Ok && r.last->ended == SlipState::BackSlide && r.last->distance_m > 0.08 &&
           r.m.lastBackslideM() == r.last->distance_m, "the slide ends with its distance");
    // A slide while still commanded forward must not become the "forward" reference (review 11-17).
    Run slide;
    slide.step(1.0, 0.27, 1000, 6, true);
    slide.step(1.5, -0.12, 600, 30, true);  // wheels forward, body sliding back
    expect(slide.m.state() == SlipState::BackSlide && slide.m.alongSpeed() && *slide.m.alongSpeed() < -0.05,
           "sliding back while driving forward is a back slide, not a new forward direction");
    Run noref;
    noref.step(1.0, 0.0, 0, 0, false, true);
    noref.step(1.0, -0.10, 0, 0, false, true);
    expect(noref.m.state() == SlipState::Ok && !noref.m.alongSpeed(), "no forward reference yet: no back-slide judgement");
}

static void gating() {
    Run r;
    r.step(1.0, 0.27, 1000, 5, true);
    r.status = false;
    r.step(0.1, 0.27, 1000, 5, true);
    expect(r.m.state() == SlipState::Off && r.m.offReason() == "localization not READY", "status not READY: off");
    r.status = true;
    r.step(0.1, 0.27, 1000, 5, true);
    expect(r.m.state() == SlipState::Ok, "back on with READY");
    r.poses = false;
    r.step(0.4, 0.27, 1000, 5, true);
    expect(r.m.state() == SlipState::Off && r.m.offReason() == "localization pose stale", "pose older than 0.3 s: off");
    // A stuck event that loses the pose ends (no false continuation over Wi-Fi gaps).
    Run s;
    s.step(1.0, 0.27, 1000, 5, true);
    s.step(2.0, 0.0, 800, 30, true);
    expect(s.m.state() == SlipState::Stuck, "stuck before the pose loss");
    s.poses = false;
    s.step(0.4, 0.0, 800, 30, true);
    expect(s.m.state() == SlipState::Off && s.last->ended == SlipState::Stuck, "losing the pose ends the event");
    // A VIO reset (jump) clears the history and the reference.
    Run j;
    j.step(1.0, 0.27, 1000, 5, true);
    j.m.onPose(j.t + 0.05, j.x + 5.0, 0.0, 0.0, j.t + 0.05);
    j.t += 0.05;
    auto e = j.m.tick(j.t, true, false);
    (void)e;
    expect(!j.m.bodySpeed() && j.m.offReason() == "pose jumped (VIO reset)", "after a pose jump: off until a fresh window");
    // A fused-pose correction (0.2 m in one 50 ms step) restarts the speed window but is not motion.
    Run c;
    c.step(1.0, 0.27, 1000, 6, true);
    c.step(1.0, 0.0, 0, 0, false, true);
    c.m.onPose(c.t + 0.05, c.x - 0.2, 0.0, 0.0, c.t + 0.05);
    c.x -= 0.2; c.next_pose = c.t + 0.1;
    c.step(1.0, 0.0, 0, 0, false, true);
    expect(c.events == 0 && c.m.state() == SlipState::Ok, "a 0.2 m pose correction is not a back slide");
    SlipMonitor none;
    expect(none.state() == SlipState::Off && none.offReason() == "no localization pose", "starts off without a pose");
}

static void holdEdges() {
    // The one-sample ERPM spike after the handbrake engages or releases is skipped.
    Run r;
    r.step(1.0, 0.27, 1000, 5, true);
    r.step(0.5, 0.0, 0, 0, false, true);
    // Readings within 0.1 s after the hold ends (the one-sample ERPM spike) are not counted.
    r.m.tick(r.t + 0.01, 1000.0, false);
    r.m.onTelemetry(r.t + 0.03, 360.0, 25.0);
    r.m.onTelemetry(r.t + 0.09, 300.0, 25.0);
    expect(r.m.telemetrySamples() == 0, "samples inside the hold-edge window are skipped");
    r.m.onTelemetry(r.t + 0.13, 900.0, 5.0);
    expect(r.m.telemetrySamples() == 1, "samples after the edge window count again");
    expect(string(slipStateName(SlipState::BackSlide)) == "backslide" && string(slipStateName(SlipState::Off)) == "off",
           "state names");
}

int main() {
    try {
        stuck();
        backslide();
        gating();
        holdEdges();
        cout << checks << " slip monitor checks PASS\n";
        return 0;
    } catch (const exception& e) { cerr << "FAIL: " << e.what() << "\n"; return 1; }
}

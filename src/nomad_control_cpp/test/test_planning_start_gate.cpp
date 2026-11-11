#include "nomad_control_cpp/planning_start_gate.hpp"
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    int checks = 0, failures = 0;
    auto check = [&](bool value, const char* description) {
        ++checks;
        if (!value) { ++failures; std::cerr << "FAIL: " << description << '\n'; }
    };
    auto feed = [](nomad::PlanningStartGate& gate, double start, int count, double erpm = 744) {
        for (int i = 0; i < count; ++i) gate.update(start + i * .05, erpm, .15);
    };

    nomad::PlanningStartGate gate;
    check(!gate.ready(10), "no command cannot start");
    feed(gate, 10, 15);
    check(!gate.ready(10.7), "short valid burst cannot start");
    gate.update(10.75, 744, .15);
    check(gate.ready(10.75), "continuous commands complete hold");
    check(!gate.ready(10.901), "query cannot extend last command lease");
    gate.update(10.95, 744, .15);
    check(!gate.ready(10.95), "gap restarts qualification even after previous readiness");

    gate.reset(11, true);
    feed(gate, 11.05, 16);
    check(!gate.ready(11.8), "hold does not bypass independent restart cooldown");
    feed(gate, 11.85, 5);
    check(gate.ready(12.05), "cooldown and hold both satisfied");
    gate.reset(12.06, true);
    feed(gate, 12.1, 12);
    gate.reset(12.7, true);
    feed(gate, 12.75, 16);
    check(!gate.ready(13.5), "repeated STOP renews cooldown and resets hold");
    feed(gate, 13.55, 5);
    check(gate.ready(13.75), "stable stream eventually resumes after last STOP");

    nomad::PlanningStartGate low;
    for (int i = 0; i < 40; ++i)
        check(!low.update(20 + i * .05, 372, .15), "low command stays inhibited without being boosted");
    check(!low.ready(21.95), "low-speed stream cannot satisfy start hold");
    check(low.reason(21.95).find("below minimum") != std::string::npos, "low-speed reason is explicit");
    feed(low, 22, 16, 550);
    check(low.ready(22.75), "minimum threshold itself can qualify");
    low.update(22.8, 549.99, .15);
    check(!low.ready(22.8), "one below-threshold command resets qualification");

    nomad::PlanningStartGate steering;
    feed(steering, 30, 16, 0);
    check(steering.ready(30.75), "zero-speed steering-only stream is supported");
    steering.update(30.8, 744, .15);
    check(!steering.ready(30.8), "steering-only stream cannot prequalify motor startup");
    feed(steering, 30.85, 15, 744);
    check(steering.ready(31.55), "new motion stream independently qualifies");
    steering.update(31.6, 0, .15);
    check(!steering.ready(31.6), "motion-to-zero stream also starts a distinct qualification");

    nomad::PlanningStartGate capped(0, 0, 0);
    capped.update(40, 1, .5);
    check(!capped.ready(40.351), "lease cap is 350ms even when command lease is 500ms");
    capped.update(40.5, 1, .02);
    check(!capped.ready(40.521), "short remaining lease is respected");
    check(!capped.update(40.4, 1, .15), "reversed time invalidates update");
    check(!capped.ready(40.4), "reversed update cannot grant output");
    const double bad[] = {NAN, INFINITY, -INFINITY};
    for (double value : bad) {
        check(!capped.update(41, value, .15), "nonfinite target rejected");
        check(!capped.update(41, 1, value), "nonfinite lease rejected");
        check(!capped.update(value, 1, .15), "nonfinite clock rejected");
        check(!capped.ready(value), "nonfinite clock cannot query readiness");
    }
    for (double lease : {0., -.1, .501}) check(!capped.update(42, 1, lease), "invalid lease rejected");
    check(!capped.update(42, -1, .15), "reverse target rejected");
    check(!capped.update(-1, 1, .15) && !capped.ready(-1), "negative steady time rejected");
    nomad::PlanningStartGate jitter(.3, 0, 550);
    for (double time : {50.,50.04,50.13,50.17,50.27,50.31}) jitter.update(time,744,.15);
    check(jitter.ready(50.31), "healthy irregular arrivals preserve qualification");
    jitter.update(50.47,744,.15);
    check(!jitter.ready(50.47), "one gap beyond last lease breaks irregular stream");
    for (int field = 0; field < 3; ++field) {
        bool refused = false;
        try { nomad::PlanningStartGate invalid(field == 0 ? NAN : .75,
                                             field == 1 ? -1. : 1., field == 2 ? INFINITY : 550.); }
        catch (const std::invalid_argument&) { refused = true; }
        check(refused, "invalid configuration rejected");
    }
    std::cout << checks << " planning start gate checks, " << failures << " failures\n";
    return failures ? 1 : 0;
}

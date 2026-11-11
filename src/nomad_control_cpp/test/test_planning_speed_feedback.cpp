#include "nomad_control_cpp/planning_speed_feedback.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    int checks = 0, failures = 0;
    auto check = [&](bool value, const char* description) {
        ++checks;
        if (!value) { ++failures; std::cerr << "FAIL: " << description << '\n'; }
    };
    auto near = [](double a, double b) { return std::abs(a - b) < 1e-9; };

    nomad::PlanningSpeedFeedback feedback;
    check(near(feedback.update(930.5, 0., .02), 930.5),
          "stationary start passes the request without an extra startup ramp");
    const double reduced = feedback.update(930.5, 1438., .02);
    check(near(reduced, 742.5), "configured-ceiling overspeed reduces the target instead of latching");
    check(near(feedback.update(930.5, 1400., .02), 766.5),
          "recovery rise is bounded to 24 ERPM per 20 ms tick");
    double recovered = 0.;
    for (int i = 0; i < 20; ++i) recovered = feedback.update(930.5, 930.5, .02);
    check(near(recovered, 930.5), "measured recovery automatically restores the request without reset");
    check(near(feedback.update(206., 200., .02), 206.),
          "low request is accepted without a minimum 550 ERPM gate or boost");
    check(near(feedback.update(206., 0., .02), 206.), "stationary feedback does not boost a low request");

    check(feedback.update(0., 1438., .02) == 0., "STOP is immediate even while rotating");
    check(near(feedback.update(930.5, 0., .02), 930.5), "STOP clears previous governor recovery state");
    feedback.reset();
    check(near(feedback.update(206., 0., .02), 206.), "explicit reset also clears previous output");

    nomad::PlanningSpeedFeedback quiet;
    for (double measured : {899., 930., 950., 979., 980., 1199., 1200., 1250.})
        check(near(quiet.update(930., measured, .01), 930.), "only configured ceiling plus deadband can reduce target");
    check(near(quiet.update(930., 1251., .01), 929.), "outside-ceiling-deadband reduction is continuous");
    check(near(quiet.update(930., 1250., .01), 930.), "small ceiling noise recovers without a latch");
    check(near(quiet.update(930., -1438., .01), 742.), "reverse measured magnitude also reduces target");

    nomad::PlanningSpeedFeedback low_request;
    for (double measured : {0., -8., 5., 199., 206., 238., 403., 762., 1006., 1199., -1199., 1200., 1250.})
        check(near(low_request.update(206.8, measured, .02), 206.8),
              "noisy feedback above a low request but below the ceiling is left to the VESC PID");
    check(near(low_request.update(206.8, 1438., .02), 18.8),
          "1438 ERPM subtracts only the 188 ERPM excess beyond the configured ceiling deadband");
    check(near(low_request.update(206.8, 1006., .02), 42.8),
          "ceiling recovery preserves the configured rise limit for a low request");
    low_request.reset();
    check(near(low_request.update(930.5, 1438., .02), 742.5),
          "the same measured ceiling excess uses the new requested cap without a request-relative cut");

    nomad::PlanningSpeedFeedback capped;
    check(near(capped.update(1800., 0., .02), 1200.), "soft request ceiling is applied before feedback");
    check(near(capped.update(1800., 1300., .02), 1150.), "overspeed is relative to the soft ceiling");
    check(capped.update(1800., 5000., .02) == 0., "large excess can reduce target to zero without a latch");
    check(near(capped.update(1800., 0., .02), 24.), "zero governor output can recover automatically");
    // The independent extreme-speed fault guard remains in the caller.

    nomad::PlanningSpeedFeedback alternating;
    for (int i = 0; i < 30; ++i) {
        check(alternating.update(0., 0., .02) == 0., "repeated STOP remains immediate");
        check(near(alternating.update(206., 0., .02), 206.),
              "alternating valid/STOP commands do not accumulate a restart cooldown");
    }

    nomad::PlanningSpeedFeedback proportional(1200., 50., .5, 100.);
    check(near(proportional.update(900., 1350., .01), 850.), "configured proportional ceiling excess reduction");
    check(near(proportional.update(900., 900., .2), 870.), "configured recovery uses elapsed time");
    check(near(proportional.update(650., 0., .01), 650.), "decreased request is applied immediately");

    const double invalid[] = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()};
    auto rejects = [&](double request, double measured, double dt) {
        try { proportional.update(request, measured, dt); }
        catch (const std::invalid_argument&) { return true; }
        return false;
    };
    for (double bad : invalid) {
        check(rejects(bad, 900., .02), "nonfinite requested ERPM rejected");
        check(rejects(900., bad, .02), "nonfinite measured ERPM rejected");
        check(rejects(900., 900., bad), "nonfinite elapsed time rejected");
    }
    check(rejects(-1., 900., .02), "negative request rejected");
    check(rejects(900., 900., 0.), "zero elapsed time rejected");
    check(rejects(900., 900., -.01), "negative elapsed time rejected");
    check(near(proportional.update(900., 900., .1), 660.), "invalid inputs do not mutate recovery state");

    auto rejects_config = [](double limit, double band, double gain, double recovery) {
        try { nomad::PlanningSpeedFeedback bad(limit, band, gain, recovery); }
        catch (const std::invalid_argument&) { return true; }
        return false;
    };
    for (double bad : invalid) {
        check(rejects_config(bad, 50., 1., 1200.), "nonfinite limit rejected");
        check(rejects_config(1200., bad, 1., 1200.), "nonfinite deadband rejected");
        check(rejects_config(1200., 50., bad, 1200.), "nonfinite gain rejected");
        check(rejects_config(1200., 50., 1., bad), "nonfinite recovery rejected");
    }
    check(rejects_config(0., 50., 1., 1200.), "zero limit rejected");
    check(rejects_config(1200., -1., 1., 1200.), "negative deadband rejected");
    check(rejects_config(1200., 50., 0., 1200.), "zero gain rejected");
    check(rejects_config(1200., 50., 1., 0.), "zero recovery rejected");

    nomad::PlanningSpeedFeedback extreme(1200., 0., std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max());
    check(extreme.update(1200., std::numeric_limits<double>::max(), .01) == 0.,
          "finite extreme measurements and gain saturate reduction without NaN");
    check(near(extreme.update(1200., 0., std::numeric_limits<double>::max()), 1200.),
          "large elapsed time recovery never exceeds the request");

    std::cout << checks << " planning speed feedback checks, " << failures << " failures\n";
    return failures ? 1 : 0;
}

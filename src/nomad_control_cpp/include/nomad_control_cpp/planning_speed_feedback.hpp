#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace nomad {

// VESC owns its RPM PID. This supervisor only reduces the requested ERPM when
// measured rotation exceeds the configured soft ceiling plus hysteresis.
// Tracking a lower requested speed remains the VESC PID's job. This supervisor
// never adds startup
// torque, a minimum target, or another integral velocity controller. ERPM is
// motor feedback, not a calibrated measurement of vehicle speed in m/s.
class PlanningSpeedFeedback {
public:
    explicit PlanningSpeedFeedback(double limit_erpm = 1200.,
                                   double hysteresis_erpm = 50.,
                                   double gain = 1.,
                                   double recovery_erpm_s = 1200.)
        : limit_erpm_(limit_erpm), hysteresis_erpm_(hysteresis_erpm),
          gain_(gain), recovery_erpm_s_(recovery_erpm_s) {
        if (!finitePositive(limit_erpm_) || !finiteNonnegative(hysteresis_erpm_) ||
            !finitePositive(gain_) || !finitePositive(recovery_erpm_s_))
            throw std::invalid_argument("invalid planning speed feedback configuration");
    }

    // requested_erpm is a nonnegative forward magnitude. measured_erpm can have
    // either sign: the feedback magnitude is used so unexpected reverse rotation
    // cannot evade overspeed reduction. The caller still owns feedback freshness,
    // hardware faults, explicit STOP, and the independent extreme-speed guard.
    // Invalid input throws without changing the previous valid output.
    double update(double requested_erpm, double measured_erpm, double dt_s) {
        if (!finiteNonnegative(requested_erpm) || !std::isfinite(measured_erpm) ||
            !finitePositive(dt_s))
            throw std::invalid_argument("invalid planning speed feedback input");

        if (requested_erpm == 0.) {
            reset();
            return 0.;
        }

        const double cap = std::min(requested_erpm, limit_erpm_);
        const double measured = std::abs(measured_erpm);
        // Use the configured ceiling, not a small requested speed, as the
        // overspeed threshold. Subtract to avoid limit + hysteresis overflow.
        const double excess = measured > limit_erpm_
            ? std::max(0., (measured - limit_erpm_) - hysteresis_erpm_) : 0.;
        const double desired = std::max(0., cap - gain_ * excess);

        double output = desired;
        if (last_output_ && desired > *last_output_) {
            const double increase = std::min(desired - *last_output_, recovery_erpm_s_ * dt_s);
            output = std::min(desired, *last_output_ + increase);
        }
        last_output_ = output;
        return output;
    }

    // A new valid motion request starts normally after an explicit STOP/reset.
    void reset() noexcept { last_output_.reset(); }

private:
    static bool finiteNonnegative(double value) {
        return std::isfinite(value) && value >= 0.;
    }
    static bool finitePositive(double value) {
        return std::isfinite(value) && value > 0.;
    }

    double limit_erpm_, hysteresis_erpm_, gain_, recovery_erpm_s_;
    std::optional<double> last_output_;
};

} // namespace nomad

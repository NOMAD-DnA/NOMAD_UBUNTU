#pragma once
// ESC (VESC MOSFET NTC) temperature guard, 2026-11-12 user decision:
//   >=65 C warn, 70..80 C derate output 100% -> 30%, >=80 C stop,
//   resume only after the hottest valid sensor is below 70 C.
// Invalid readings (non-finite, <=0, >150 C) are ignored; with no valid
// reading the guard only reports "unknown" and never blocks driving.
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>

namespace nomad_thermal {

struct ThermalConfig {
  double warn_c = 65., derate_start_c = 70., stop_c = 80., resume_c = 70., min_scale = .3;
};

class ThermalGuard {
 public:
  explicit ThermalGuard(ThermalConfig c = {}) : c_(c) {
    if (!(c_.warn_c <= c_.derate_start_c && c_.derate_start_c < c_.stop_c && c_.resume_c < c_.stop_c &&
          c_.min_scale > 0 && c_.min_scale <= 1))
      throw std::invalid_argument("invalid thermal guard thresholds");
  }
  // Returns true when the stop state changed (for one-shot logging).
  bool update(std::initializer_list<double> readings) {
    double hottest = -1;
    for (double t : readings)
      if (std::isfinite(t) && t > 0. && t <= 150.) hottest = std::max(hottest, t);
    valid_ = hottest > 0;
    temp_c_ = valid_ ? hottest : 0.;
    const bool before = stopped_;
    if (valid_) {
      if (temp_c_ >= c_.stop_c) stopped_ = true;
      else if (stopped_ && temp_c_ < c_.resume_c) stopped_ = false;
    }
    return before != stopped_;
  }
  bool valid() const { return valid_; }
  bool stopped() const { return stopped_; }
  bool warning() const { return valid_ && temp_c_ >= c_.warn_c; }
  double temp_c() const { return temp_c_; }
  // Output scale for duty-type controllers: 1.0 below derate_start, linear to
  // min_scale at stop_c, 0 when stopped.
  double scale() const {
    if (stopped_) return 0.;
    if (!valid_ || temp_c_ < c_.derate_start_c) return 1.;
    const double f = (temp_c_ - c_.derate_start_c) / (c_.stop_c - c_.derate_start_c);
    return std::clamp(1. - (1. - c_.min_scale) * f, c_.min_scale, 1.);
  }
  std::string state() const {
    if (!valid_) return "unknown";
    if (stopped_) return "stop";
    if (temp_c_ >= c_.derate_start_c) return "derate";
    if (temp_c_ >= c_.warn_c) return "warn";
    return "ok";
  }
  const ThermalConfig& config() const { return c_; }

 private:
  ThermalConfig c_;
  bool valid_ = false, stopped_ = false;
  double temp_c_ = 0.;
};

}  // namespace nomad_thermal

#pragma once
// Shared helpers split out of the original single core.cpp (2026-11-13).
// Bodies are unchanged; only `static` became `inline` so several packages share them.
#include "nomad_costmap/core.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace nomad {
  inline constexpr double pi = 3.14159265358979323846;
  inline void require(bool b, const std::string& s)
  {
    if (!b)
      throw std::runtime_error(s);
  }
  inline Json num(double v)
  {
    return std::isfinite(v)?Json(v):Json(nullptr);
  }
  inline double val(const Json& v)
  {
    return v.is_null()?nan:v.get<double>();
  }
  inline void count(Json& c, const char* k, uint64_t n = 1)
  {
    c[k] = c.value(k, uint64_t(0)) + n;
  }
  inline Key key_of(const V3& v, double size)
  {
    return {int64_t(std::floor(v.x() / size)), int64_t(std::floor(v.y() / size))};
  }
  inline Eigen::Vector2d center(Key k, double size)
  {
    return {(k.first + .5) * size, (k.second + .5) * size};
  }
  // 2026-11-16: the uphill/gravity score per degree (k) and the blocking angle (B)
  // come from terrain.json (Config::slope_score_per_deg / slope_block_deg).
  // Defaults 4 and 20 give exactly the former constants. The heading-mode
  // downhill rule (-10..0 deg -> 0, below -10 deg -> 100) is unchanged.
  inline float slope_score_of(double angle, double tilt, double k = 4., double block = 20.)
  {
    float score = std::numeric_limits<float>::quiet_NaN();
    if (std::isfinite(angle)&&std::isfinite(tilt))
    {
      if (angle >= -10&&angle <= 0)
        score = 0;
      if (angle > 0)
        score = float(k * tilt);
      if (angle < -10)
        score = 100;
    }
    if (std::isfinite(tilt)&&tilt >= block)
      score = 100;
    return score;
  }
}

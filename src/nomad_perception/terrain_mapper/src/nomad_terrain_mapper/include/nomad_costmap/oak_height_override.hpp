#pragma once
#include <nomad_costmap/core.hpp>
#include <cmath>
#include <stdexcept>
namespace nomad {
// terrain.json uses centimetres for the GUI; the unchanged core expects metres.
inline void apply_oak_height_override(const Json& terrain, Json& slope) {
  const char* key = "oak_lidar_max_abs_z_difference_cm";
  if (!terrain.contains(key)) return; // Preserve the legacy slope file when omitted.
  const double cm = terrain.at(key).get<double>();
  if (!std::isfinite(cm) || cm < 0.1 || cm > 100.)
    throw std::runtime_error("OAK/LiDAR height difference must be finite and within 0.1..100cm");
  slope["oak_lidar_max_abs_z_difference_m"] = cm / 100.;
}
}

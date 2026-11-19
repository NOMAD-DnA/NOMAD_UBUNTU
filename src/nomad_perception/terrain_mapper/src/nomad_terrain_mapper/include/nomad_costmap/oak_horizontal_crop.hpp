#pragma once
#include <nomad_costmap/core.hpp>
#include <cmath>
#include <stdexcept>

namespace nomad {
// Input and world arrays describe the same capture. Crop both before any accumulation.
inline size_t crop_oak_horizontal(Points& optical, Points& world,
                                 const M4& world_from_vehicle,
                                 const V3& vehicle_forward, double max_distance_m) {
  if (optical.size() != world.size() || !std::isfinite(max_distance_m) ||
      max_distance_m < 0.1 || max_distance_m > 30.0)
    throw std::runtime_error("Invalid OAK horizontal crop inputs/range (0.1..30m)");
  Eigen::Vector2d forward = (world_from_vehicle.block<3,3>(0,0)*vehicle_forward).head<2>();
  if (!world_from_vehicle.allFinite() || !forward.allFinite() || forward.norm() < 1e-8)
    throw std::runtime_error("Invalid vehicle horizontal forward for OAK crop");
  forward.normalize();
  const Eigen::Vector2d origin = world_from_vehicle.block<2,1>(0,3);
  const size_t original_size = world.size();
  size_t kept = 0;
  for (size_t i=0; i<original_size; ++i) {
    if (!world[i].allFinite() || !optical[i].allFinite()) continue;
    const Eigen::Vector2d d = world[i].head<2>() - origin;
    if (d.squaredNorm() > max_distance_m*max_distance_m + 1e-10 || d.dot(forward) < -1e-10) continue;
    world[kept] = world[i]; optical[kept] = optical[i]; ++kept;
  }
  world.resize(kept); optical.resize(kept);
  return original_size-kept;
}
} // namespace nomad

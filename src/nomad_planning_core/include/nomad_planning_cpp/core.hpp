#pragma once
#include "nomad_planning_cpp/types.hpp"
#include <functional>

namespace nomad_planning_cpp {
double wrap(double angle);
double distance(const Pose& a,const Pose& b);
Pose advance(const Pose& pose,double curvature,double distance,double reference_offset);
void validate_grid(const Grid& grid);
void validate_vehicle(const VehicleConfig& vehicle);
double equivalent_steering_limit(const VehicleConfig& vehicle);
Cell world_to_cell(const Grid& grid,double x,double y);
Pose cell_to_world(const Grid& grid,Cell cell);
bool in_bounds(const Grid& grid,Cell cell);
using Gate = std::function<bool(const Pose&,const Pose&)>; // true means blocked
struct CostField {
  Grid grid;
  std::vector<double> weights;
  std::vector<Gate> gates;
  double value(double x,double y) const;
  double segment(const Pose& a,const Pose& b) const;
  double path_cost(const Path& path) const;
};
CostField compose_costs(const Layers& layers,const CostConfig& config=CostConfig{});
CostField occupancy_costs(const Grid& grid,double unknown_penalty=4.5,
                          const std::vector<double>& proximity={});
std::vector<double> obstacle_distances(const Grid& grid);
std::vector<double> blocked_clearances(const Grid& grid);
std::vector<double> proximity_costs(const Grid& grid,double radius=2,double weight=4);
std::vector<double> cluster_density(const Grid& grid,double radius=4,
    double influence_radius=0,double saturation=1);
PlanResult hybrid_astar(const CostField& field,const Pose& start,const Pose& goal,
    const VehicleConfig& vehicle=VehicleConfig{},const SearchConfig& search=SearchConfig{},
    const std::function<bool(Cell)>& corridor={});
}  // namespace nomad_planning_cpp

# Atomic snapshot; all row-major arrays share grid geometry/frame/capture stamp.
# grid >=80 remains blocked; -1 remains unobserved. Its soft values are NOT
# included again in multi-factor mode. Arrays are normalized [0,1]; NaN means
# insufficient evidence, never zero/flat. Global observation memory is retained.
nav_msgs/OccupancyGrid grid
float32[] slope_cost
float32[] terrain_cost

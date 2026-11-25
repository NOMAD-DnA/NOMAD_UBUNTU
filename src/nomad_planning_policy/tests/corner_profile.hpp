#pragma once
#include "nomad_planning_cpp/policy.hpp"
using namespace nomad_planning_cpp;
inline NavigatorConfig profile(){
 NavigatorConfig c;c.vehicle.wheelbase=.335;c.vehicle.joint_limit=1.0813901513620499;c.vehicle.max_steer=.6108652381980153;c.vehicle.reference_offset=-.1675;
 c.vehicle.track=.60;c.vehicle.wheel_radius=.055;c.rpp.speed=.25;c.rpp.unknown_speed=.10;c.search.seconds=.5;c.search.max_expansions=50000;c.corridor.seconds=.25;c.corridor.max_expansions=20000;
 c.corridor.arrival=.25;c.corridor.allow_unknown_transit=true;c.corridor.observation_guard=false;c.corridor.observation_turn_weight=0;
 c.allow_initial_footprint_outside=true;c.guard_half_length=.1;c.guard_half_width=.05;c.guard_center_offset=-.075;
 c.costs.obstacle_weight=16;c.costs.radius=4;c.costs.slope_weight=5;c.costs.terrain_weight=.5;c.costs.unknown_surcharge=9;c.costs.unknown_penalty=9;c.costs.cluster_weight=32;c.costs.cluster_radius=4;c.costs.cluster_influence=.75;c.costs.cluster_saturation=.4;
#ifdef NOMAD_CORNER_ENABLED
 c.costs.clearance_weight=24;c.costs.clearance_radius=.45;c.corridor.corner_aware=true;c.corridor.preferred_clearance=.45;c.corridor.transit_clearance_weight=4;c.footprint_planning=true;
#endif
 return c;
}

# RPP rejection diagnostics

2026-11-24: diagnostics only; candidate ordering, steering limits, occupancy policy,
footprint validation, braking thresholds and first-success selection are unchanged.

`RppResult.rejection_key` contains stable ordered candidate/cause pairs on failure.
`rejection_detail` adds numerical evidence. Successful results leave both empty.
The optional rejection-reason callback describes a failed MotionCheck; it never
replaces that boolean safety decision. Navigator supplies the actual footprint
validator reason or braking speed cap.

The ROS node emits `[LPP_REJECT]` together with `[LPP_PAUSE]` when entering a failure
or changing the candidate cause set. Repeated numerical changes alone do not
create 20 Hz logs. Old rejection details are not emitted as the cause of a new
input/result freshness error. `result_stamp_ns` and `pose` identify the worker
result; `current_local_map_age` is the age at publication, while `map_age_s` in a
BRAKING detail is the age used by the worker's braking calculation.

Groups:
- GEOMETRY_*: behind/degenerate carrot, steering limit, behind route.
- PATH_*: centre path outside map, known obstacle, disallowed unknown, segment
  cost rejection or invalid current cell. Collision checks provide the first
  sampled failing grid cell when available.
- FOOTPRINT:*: exact existing swept validator result, including outside map,
  known obstacle and unobserved cells. This does not claim a collision-cell
  coordinate when the existing validator only provides a reason.
- BRAKING_INSUFFICIENT: computed cap below 0.02 m/s (with map age).
- INPUT_INVALID / MOTION_GUARD_REJECTED: explicit defensive fallback reasons.

The first failing gate is reported for each candidate; checks after it are not
executed. Different candidates can fail for different reasons.

Tests: rpp_rejections_test, existing policy/corner/footprint/result validation
regressions, and the ROS package's lpp_diagnostics_test.

# Waypoint reissue (2026-11-25, default off)

With `corridor.reissue_enabled: true` (unknown-transit profile only), a safe active
route no longer blocks waypoint reselection until arrival. After a new global map
(`stamp_ns` changed), at most once per `reissue_min_interval` (1 s), and once the
vehicle has progressed along the adopted GPP path by
`max(reissue_min_travel 0.2 m, reissue_travel_ratio 0.4 x path length)` or has
`reissue_end_margin` (0.6 m) or less left, a fresh candidate set is computed
without the active-waypoint shortcut. Only candidates scoring better than the
active waypoint by `reissue_margin` (0.15), or the final goal, are tried with
Hybrid A* within `reissue_search_seconds` (0.5 s since 2026-11-25 field review, was 0.3; counted from the reissue start).
On success the new waypoint/path is adopted; otherwise the safe active route and
waypoint are kept unchanged (no reject, no retry flag, no new status text).

Interfaces (ROS topics/messages, status strings, existing public functions) are
unchanged. With the feature off, the corner scenario traces are byte-identical to
the previous code. Test: `waypoint_reissue_test`.

Logging (2026-11-25): every reissue attempt writes one planner.log line,
`[GPP_REISSUE] outcome=switched|kept_no_better|kept_search_failed from=(x,y) to=(x,y)
progress=s/L m active_score=.. best_score=.. candidates=N`, at most once per new map.
`Navigator::reissue_event()` exposes the same data; no ROS interface changed.

# Immediate waypoint handover (2026-11-25, always on)

Reaching an intermediate waypoint (within `corridor.arrival_tolerance`, not the
final goal, with no other invalidation reason) now plans the next waypoint in the
same planning step. Previously the route was cleared and the vehicle stopped with
"waiting for global path retry interval" until `replan_period` (1 s) had elapsed
since the last plan. Obstacle/unknown invalidation and failed searches keep the
`replan_period` backoff unchanged. `replan_period` is now a node parameter
(default 1.0 s, must be > 0). In the corner scenarios the handover stops went from
2-7 per run to 0, and all runs still arrive. Test: `waypoint_handover_test`.

# Obstacle-aware waypoint direction score (2026-11-25, default on)

Field records 2026-11-25 17:16-17:51 (8 of 10 runs): with an obstacle straight between the
start and the goal, the waypoint score's straight-line bearing term (2*error/pi) chose a short
0.4-0.9 m waypoint under the obstacle on the goal side, so the vehicle steered back toward it
for 2-3 s before a waypoint around the obstacle appeared.

`corridor.goal_cost_scoring` (default true, only with `corridor.corner_aware`) adds an
obstacle-aware correction to that term:
- a goal-rooted 8-neighbour grid search gives each cell its remaining distance to the goal;
  cells >= 80 block (cells within 0.3 m of the start excepted), unknown cells cost like free
  cells, cells nearer than `preferred_clearance` cost `1+w*(1-clearance/preferred)^2` with
  `w = corridor.goal_cost_clearance_weight` (default 1);
- a goal outside the map seeds only the map edges facing the goal with straight-line distance
  (seeding every edge let routes "leave" through the back edge and chose points behind the car);
- direction = 2*error/pi + 2/pi*(acos(po/r) - acos(pf/r)), floored at 0, where pf is the
  straight-line progress and po = pf + start detour - candidate detour (detour = this search
  minus the same search without obstacles, which cancels the grid's 8-neighbour anisotropy).
  Without obstacles po == pf and the score is unchanged.

Replay of the 10 recorded start scenes on the same code: every waypoint moved to the obstacle's
open side (e.g. (0.44,1.56) -> (0.81,1.56)), none behind the car. w=5 made a 0.5 m demo passage
cost ~6x and chose a long detour, so the default is 1. Test: `goal_cost_scoring_test`.
Corner simulation (corner_scenarios): all scenarios arrive at 0.08/0.10/0.125/0.15 m; at 0.10 m
right_corner now has 6 stop cycles (0 before) caused by a different early trajectory, 0 at the
other resolutions.

# Relative clearance cost (2026-11-25, `cost.clearance_factor`, node default 2)

Field records 2026-11-25: corner paths ran 0.09-0.18 m (reference point to blocked-cell edge)
from obstacles although clearance parameters were raised. With obstacle_weight 16 over a 4 m
proximity radius and cluster_weight 32, every cell near the course already cost ~30-45, so the
additive clearance term (<= ~23 at 0.16 m) was diluted and a shorter hugging path stayed
cheapest; clearance_radius changes had no effect in replays.

`compose_costs` (core) now applies `w = base*(1+clearance_factor*q) + clearance_weight*q`,
q = (1-d/clearance_radius)^2, to free and unknown cells (blocked stays infinite). Core default 0
keeps the old weights exactly (`clearance_factor_test`); the planner node defaults to 2.
Replay of all 135 GPP replans in the 10 records (goal_cost_scoring on): paths under 0.15 m after
the first 0.3 m 12 -> 3, under 0.20 m 41 -> 29, mean 0.343 -> 0.366 m; adoption 133 -> 132
(one marginal scene where nearly every candidate already failed RPP). Factor 6 gave the same
paths as 2. Corner simulation: all scenarios arrive with 0 stops at 0.10/0.125/0.15 m (the
0.10 m right_corner stops seen with goal_cost_scoring alone disappear).
`corridor.min_clearance` (hard, 0.15 m) is unchanged: narrow passages still pass at that limit.

# Goal approach and GPP feasibility (2026-11-25, records 19:03-19:13)

Field failures: near the goal the vehicle drove to 2 m waypoints elsewhere (goal 0.2-0.26 m
from the end wall, inside the 0.15 m band or its search failed, then the radial term and
reissue moved the waypoint away); RPP could not follow GPP paths drawn at full 35 deg steering
(`GEOMETRY_STEERING_LIMIT` 36-47 deg) and stopped facing obstacles; a waypoint chosen before a
plate was observed was kept and driven into (run 3). Library defaults keep the old behaviour;
the planner node defaults are given in brackets.
- `corridor.final_goal_tolerance` [0.30]: the final-goal Hybrid A* may end within 0.30 m of
  the goal (was 0.75 cell = 0.094 m); a goal inside the min_clearance band / unknown is moved
  to the nearest known-free cell leaving that tolerance.
- `corridor.goal_approach_radius` [2.0]: within 2 m of the goal no reissue; waypoints nearer
  the goal than the vehicle are scored by remaining distance and tried first (goal + 2),
  the rest only fill the remaining attempts (vehicle facing away has to loop).
- `corridor.gpp_steer_ratio` [0.85]: GPP searches at max_steer*0.85 with 60% of the budget;
  only if no candidate succeeds, a second pass at full max_steer uses the rest.
- `corridor.exit_check_length` [0.6]: an intermediate waypoint (new or retained) needs one
  forward 0.6 m arc (5 curvatures at the GPP steering limit) clear of the band; a retained
  waypoint that newly fails is rejected and replanned in the same step.
Checks: `goal_approach_test`; closed-loop runs on the 15 recorded final maps (static map,
start (0,0), vehicle follows the RPP command): arrivals 8 (before goal_cost/clearance_factor)
-> 9 (those two) -> 11 (all), no collisions; corner_scenarios 0 stops at 0.10/0.125/0.15 m.
Known gap: a vehicle already facing away from the goal beside a wall can still stop (replay
of run 2 at 22 s); the new approach behaviour is meant to avoid reaching that state.

# Start-area exemption from the min_clearance band (2026-11-25, node default 0.2 m)

With `corridor.min_clearance` raised to 0.20 (Jetson saved settings, 20:35), run 20:39 stopped
beside the plate end: the vehicle's own cell lay in the band, so GPP answered
`start_blocked_or_outside_map` 40 times. `corridor.start_clearance_exempt` (library 0, node 0.2)
restores the pre-band cells within that radius of the vehicle each planning step; outside it
the band applies unchanged. Closed loop on 22 recorded final maps x2 (min_clearance 0.20):
arrivals 23/44 -> 27-29/44, start_blocked runs 9 -> 0, mean minimum clearance 0.194 -> 0.18-0.19 m.
A 0.4 m radius arrived as often but averaged 0.167 m. Limit: alongside a long wall the vehicle
needs more than 0.2 m of travel to leave the band, so that case can still stop.
Test: `start_exempt_test`.

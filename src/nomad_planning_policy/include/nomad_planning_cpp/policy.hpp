#pragma once
#include "nomad_planning_cpp/core.hpp"
#include <optional>
#include <set>
#include <string>
#include <utility>

namespace nomad_planning_cpp {
struct RppConfig {
  double lookahead=.8,lateral_accel=.35,speed=.5,dt=.1,horizon=1.5;
  int obstacle_threshold=80;
  double unknown_penalty=2.5;
  // Optional speed cap for an unknown current cell. Zero keeps legacy cost
  // regulation; positive values never override other RPP/braking limits.
  double unknown_speed=0;
  bool adaptive_lookahead=true,allow_unknown=false;
  // Ordered explicit candidates. Empty preserves legacy lookahead/adaptive settings.
  std::vector<double> lookahead_candidates;
};
std::vector<double> rpp_lookaheads(const RppConfig& config);
struct RppResult {
  bool success=false;
  Motion motion;
  Path trajectory;
  double score=kInf,lookahead=0;
  // Diagnostics only: stable categorical key and bounded per-candidate evidence.
  std::string rejection_key,rejection_detail;
};
bool finite_pose(const Pose& pose);
bool route_is_behind(const Pose& pose,const Path& path,double lookahead=1,double margin=.2);
Path known_prefix(const Grid& grid,const Pose& start,const Path& reference,bool allow_unknown=false);
using MotionCheck=std::function<bool(const Path&,const Motion&)>;
RppResult regulated_pure_pursuit(const Pose& start,const Path& path,const Grid& grid,
                               const VehicleConfig& vehicle,const RppConfig& config=RppConfig{},const MotionCheck& permitted={},
                               const std::function<std::pair<std::string,std::string>()>& rejection_reason={});
double observed_distance(const Grid& grid,const Pose& pose,double steer,
                         const VehicleConfig& vehicle,double half_length=.30,
                         double half_width=.20,double horizon=2,bool allow_unknown=false,
                         const Pose* initial_footprint=nullptr,double center_offset=0);
double observation_speed_limit(double distance,double latency,double deceleration,double margin);

struct CorridorConfig {
  double horizon=8,inset=.7,arrival=1,seconds=.35;
  int max_expansions=20000;
  double goal_weight=1,length_weight=.15,narrow_weight=2,heading_weight=1;
  double visit_weight=.8,clearance_weight=1,risk_weight=3,risk_scale=50,bearing_weight=1;
  bool incremental=true,reselect_new_blocked=true,observed_goal_lock=true;
  double final_approach_radius=3,continuation_distance=3;
  double reselect_half_width=1.5,reselect_lookahead=3,retained_slope_increase=.2;
  bool retain_safe_path=true,observation_guard=true;
  double observation_turn_weight=80,min_clearance=0;
  // Endpoints remain genuinely known-free in the global grid; unknown cells
  // inside the map may connect them. Confirmed obstacles always remain blocked.
  bool allow_unknown_transit=false;
  double waypoint_radius=1.5;
  bool corner_aware=false;
  double preferred_clearance=.45,transit_clearance_weight=4;
  // 2026-11-25 (corner_aware only): score a waypoint's direction by obstacle-aware
  // progress toward the goal instead of the straight-line bearing alone. The remaining
  // distance per cell comes from a goal-rooted grid search in which known obstacles block
  // and cells nearer than preferred_clearance cost more (transit_clearance_weight); a goal
  // outside the map starts from the map edge with straight-line distance. Without
  // obstacles the score equals the previous bearing score.
  bool goal_cost_scoring=true;
  // Extra cost factor 1+w*(1-clearance/preferred_clearance)^2 inside that remaining-distance
  // search. 1 keeps 0.5m passages usable; 5 made a demo passage cost ~6x and chose detours.
  double goal_cost_clearance_weight=1;
  // 2026-11-25 goal approach / feasibility (library defaults 0 = previous behaviour;
  // planner node defaults: 0.30, 2.0, 0.85, 0.6).
  // final_goal_tolerance: the final-goal Hybrid A* may end within this distance of the
  // goal (was 0.75 cell), and a goal inside the min_clearance band or unknown is moved to
  // the nearest known-free cell when that cell leaves room inside this tolerance.
  double final_goal_tolerance=0;
  // goal_approach_radius: within this distance of the goal no waypoint reissue, and
  // intermediate waypoints must be nearer the goal than the vehicle (closer = better)
  // whenever such a waypoint exists.
  double goal_approach_radius=0;
  // gpp_steer_ratio: GPP Hybrid A* uses max_steer*ratio so RPP keeps steering margin.
  double gpp_steer_ratio=1;
  // exit_check_length: an intermediate waypoint (new or retained) needs at least one
  // forward arc of this length (left/half-left/straight/half-right/right at the GPP
  // steering limit) through traversable cells outside the min_clearance band.
  double exit_check_length=0;
  // start_clearance_exempt (node default 0.2): within this radius of the vehicle the
  // min_clearance band is not applied (vehicle may already be inside it). 0 = off.
  double start_clearance_exempt=0;
  // 2026-11-25 waypoint reissue (default off = previous behaviour). With a safe
  // active route, a fresh waypoint selection is evaluated after a new global map
  // once the vehicle has progressed along the adopted GPP path. The new waypoint
  // replaces the active one only when its score is better by reissue_margin.
  bool reissue_enabled=false;
  double reissue_travel_ratio=.4,reissue_min_travel=.2,reissue_end_margin=.6;
  double reissue_margin=.15,reissue_min_interval=1;
  // Own Hybrid A* budget for a reissue attempt, counted from the reissue start (the
  // safe route keeps being followed; planner results stay valid for 1 s).
  double reissue_search_seconds=.5;
};
struct CorridorCandidate {
  Pose point;
  double score=0,distance=0,clearance=0,heading=0,risk=0;
  bool final=false;
};
class CorridorSelector {
 public:
  explicit CorridorSelector(CorridorConfig config=CorridorConfig{});
  void reset();
  void reject(const Pose& point,double now);
  void accept(const CorridorCandidate& candidate,const Path& path,const Layers& layers);
  std::optional<CorridorCandidate> observed_goal_candidate(const CostField& field,const Pose& start,const Pose& goal,double now);
  std::vector<CorridorCandidate> candidates(const CostField& field,const Pose& start,const Pose& goal,double now);
  std::string invalidate(const CostField& field,const Layers& layers,const Pose& start,const Pose& goal,double now);
  std::optional<std::pair<CorridorCandidate,Path>> retained_path(const CostField& field,const Pose& start,const Pose& goal);
  // Same selection as candidates() without returning the active waypoint early.
  // active_score receives the active waypoint's fresh score (infinity if absent).
  std::vector<CorridorCandidate> reissue_candidates(const CostField& field,const Pose& start,const Pose& goal,double now,double* active_score);
  std::string detail;
 private:
  bool continuation_blocked(const Grid& grid,const Pose& start,const Pose& point,const Pose& goal) const;
  std::optional<Pose> final_point(const Grid& grid,const Pose& goal) const;
  bool new_direction_block(const Grid& grid,const Pose& start,const Pose& goal);
  CorridorConfig config_;
  std::optional<Pose> active_,final_goal_;
  Path active_path_;
  std::optional<double> accepted_slope_;
  std::vector<Pose> visited_;
  std::vector<std::pair<Pose,double>> rejected_;
  double last_probe_=-kInf,previous_resolution_=0;
  std::set<std::pair<int,int>> previous_blocked_;
  bool have_previous_blocked_=false;
  bool skip_active_shortcut_=false;
  double fresh_active_score_=kInf;
};
struct NavigatorConfig {
  VehicleConfig vehicle;
  CostConfig costs;
  SearchConfig search;
  RppConfig rpp;
  CorridorConfig corridor;
  std::string frame="odom";
  double arrival=.35,replan_period=1;
  double unknown_direction_weight=4,unknown_direction_radius=6,unknown_direction_free_angle=15*kPi/180;
  bool observed_guard=true;
  // Reuse the execution footprint in GPP and candidate RPP validation.
  bool footprint_planning=false;
  bool allow_initial_footprint_outside=false;
  double guard_half_length=.20,guard_half_width=.10,guard_deceleration=.4,guard_latency=.3,guard_margin=.1;
  // Centre offset along planning forward; -.15 gives front=.05 and rear=.35.
  double guard_center_offset=-.15;
};
struct NavigationOutput {
  Path path,trajectory;
  Motion motion;
  // Execution layer compares measured speed against this braking/observation
  // cap as well as checking the target command. Infinity means guard disabled.
  double observed_speed_cap=kInf;
  Pose target;
  bool has_target=false,arrived=false,path_changed=false,motion_valid=false;
  Pose start_footprint;
  bool has_start_footprint=false;
  std::string state="WAIT",reason;
  std::string rpp_rejection_key,rpp_rejection_detail;
};
// No ROS, threads, sensor or actuator ownership. Caller serializes goal/step.
// Absolute timestamps use the same clock as Grid::stamp_ns. A missing/mismatched
// camera-support grid stops the default observation-guarded profile.
// 2026-11-25: outcome of the latest waypoint reissue attempt (logging only).
struct ReissueEvent {
  std::uint64_t seq=0;          // increments on every reissue attempt
  std::string outcome;          // switched | kept_no_better | kept_search_failed
  Pose from,to;                 // active waypoint before / after the attempt
  double progress=0,length=0;   // metres travelled / total along the adopted GPP path
  double active_score=kInf,best_score=kInf;
  int candidates=0;             // candidates left after the score-margin filter
};
class Navigator {
 public:
  const ReissueEvent& reissue_event() const {return reissue_event_;}
  explicit Navigator(NavigatorConfig config=NavigatorConfig{});
  void set_goal(const Pose& goal);
  void cancel();
  void execution_feedback(bool blocked,double now);
  NavigationOutput step(const Layers& layers,const Grid& local,const Pose& pose,double now,
                        const Grid* observation_support=nullptr);
 private:
  NavigationOutput stop(std::string reason,bool clear_reference);
  void motion_failed(double now);
  void request_motion_replan(double now);
  bool reissue_due(const Layers& layers,const Pose& pose,double now,bool previous_route_clear);
  void adopt_plan(const Path& path,const Layers& layers,const Pose& pose,double now);
  NavigatorConfig config_;
  CorridorSelector selector_;
  std::optional<Pose> goal_,target_;
  std::optional<Pose> start_footprint_anchor_;
  Path reference_;
  double last_plan_=-kInf,failed_since_=-kInf,execution_failed_since_=-kInf;
  bool retry_=true;
  // Waypoint reissue bookkeeping (see CorridorConfig::reissue_enabled).
  Path plan_path_;
  std::vector<double> plan_cumulative_;
  std::size_t progress_index_=0;
  std::int64_t plan_map_stamp_=0;
  double last_reissue_=-kInf;
  double reissue_progress_=0;
  ReissueEvent reissue_event_;
};
} // namespace nomad_planning_cpp

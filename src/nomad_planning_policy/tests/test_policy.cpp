#include "nomad_planning_cpp/policy.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace nomad_planning_cpp;
namespace {
int checks=0;
void check(bool result,const char* message){++checks;if(!result)throw std::runtime_error(message);}
Grid clear_grid(){Grid g;g.width=100;g.height=80;g.resolution=.1;g.origin_x=-3;g.origin_y=-4;g.stamp_ns=100000000000LL;g.data.assign(g.width*g.height,0);return g;}
void paint(Grid& g,double x,double y,int value){auto c=world_to_cell(g,x,y);g.data[c.y*g.width+c.x]=value;}
Path line(double end,double step=.1){Path p;for(double x=0;x<end-1e-9;x+=step)p.push_back({x,0,0});p.push_back({end,0,0});return p;}
Layers layer_grid(const Grid& g){return {g,std::vector<double>(g.data.size(),0),std::vector<double>(g.data.size(),0)};}
}
int main(){try{
  {
    RppConfig cfg;
    check(rpp_lookaheads(cfg)==std::vector<double>({.8,1.,.6,1.2,1.5}),"legacy default candidate order changed");
    cfg.adaptive_lookahead=false;
    check(rpp_lookaheads(cfg)==std::vector<double>({.8}),"legacy single candidate changed");
    cfg.lookahead_candidates={1.2,.9,.6};
    check(rpp_lookaheads(cfg)==cfg.lookahead_candidates,"explicit order was sorted or extended");
    auto grid=clear_grid();auto path=line(3);VehicleConfig vehicle;
    int attempted=0;
    auto selected=regulated_pure_pursuit({},path,grid,vehicle,cfg,[&](const Path&,const Motion&){++attempted;return true;});
    check(selected.success&&selected.lookahead==1.2&&attempted==1,"did not stop at first feasible candidate");
    attempted=0;
    selected=regulated_pure_pursuit({},path,grid,vehicle,cfg,[&](const Path&,const Motion&){return ++attempted==2;});
    check(selected.success&&selected.lookahead==.9&&attempted==2,"added candidate/fallback order ignored");
    cfg.lookahead_candidates={.6};
    selected=regulated_pure_pursuit({},path,grid,vehicle,cfg);
    check(selected.success&&selected.lookahead==.6,"removed candidates were still selected");
    paint(grid,.5,0,100);
    check(!regulated_pure_pursuit({},path,grid,vehicle,cfg).success,"explicit candidates bypassed collision check");
    for(auto values:std::vector<std::vector<double>>{{0},{3.1},{.8,.8},{std::numeric_limits<double>::quiet_NaN()},std::vector<double>(33,.8)}){
      cfg.lookahead_candidates=values;bool rejected=false;
      try{rpp_lookaheads(cfg);}catch(const std::invalid_argument&){rejected=true;}
      check(rejected,"invalid explicit candidate list accepted");
    }
  }
  auto g=clear_grid();VehicleConfig v;RppConfig c;c.adaptive_lookahead=false;
  auto p=line(3);auto r=regulated_pure_pursuit({},p,g,v,c);
  check(r.success,"straight rear-steer RPP must succeed");check(r.motion.speed==.5&&r.motion.steering==0,"straight exact motion");check(r.trajectory.size()>2,"checked trajectory missing");
  check(distance(r.trajectory.back(),{})>=1.5-1e-10,"horizon was silently shortened");
  Path circle{{0,0,0}};for(int i=0;i<35;++i)circle.push_back(advance(circle.back(),.25,.1,v.reference_offset));
  r=regulated_pure_pursuit({},circle,g,v,c);check(r.success,"rear-steer curve must succeed");check(std::abs(r.motion.steering-std::atan(.72*.25))<2e-5,"rear nonsteering-axis curvature mismatch");
  v.rear_steer=false;v.reference_offset=.36;circle={{0,0,0}};for(int i=0;i<35;++i)circle.push_back(advance(circle.back(),-.25,.1,v.reference_offset));
  r=regulated_pure_pursuit({},circle,g,v,c);check(r.success&&r.motion.steering<0,"front-steer reference unsupported");v=VehicleConfig{};
  Path reverse{{0,0,0},{-1,0,0},{-2,0,0}};check(route_is_behind({},reverse),"behind route ignored vehicle-forward sign");check(!regulated_pure_pursuit({},reverse,g,v,c).success,"forward-only RPP accepted reverse path");
  Path loop{{0,0,0},{1,0,0},{2,1,1},{1,1,3},{-2,1,3}};check(!route_is_behind({},loop),"final goal incorrectly used for direction");
  paint(g,.5,0,100);check(!regulated_pure_pursuit({},p,g,v,c).success,"RPP crossed obstacle");g=clear_grid();paint(g,.5,0,-1);check(!regulated_pure_pursuit({},p,g,v,c).success,"RPP crossed unknown");
  auto prefix=known_prefix(g,{},p);check(!prefix.empty()&&prefix.back().x<.5,"known prefix did not sample route interior");
  g=clear_grid();paint(g,0,0,50);r=regulated_pure_pursuit({},p,g,v,c);check(r.success&&r.motion.speed<.2,"map cost speed regulation absent");
  {
    RppConfig slow;slow.adaptive_lookahead=false;slow.speed=.25;slow.allow_unknown=true;
    auto unknown=clear_grid();paint(unknown,0,0,-1);
    auto old=regulated_pure_pursuit({},p,unknown,v,slow);
    check(old.success&&std::abs(old.motion.speed-.25/4.5)<1e-12,"legacy unknown slowdown changed");
    slow.unknown_speed=.10;
    auto selected=regulated_pure_pursuit({},p,unknown,v,slow);
    check(selected.success&&std::abs(selected.motion.speed-.10)<1e-12,"unknown-cell 0.10m/s cap not applied");
    auto near=regulated_pure_pursuit({},line(.10),unknown,v,slow);
    check(near.success&&near.motion.speed<.10,"unknown speed incorrectly forced a minimum near goal");
    auto clear=clear_grid();auto normal=regulated_pure_pursuit({},p,clear,v,slow);
    check(normal.success&&std::abs(normal.motion.speed-.25)<1e-12,"known-free speed changed");
    paint(clear,0,0,79);auto costly=regulated_pure_pursuit({},p,clear,v,slow);
    check(costly.success&&std::abs(costly.motion.speed-.05)<1e-12,"known-cost slowdown below 0.10m/s was overridden");
    paint(unknown,.5,0,80);
    check(!regulated_pure_pursuit({},p,unknown,v,slow).success,"unknown speed cap bypassed known obstacle");
    unknown=clear_grid();paint(unknown,0,0,-1);slow.allow_unknown=false;
    check(!regulated_pure_pursuit({},p,unknown,v,slow).success,"unknown speed enabled prohibited unknown transit");
    for(double bad:{-.1,.3,std::numeric_limits<double>::quiet_NaN()}){
      slow.unknown_speed=bad;bool rejected=false;
      try{regulated_pure_pursuit({},p,unknown,v,slow);}catch(const std::invalid_argument&){rejected=true;}
      check(rejected,"invalid unknown speed accepted");
    }
  }
  g=clear_grid();auto invalid=p;invalid[1].yaw=std::numeric_limits<double>::quiet_NaN();check(!regulated_pure_pursuit({},invalid,g,v,c).success,"NaN yaw accepted");
  check(observed_distance(g,{},0,v)>1.9,"free observed horizon unexpectedly short");
  for(int y=0;y<g.height;++y)for(int x=40;x<g.width;++x)g.data[y*g.width+x]=-1;
  // Preserve this braking scenario's explicit original footprint; the selected
  // smaller default is covered by the width/obstacle contract test.
  double known=observed_distance(g,{},0,v,.52,.42);check(known>0&&known<.6,"swept footprint failed to stop before unknown");
  check(observation_speed_limit(known,.3,.4,.1)<.5,"observation speed cap absent");check(observation_speed_limit(0,.3,.4,.1)==0,"zero known distance must stop");
  g=clear_grid();auto layers=layer_grid(g);auto support=g;support.data.assign(g.data.size(),100);
  NavigatorConfig cfg;cfg.search.seconds=.5;cfg.search.max_expansions=100000;cfg.corridor.seconds=.5;cfg.costs.cluster_weight=0;
  Navigator nav(cfg);nav.set_goal({2,0,0});auto out=nav.step(layers,g,{},100,&support);
  check(out.motion_valid&&!out.path.empty()&&out.path_changed,"navigator failed free map");check(out.has_target&&distance(out.target,{2,0,0})<1e-6,"known mission goal not selected");
  auto retained=nav.step(layers,g,{.03,0,0},101.01,&support);check(retained.motion_valid&&!retained.path_changed,"safe route not retained");
  {
    Navigator continuous(cfg);continuous.set_goal({2,0,0});
    auto first=continuous.step(layers,g,{},100,&support);
    check(first.motion_valid,"continuous-replanning setup failed");
    auto changed_costs=layers;changed_costs.slope_cost.assign(g.data.size(),.3);
    auto pending=continuous.step(changed_costs,g,{.03,0,0},100.1,&support);
    check(pending.motion_valid&&!pending.path_changed,"safe route discarded merely for slope-triggered reselection");
    check(pending.reason.find("replan pending")!=std::string::npos,"reselection reason not exposed");
    check(!pending.trajectory.empty()&&distance(pending.trajectory.front(),{.03,0,0})<1e-9,"reselection reused old motor command instead of fresh-pose RPP");
    pending=continuous.step(changed_costs,g,{.06,0,0},100.2,&support);
    check(pending.motion_valid,"retry interval unnecessarily stopped the retained clear route");
    auto blocked=g;paint(blocked,.5,0,100);auto blocked_layers=layer_grid(blocked);
    auto halted=continuous.step(blocked_layers,blocked,{.06,0,0},100.3,&support);
    check(!halted.motion_valid,"retained route bypassed a newly observed obstacle");
    continuous.cancel();
    check(!continuous.step(layers,g,{},100.4,&support).motion_valid,"cancel reused continuous route");
  }

  nav.set_goal({0,2,0});auto changed=nav.step(layers,g,{},101.1,nullptr);check(!changed.motion_valid&&changed.path.empty(),"new goal reused old reference");
  nav.set_goal({2,0,0});out=nav.step(layers,g,{},100,&support);check(out.motion_valid,"reference reset failed");
  auto broken=g;broken.frame="map";auto stopped=nav.step(layers,broken,{},100,&support);check(!stopped.motion_valid&&stopped.path.empty()&&stopped.path_changed,"frame error left retained route live");
  auto mismatch=support;++mismatch.stamp_ns;check(!nav.step(layers,g,{},100,&mismatch).motion_valid,"unsynchronized camera support accepted");
  check(!nav.step(layers,g,{},104,&support).motion_valid,"stale map accepted");
  nav.set_goal({.2,0,0});check(nav.step(layers,g,{},100,&support).arrived,"arrival not detected");
  nav.cancel();check(nav.step(layers,g,{},100,&support).path.empty(),"cancel retained path");
  g=clear_grid();layers=layer_grid(g);support=g;support.data.assign(g.data.size(),100);nav.set_goal({2,0,0});out=nav.step(layers,g,{},100,&support);check(out.motion_valid,"obstacle test setup failed");
  for(int y=0;y<g.height;++y)for(int x=38;x<=42;++x)g.data[y*g.width+x]=100;
  layers=layer_grid(g);support=g;support.data.assign(g.data.size(),100);out=nav.step(layers,g,{},101.1,&support);check(!out.motion_valid&&out.path.empty(),"blocked map reused old reference");
  g=clear_grid();g.stamp_ns=101200000000LL;layers=layer_grid(g);support=g;support.data.assign(g.data.size(),100);
  out=nav.step(layers,g,{},101.2,&support);check(!out.motion_valid,"failed plan retried before retry interval");
  out=nav.step(layers,g,{},102.2,&support);check(out.motion_valid,"failed plan did not retry after interval");
  CorridorConfig cc;cc.seconds=1;CorridorSelector selector(cc);g=clear_grid();auto f=occupancy_costs(g);auto candidate=selector.observed_goal_candidate(f,{},Pose{2,0,0},100);
  check(candidate&&candidate->final,"observed mission probe failed");selector.accept(*candidate,line(2),layer_grid(g));
  check(selector.retained_path(f,{.1,0,0},{2,0,0}).has_value(),"selector safe retention failed");
  paint(g,1,0,100);auto bad=occupancy_costs(g);check(!selector.invalidate(bad,layer_grid(g),{},Pose{2,0,0},101).empty(),"new obstacle failed to invalidate");check(!selector.retained_path(bad,{},Pose{2,0,0}),"invalidated selector retained old path");
  // A far unknown mission must be able to choose a short, still-width-checked
  // frontier inside the approved 1.5m initial forward semicircle. No other
  // unknown cell is made free and the swept/braking guard stays enabled.
  {
    Grid startup;startup.width=startup.height=80;startup.resolution=.12;
    startup.origin_x=startup.origin_y=-4.8;startup.stamp_ns=100000000000LL;
    startup.data.assign(80*80,-1);
    for(int y=0;y<startup.height;++y)for(int x=0;x<startup.width;++x){auto p=cell_to_world(startup,{x,y});
      if(p.x*p.x+p.y*p.y<=1.5*1.5+1e-12&&p.y>=-1e-12)startup.data[y*startup.width+x]=0;}
    NavigatorConfig preview;preview.corridor.observation_guard=false;preview.corridor.observation_turn_weight=0;
    preview.corridor.arrival=.25;preview.search.seconds=.5;preview.corridor.seconds=.25;
    preview.costs.obstacle_weight=16;preview.costs.radius=4;preview.costs.slope_weight=5;preview.costs.terrain_weight=.5;
    preview.costs.cluster_weight=32;preview.costs.cluster_influence=.75;preview.costs.cluster_saturation=.4;
    preview.costs.unknown_surcharge=9;preview.costs.unknown_penalty=9;
    // Preserve the historical centred-box frontier expectation. With the new
    // longer rear, turning can leave its initial unknown-cell exemption.
    preview.guard_half_length=.30;preview.guard_half_width=.20; // Historical geometry for this regression only.
    preview.guard_center_offset=0;
    Navigator boot(preview);boot.set_goal({.15,6.11,0});
    const Pose start{0,0,kPi/2};auto seeded=boot.step(layer_grid(startup),startup,start,100,nullptr);
    check(seeded.motion_valid&&seeded.has_target&&!seeded.path.empty(),"short known-frontier fallback missing for distant unknown mission");
    check(distance(start,seeded.target)>=.4&&distance(start,seeded.target)<2,"fallback target outside short range");
    check(seeded.motion.speed>0&&seeded.motion.speed<=.5,"startup fallback violated speed bound");
    check(std::isfinite(seeded.observed_speed_cap),"startup fallback bypassed swept/braking guard");
    preview.guard_center_offset=-.20;Navigator asymmetric(preview);asymmetric.set_goal({.15,6.11,0});
    const auto rear_swing=asymmetric.step(layer_grid(startup),startup,start,100,nullptr);
    check(!rear_swing.motion_valid&&rear_swing.reason=="observation boundary","longer rear bypassed newly unobserved turning cells");
    std::cout<<"startup far goal: target="<<seeded.target.x<<","<<seeded.target.y<<" poses="<<seeded.path.size()
             <<" speed="<<seeded.motion.speed<<" steer="<<seeded.motion.steering<<" cap="<<seeded.observed_speed_cap<<"\n";
  }
  // Explicit relaxed profile: unknown may connect real observed waypoints,
  // but neither an entirely unknown endpoint nor a confirmed obstacle is free.
  {
    Grid gap=clear_grid();gap.origin_x=-3.05;gap.origin_y=-4.05;gap.data.assign(gap.data.size(),-1);
    paint(gap,1.2,0,0);paint(gap,1.4,0,0);
    CorridorConfig relaxed;relaxed.allow_unknown_transit=true;relaxed.waypoint_radius=1.5;
    relaxed.observation_guard=false;relaxed.observation_turn_weight=0;relaxed.arrival=.25;
    CorridorSelector pick(relaxed);auto options=pick.candidates(occupancy_costs(gap),{},Pose{2.5,0,0},100);
    check(!options.empty(),"unknown start / sparse known endpoint rejected by old inset");
    check(options.front().distance>1.3&&options.front().distance<=1.5,"furthest goal-direction waypoint not selected");
    check(!options.front().final,"unknown goal inside former 3m lock incorrectly became final");
    pick.reject(options.front().point,100);
    auto alternatives=pick.candidates(occupancy_costs(gap),{},Pose{2.5,0,0},100.1);
    check(!alternatives.empty(),"one candidate failure blacklisted whole 1.5m region");
    NavigatorConfig relaxed_nav;relaxed_nav.corridor=relaxed;relaxed_nav.costs.multi=false;
    relaxed_nav.vehicle.wheelbase=.335;relaxed_nav.vehicle.wheel_radius=.055;relaxed_nav.vehicle.reference_offset=-.1675;relaxed_nav.vehicle.max_steer=.3;
    relaxed_nav.search.seconds=.5;relaxed_nav.search.max_expansions=100000;relaxed_nav.rpp.speed=.25;
    Navigator drive(relaxed_nav);drive.set_goal({2.5,0,0});auto driven=drive.step(layer_grid(gap),gap,{},100,nullptr);
    check(driven.motion_valid&&!driven.path.empty(),"GPP/LPP/observation guard disagreed on allowed unknown transit");
    check(distance({},driven.target)<=1.5&&driven.path.back().x>=1.1,"unknown transit ended outside known waypoint");
    auto local_unknown=gap;local_unknown.data.assign(local_unknown.data.size(),-1);
    auto continued=drive.step(layer_grid(gap),local_unknown,{},100.1,nullptr);
    check(continued.motion_valid,"local unknown duplicated global endpoint observation gate");
    check(!known_prefix(gap,{},line(1.4)).size()&&known_prefix(gap,{},line(1.4),true).size()>2,"prefix unknown policy is inconsistent");
    auto no_endpoint=gap;no_endpoint.data.assign(no_endpoint.data.size(),-1);
    CorridorSelector none(relaxed);check(none.candidates(occupancy_costs(no_endpoint),{},Pose{3,0,0},100).empty(),"unknown endpoint fabricated");
    // An in-map wall still separates the observed endpoint from the start.
    auto wall=gap;for(int y=0;y<wall.height;++y)paint(wall,.7,wall.origin_y+(y+.5)*wall.resolution,80);
    CorridorSelector blocked(relaxed);check(blocked.candidates(occupancy_costs(wall),{},Pose{3,0,0},100).empty(),"relaxed selector crossed known obstacle");
    check(known_prefix(wall,{},line(1.4),true).empty(),"relaxed local prefix crossed known obstacle");
    RppConfig relaxed_rpp;relaxed_rpp.allow_unknown=true;check(!regulated_pure_pursuit({},line(1.4),wall,relaxed_nav.vehicle,relaxed_rpp).success,"relaxed RPP crossed known obstacle");
    paint(local_unknown,0,.3,80);
    check(observed_distance(local_unknown,{},0,relaxed_nav.vehicle,.52,.42,2,true)==0,"swept footprint ignored off-centre known obstacle");
    check(observed_distance(gap,{gap.origin_x+.1,0,0},0,relaxed_nav.vehicle,.52,.42,2,true)==0,"relaxed footprint left map bounds");
    // First (goal-aligned lateral) point has no short forward curve. A nearby
    // straight alternative must be tried, rather than rejecting all neighbours.
    auto choices=gap;choices.data.assign(choices.data.size(),-1);paint(choices,0,1.4,0);paint(choices,1.2,0,0);
    Navigator fallback(relaxed_nav);fallback.set_goal({0,4,0});auto alternative=fallback.step(layer_grid(choices),choices,{},100,nullptr);
    check(alternative.motion_valid&&alternative.target.x>1,"unreachable highest-ranked waypoint prevented feasible fallback");
  }
  {
    // The initial vehicle rectangle may straddle a missing rear map margin.
    // Its fixed world extent is the entire exception; it cannot move/turn into
    // additional outside space or exempt an observed obstacle inside the map.
    auto edge=clear_grid();edge.origin_x=-.2;const Pose anchor{};
    VehicleConfig physical;physical.wheelbase=.335;physical.wheel_radius=.055;physical.reference_offset=-.1675;physical.max_steer=.3;
    check(observed_distance(edge,anchor,0,physical,.52,.42,2,true)==0,"strict/default outside policy changed");
    check(observed_distance(edge,anchor,0,physical,.52,.42,2,true,&anchor)>1.9,"fixed initial rear footprint did not permit forward exit");
    check(observed_distance(edge,anchor,.3,physical,.52,.42,2,true,&anchor)<.3,"left turn expanded initial outside exception");
    check(observed_distance(edge,anchor,-.3,physical,.52,.42,2,true,&anchor)<.3,"right turn expanded initial outside exception");
    check(observed_distance(edge,Pose{-.3,0,0},0,physical,.52,.42,2,true,&anchor)==0,"initial exception allowed vehicle centre outside map");
    auto obstacle=edge;paint(obstacle,-.1,0,80);
    check(observed_distance(obstacle,anchor,0,physical,.52,.42,2,true,&anchor)==0,"initial exception hid an in-map known obstacle");
    auto shifted=edge;shifted.origin_x=.2;
    check(observed_distance(shifted,Pose{.3,0,0},0,physical,.52,.42,2,true,&anchor)>1.9,"map origin shift changed world-fixed initial footprint");
    check(observed_distance(shifted,Pose{.3,.4,0},0,physical,.52,.42,2,true,&anchor)==0,"lateral motion grew initial footprint exception");
    // Keep the established frontier-turn fixture geometry explicit; the new
    // footprint's edge/anchor behaviour is exercised by asymmetric_footprint_test.
    NavigatorConfig initial;initial.guard_half_length=.30;initial.guard_half_width=.20;initial.guard_center_offset=-.20;initial.vehicle=physical;initial.corridor.allow_unknown_transit=true;initial.corridor.arrival=.25;
    initial.corridor.observation_guard=false;initial.corridor.observation_turn_weight=0;initial.allow_initial_footprint_outside=true;
    initial.costs.multi=false;initial.rpp.speed=.25;initial.search.seconds=.3;
    Navigator fixed(initial);fixed.set_goal({3,0,0});auto first=fixed.step(layer_grid(edge),edge,anchor,100,nullptr);
    check(first.motion_valid&&first.has_start_footprint&&distance(first.start_footprint,anchor)<1e-12,"navigator did not capture initial footprint once");
    const Pose moved{.1,.2,0};auto later=fixed.step(layer_grid(edge),edge,moved,100.1,nullptr);
    check(later.has_start_footprint&&distance(later.start_footprint,anchor)<1e-12,"navigator moved its fixed initial anchor");
    fixed.set_goal({3,.2,0});auto new_goal=fixed.step(layer_grid(edge),edge,moved,100.2,nullptr);
    check(new_goal.has_start_footprint&&distance(new_goal.start_footprint,moved)<1e-12,"new goal did not reset initial footprint anchor");
    fixed.cancel();check(!fixed.step(layer_grid(edge),edge,moved,100.3,nullptr).has_start_footprint,"cancel retained initial footprint anchor");
  }
  std::cout<<"policy: "<<checks<<" checks PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<"policy test failure after "<<checks<<": "<<e.what()<<"\n";return 1;}}

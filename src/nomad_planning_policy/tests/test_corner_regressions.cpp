#include "corner_profile.hpp"
#include "nomad_planning_cpp/result_validation.hpp"
#include <iostream>
#include <stdexcept>
using namespace nomad_planning_cpp;
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
ResultGridView view(const Grid& g){return {g.resolution,g.width,g.height,g.origin_x,g.origin_y,g.origin_yaw,g.data};}
Layers free_layers(){Layers l;auto& g=l.grid;g.resolution=.1;g.width=80;g.height=60;g.origin_x=-2;g.origin_y=-2;g.stamp_ns=1000000000;g.data.assign(g.width*g.height,0);l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;}
int cell(const Grid& g,double x,double y){auto c=world_to_cell(g,x,y);return c.y*g.width+c.x;}
int main(){try{
 auto c=profile();
 for(int blocked:{80,99,100})for(bool multi:{false,true}){
  auto l=free_layers();auto& g=l.grid;g.data[cell(g,1.05,.05)]=blocked;g.data[cell(g,.85,.05)]=-1;
  auto legacy=c.costs;legacy.clearance_weight=0;legacy.multi=multi;auto enabled=legacy;enabled.clearance_weight=24;
  auto a=compose_costs(l,legacy),b=compose_costs(l,enabled);
  check(b.value(.85,.05)>a.value(.85,.05)+1,"unknown transit ignored nearby blocked clearance");
  check(b.value(.85,.15)>a.value(.85,.15)+1,"known transit ignored nearby blocked clearance");
  check(b.value(3,.05)==a.value(3,.05),"soft clearance affected distant free space");
  check(!std::isfinite(b.value(1.05,.05)),"blocked cell became traversable");
  check(b.grid.data==g.data,"clearance changed occupancy evidence");
 }
 auto l=free_layers();auto& g=l.grid;for(int y=0;y<g.height;++y)g.data[y*g.width+45]=100;
 CorridorSelector old_selector([&]{auto x=c.corridor;x.corner_aware=false;return x;}()),new_selector(c.corridor);
 auto field=compose_costs(l,c.costs);Pose pose{1.05,.05,0},goal{6,.05,0};auto old=old_selector.candidates(field,pose,goal,1),fresh=new_selector.candidates(field,pose,goal,1);
 check(!old.empty()&&!fresh.empty(),"wall candidate selection empty");
 check(fresh.front().point.x<old.front().point.x-.2,"near-wall waypoint still wins over clearance");
 // The default 0.8m lookahead clears this obstacle with its centre but
 // clips it with the body. The 1.5m alternative clears the same full footprint.
 auto clear=free_layers();Pose start{0,0,0};Path path{start};
 for(int j=0;j<60;++j)path.push_back(advance(path.back(),j<20?0:.4,.025,c.vehicle.reference_offset));
 auto map=clear.grid;map.data[19*map.width+30]=100;
 auto centre=regulated_pure_pursuit(start,path,map,c.vehicle,c.rpp);
 check(centre.success&&centre.lookahead==.8,"RPP fallback fixture no longer exercises default candidate");
 check(validate_remaining_result_path(view(map),centre.trajectory,start,true,false,c.guard_half_length,c.guard_half_width,nullptr,c.guard_center_offset)=="FOOTPRINT_KNOWN_OBSTACLE","RPP fixture must clip body only");
 MotionCheck guard=[&](const Path& trajectory,const Motion&){return validate_remaining_result_path(view(map),trajectory,start,true,false,c.guard_half_length,c.guard_half_width,nullptr,c.guard_center_offset).empty();};
 auto alternate=regulated_pure_pursuit(start,path,map,c.vehicle,c.rpp,guard);
 check(alternate.success&&alternate.lookahead!=centre.lookahead&&guard(alternate.trajectory,alternate.motion),"RPP did not try a safe alternative lookahead");
 auto open=free_layers();Navigator nav(c);nav.set_goal({3,0,0});auto out=nav.step(open,open.grid,{0,0,0},1);check(out.motion_valid,"new profile rejected straight free start");
 open.grid.stamp_ns=1200000000;auto retained=nav.step(open,open.grid,{0,0,0},1.2);check(retained.motion_valid&&!retained.path_changed,"safe path unnecessarily republished");
 auto local=open.grid;local.data[cell(local,-.15,0)]=80;auto blocked=nav.step(open,local,{0,0,0},1.2);check(!blocked.motion_valid,"new local footprint obstacle did not STOP");
 // Execution feedback is debounced, and cancel/new goal erase its history.
 nav.execution_feedback(true,1.3);nav.execution_feedback(false,1.8);nav.execution_feedback(true,2.0);
 open.grid.stamp_ns=2200000000;auto brief=nav.step(open,open.grid,{0,0,0},2.2);check(brief.motion_valid,"brief obstacle feedback broke retained motion");
 nav.execution_feedback(true,3.1);open.grid.stamp_ns=3200000000;auto replanned=nav.step(open,open.grid,{0,0,0},3.2);check(replanned.path_changed&&replanned.motion_valid,"persistent execution rejection did not replan");
 nav.cancel();check(!nav.step(open,open.grid,{0,0,0},3.2).motion_valid,"cancel failed");
 nav.set_goal({3,0,0});auto restarted=nav.step(open,open.grid,{0,0,0},3.2);check(restarted.motion_valid,"cancel did not reset rejection state");
 auto stale=nav.step(open,open.grid,{0,0,0},7);check(!stale.motion_valid&&stale.path.empty(),"stale inputs survived new profile");
 // Unknown transit stays unknown, and an unknown global endpoint cannot be used.
 auto sparse=free_layers();std::fill(sparse.grid.data.begin(),sparse.grid.data.end(),-1);
 Navigator unknown(c);unknown.set_goal({3,0,0});check(!unknown.step(sparse,sparse.grid,{0,0,0},1).motion_valid,"all-unknown map invented waypoint");
 for(int y=17;y<=23;++y)for(int x=29;x<=34;++x)sparse.grid.data[y*sparse.grid.width+x]=0;
 sparse.grid.stamp_ns=2200000000;auto transit=unknown.step(sparse,sparse.grid,{0,0,0},2.2);
 check(transit.motion_valid,"approved unknown transit to known endpoint regressed");check(sparse.grid.data[cell(sparse.grid,0,0)]==-1,"unknown starting cell relabelled");
 check(sparse.grid.data[cell(sparse.grid,transit.path.back().x,transit.path.back().y)]>=0,"unknown GPP endpoint accepted");
 // Fixed outside-map anchor permits departure, never newly swept outside space.
 auto edge=free_layers();edge.grid.origin_x=-.05;Navigator departure(c);departure.set_goal({3,0,0});
 check(departure.step(edge,edge.grid,{0,0,0},1).motion_valid,"initial rear footprint map-edge departure regressed");
 std::cout<<"corner_regressions: PASS (cost classes, waypoint clearance, RPP alternative, dynamic stop, retention, feedback, cancel, freshness, unknown, map edge)\n";return 0;
}catch(const std::exception& e){std::cerr<<"corner_regressions: FAIL: "<<e.what()<<'\n';return 1;}}

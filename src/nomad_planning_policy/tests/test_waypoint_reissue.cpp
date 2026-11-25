// 2026-11-25 waypoint reissue regression: a branch that becomes observed while
// following a safe route replaces the active waypoint only when the feature is on,
// a new map arrived, enough of the adopted path was consumed and the score gain
// exceeds the margin. Disabled behaviour must match the previous retention.
#include "corner_profile.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace nomad_planning_cpp;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static bool same_point(const Pose& a,const Pose& b){return distance(a,b)<1e-6;}
// ㅏ-shaped course: corridor along +x (|y|<=0.6) and a branch towards +y at x in [1.0,2.2].
static Layers course(bool branch_known,std::int64_t stamp){
  Layers l;auto& g=l.grid;g.resolution=.1;g.width=80;g.height=60;g.origin_x=-2;g.origin_y=-2;g.stamp_ns=stamp;
  g.data.assign(g.width*g.height,100);
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){
    const double wx=g.origin_x+(x+.5)*g.resolution,wy=g.origin_y+(y+.5)*g.resolution;
    auto& v=g.data[y*g.width+x];
    if(std::abs(wy)<=.6)v=0;
    else if(wy>.6&&wx>=1.0&&wx<=2.2&&wy<=3.9)v=branch_known?0:-1;
  }
  l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;
}
static Pose along(const Path& p,double s){double acc=0;for(std::size_t i=1;i<p.size();++i){acc+=distance(p[i-1],p[i]);if(acc>=s)return p[i];}return p.back();}
static double length(const Path& p){double acc=0;for(std::size_t i=1;i<p.size();++i)acc+=distance(p[i-1],p[i]);return acc;}
struct Outcome{NavigationOutput first,second;ReissueEvent event;};
static Outcome run(NavigatorConfig c,double progress_ratio,bool new_stamp){
  const Pose goal{1.6,3.5,0},start{0,0,0};
  Navigator nav(c);nav.set_goal(goal);
  auto m1=course(false,1000000000);
  auto first=nav.step(m1,m1.grid,start,1.0);
  check(first.motion_valid&&first.has_target,"initial plan failed");
  const Pose p=along(first.path,progress_ratio*length(first.path));
  auto m2=course(true,new_stamp?2000000000:1000000000);
  auto second=nav.step(m2,m2.grid,p,new_stamp?2.0:1.0+1.0);
  return {first,second,nav.reissue_event()};
}
int main(){try{
  auto base=profile();
  // 1) disabled: safe route retained even though the branch is now observed.
  {auto c=base;auto r=run(c,.5,true);
   check(r.second.motion_valid,"disabled: motion lost");
   check(!r.second.path_changed&&same_point(r.second.target,r.first.target),"disabled: waypoint changed (previous behaviour broken)");
   check(r.event.seq==0,"disabled: reissue event recorded");}
  // 2) enabled: after 50% progress and a new map, the branch waypoint wins.
  {auto c=base;c.corridor.reissue_enabled=true;auto r=run(c,.5,true);
   std::cout<<"enabled: first target=("<<r.first.target.x<<","<<r.first.target.y<<") second target=("<<r.second.target.x<<","<<r.second.target.y<<")\n";
   check(r.second.motion_valid,"enabled: motion lost");
   check(r.second.path_changed&&r.second.target.y>r.first.target.y+.2,"enabled: better branch waypoint not reissued");
   check(r.event.seq==1&&r.event.outcome=="switched"&&same_point(r.event.to,r.second.target)&&same_point(r.event.from,r.first.target)&&r.event.length>0,"enabled: switched event not recorded");}
  // 3) hysteresis: an unreachable margin keeps the active waypoint.
  {auto c=base;c.corridor.reissue_enabled=true;c.corridor.reissue_margin=100;auto r=run(c,.5,true);
   check(r.second.motion_valid&&!r.second.path_changed&&same_point(r.second.target,r.first.target),"margin: waypoint switched without enough gain");
   check(r.event.seq==1&&r.event.outcome=="kept_no_better"&&r.event.candidates==0,"margin: kept event not recorded");}
  // 4) same map stamp: no reissue (nothing new observed by the mapper).
  {auto c=base;c.corridor.reissue_enabled=true;auto r=run(c,.5,false);
   check(r.second.motion_valid&&same_point(r.second.target,r.first.target),"same map: waypoint reissued");}
  // 5) too little progress along the adopted path: no reissue yet.
  {auto c=base;c.corridor.reissue_enabled=true;c.corridor.reissue_end_margin=0;auto r=run(c,.05,true);
   check(r.second.motion_valid&&same_point(r.second.target,r.first.target),"progress: reissued before travel threshold");}
  // 6) invalid parameters are rejected.
  {auto c=base;c.corridor.reissue_travel_ratio=1.5;bool threw=false;try{Navigator n(c);}catch(const std::invalid_argument&){threw=true;}check(threw,"invalid ratio accepted");}
  std::cout<<"waypoint_reissue: 6 checks + 3 event checks PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}

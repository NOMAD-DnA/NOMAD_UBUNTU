// 2026-11-25 waypoint handover regression: reaching the active waypoint less than
// replan_period after the last plan must select the next waypoint in the same step
// instead of stopping with "waiting for global path retry interval".
#include "corner_profile.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace nomad_planning_cpp;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
// Straight corridor along +x (|y|<=0.6), everything else known obstacle.
static Layers corridor(){
  Layers l;auto& g=l.grid;g.resolution=.1;g.width=100;g.height=40;g.origin_x=-2;g.origin_y=-2;g.stamp_ns=1000000000;
  g.data.assign(g.width*g.height,100);
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){const double wy=g.origin_y+(y+.5)*g.resolution;if(std::abs(wy)<=.6)g.data[y*g.width+x]=0;}
  l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;
}
static Pose near_end(const Path& p,double back){double acc=0;for(std::size_t i=p.size()-1;i>0;--i){acc+=distance(p[i-1],p[i]);if(acc>=back)return p[i-1];}return p.front();}
int main(){try{
  auto c=profile();
  const Pose goal{6.5,0,0},start{0,0,0};auto m=corridor();
  // 1) arrival 0.3 s after the last plan: next waypoint is planned immediately.
  {Navigator nav(c);nav.set_goal(goal);
   auto first=nav.step(m,m.grid,start,1.0);
   check(first.motion_valid&&first.has_target,"initial plan failed");
   check(distance(first.target,goal)>.3,"test needs an intermediate waypoint");
   const Pose p=near_end(first.path,.15);
   auto second=nav.step(m,m.grid,p,1.3);
   std::cout<<"handover: first target=("<<first.target.x<<","<<first.target.y<<") second reason=\""<<second.reason<<"\" target=("<<second.target.x<<","<<second.target.y<<")\n";
   check(second.motion_valid,"stopped at waypoint handover");
   check(second.path_changed&&second.target.x>first.target.x+.3,"next waypoint not selected at handover");}
  // 2) replan_period remains configurable and validated.
  {auto k=c;k.replan_period=0;bool threw=false;try{Navigator n(k);}catch(const std::invalid_argument&){threw=true;}check(threw,"replan_period=0 accepted");}
  std::cout<<"waypoint_handover: 2 checks PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}}

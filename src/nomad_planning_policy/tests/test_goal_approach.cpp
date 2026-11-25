// 2026-11-25 field review (19:03-19:13 records): final goal tolerance, goal approach zone,
// GPP steering margin and dead-end waypoint endpoints.
#include "corner_profile.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {
int checks=0;
void expect(bool ok,const char* what){++checks;if(!ok){std::cerr<<"FAIL: "<<what<<"\n";std::exit(1);}}
Layers open_map(){Layers l;auto& g=l.grid;g.resolution=.125;g.width=40;g.height=48;g.origin_x=-2.5;g.origin_y=-.5;g.stamp_ns=1;g.data.assign(g.width*g.height,0);
  l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;}
void block(Layers& l,double x0,double x1,double y0,double y1){auto& g=l.grid;for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){const double wx=g.origin_x+(x+.5)*g.resolution,wy=g.origin_y+(y+.5)*g.resolution;if(wx>=x0&&wx<=x1&&wy>=y0&&wy<=y1)g.data[y*g.width+x]=100;}}
NavigatorConfig config(bool on){auto c=profile();c.corridor.waypoint_radius=2;c.corridor.min_clearance=.15;
  if(on){c.corridor.final_goal_tolerance=.30;c.corridor.goal_approach_radius=2;c.corridor.gpp_steer_ratio=.85;c.corridor.exit_check_length=.6;}return c;}
}
int main(){
  // 1) Goal 0.2m in front of an end wall (inside the 0.15m band once the cell geometry is
  //    applied): the goal is still offered as the final candidate, moved to a free cell.
  {auto l=open_map();block(l,-2.5,2.5,4.55,4.8);const Pose goal{0,4.42,0},start{0,3.0,kPi/2};
   for(bool on:{false,true}){auto c=config(on);Navigator nav(c);nav.set_goal(goal);
     auto out=nav.step(l,l.grid,start,1);
     if(on){expect(out.motion_valid,"goal near wall: route found");
       expect(!out.path.empty()&&distance(out.path.back(),goal)<=.30+1e-9,"goal near wall: route ends within 0.30 m of the goal");}
     else{std::cout<<"goal near wall without tolerance: target=("<<out.target.x<<","<<out.target.y<<") reason="<<out.reason<<"\n";
       expect(out.path.empty()||distance(out.path.back(),goal)>.30,"goal near wall: previous behaviour cannot finish at the goal");}}}
  // 2) Approach zone: the goal is not selectable (blocked cell, no snapping), 1.5m away.
  //    Previously the radial term picked 2m points past or beside the goal; now waypoints
  //    that close in on the goal come first.
  {auto l=open_map();block(l,-.1,.1,3.9,4.1);const Pose goal{0,4,0},start{-.6,2.6,kPi/4};
   auto on=config(true);on.corridor.final_goal_tolerance=0;auto off=config(false);
   auto f=compose_costs(l,on.costs);CorridorSelector s1(on.corridor),s0(off.corridor);
   auto ch=s1.candidates(f,start,goal,0),ch0=s0.candidates(f,start,goal,0);
   expect(!ch.empty()&&!ch0.empty(),"approach: candidates exist");
   std::cout<<"approach: off first=("<<ch0[0].point.x<<","<<ch0[0].point.y<<") goalD="<<distance(ch0[0].point,goal)
            <<"  on first=("<<ch[0].point.x<<","<<ch[0].point.y<<") goalD="<<distance(ch[0].point,goal)<<" start goalD="<<distance(start,goal)<<"\n";
   expect(distance(ch[0].point,goal)<distance(start,goal),"approach: first waypoint is nearer the goal than the vehicle");
   // The previous choice is a 2m point beyond the goal (overshoot); the new one is short of it.
   expect(distance(start,ch0[0].point)>distance(start,goal),"approach: previous behaviour overshoots the goal");
   expect(distance(start,ch[0].point)<distance(start,goal),"approach: new waypoint does not overshoot the goal");}
  // 3) GPP steering margin: the adopted path curvature stays within max_steer*0.85.
  {auto l=open_map();const Pose goal{1.6,1.6,0},start{0,0,kPi/2};auto c=config(true);c.corridor.exit_check_length=0;
   Navigator nav(c);nav.set_goal(goal);auto out=nav.step(l,l.grid,start,1);expect(out.motion_valid,"steer margin: route found");
   const double kmax=std::tan(c.vehicle.max_steer*.85)/c.vehicle.wheelbase;double worst=0;
   for(std::size_t i=1;i<out.path.size();++i){const double ds=distance(out.path[i-1],out.path[i]);if(ds>1e-6)worst=std::max(worst,std::abs(std::remainder(out.path[i].yaw-out.path[i-1].yaw,2*kPi))/ds);}
   expect(worst<=kmax*1.05,"steer margin: path curvature within 85% steering");}
  // 4) Dead-end endpoint: a wall appears 0.4m beyond the retained waypoint (as in the
  //    3rd demo run); the waypoint is dropped and a new one is planned at once.
  {const Pose goal{0,6,0},start{0,0,kPi/2};
   for(bool on:{false,true}){auto c=config(on);c.corridor.gpp_steer_ratio=1;Navigator nav(c);nav.set_goal(goal);
     auto l=open_map();auto first=nav.step(l,l.grid,start,1);expect(first.motion_valid,"dead end: first route");
     const Pose wp=first.target;auto l2=open_map();block(l2,-2.5,2.5,wp.y+.35,wp.y+.55);l2.grid.stamp_ns=2;
     auto second=nav.step(l2,l2.grid,start,1.2);
     const bool changed=second.path_changed||distance(second.target,wp)>.1||!second.motion_valid;
     std::cout<<"dead end ("<<(on?"on":"off")<<"): first=("<<wp.x<<","<<wp.y<<") second=("<<second.target.x<<","<<second.target.y<<") reason="<<second.reason<<"\n";
     if(on)expect(changed,"dead end: retained waypoint dropped");else expect(!changed,"dead end: previous behaviour keeps the waypoint");}}
  // 5) Invalid parameters.
  for(int k=0;k<4;++k){auto c=config(true);if(k==0)c.corridor.gpp_steer_ratio=0;if(k==1)c.corridor.gpp_steer_ratio=1.2;if(k==2)c.corridor.final_goal_tolerance=-1;if(k==3)c.corridor.exit_check_length=-1;
    bool threw=false;try{CorridorSelector s(c.corridor);}catch(const std::invalid_argument&){threw=true;}expect(threw,"invalid parameter rejected");}
  std::cout<<"goal_approach: "<<checks<<" checks PASS\n";
}

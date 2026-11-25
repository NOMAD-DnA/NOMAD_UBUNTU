// 2026-11-25 goal_cost_scoring: waypoint direction score uses obstacle-aware progress.
#include "corner_profile.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {
int checks=0;
void expect(bool ok,const char* what){++checks;if(!ok){std::cerr<<"FAIL: "<<what<<"\n";std::exit(1);}}
// 4.5m x 4.5m map, goal (0,5) beyond the top edge. As at the demo start (2026-11-25
// records): a wall from the left blocks the straight line to the goal, a post stands to its right and
// everything beyond the plate's row is still unobserved.
Layers scene(bool plate){
  Layers l;auto& g=l.grid;g.resolution=.125;g.width=36;g.height=36;g.origin_x=-2;g.origin_y=-.5;g.stamp_ns=1;g.data.assign(g.width*g.height,0);
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){const double wx=g.origin_x+(x+.5)*g.resolution,wy=g.origin_y+(y+.5)*g.resolution;auto& v=g.data[y*g.width+x];
    if(plate&&((wy>=1.9&&wy<=2.15&&wx<=.8)||(wy>=1.3&&wy<=2.4&&wx>=1.5&&wx<=1.7)))v=100;else if(plate&&wy>2.15)v=-1;}
  l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;}
NavigatorConfig config(bool on){auto c=profile();c.corridor.waypoint_radius=2;c.corridor.preferred_clearance=.52;c.corridor.transit_clearance_weight=5;c.corridor.goal_cost_scoring=on;return c;}
std::vector<CorridorCandidate> pick(bool on,bool plate,const Pose& start){auto c=config(on);auto l=scene(plate);auto f=compose_costs(l,c.costs);CorridorSelector s(c.corridor);return s.candidates(f,start,{0,5,0},0);}
}
int main(){
  const Pose start{.1,.75,70*kPi/180};
  auto before=pick(false,true,start),after=pick(true,true,start);
  expect(!before.empty()&&!after.empty(),"candidates exist");
  std::cout<<"plate: bearing-only best=("<<before[0].point.x<<","<<before[0].point.y<<") r="<<before[0].distance
           <<"  goal-cost best=("<<after[0].point.x<<","<<after[0].point.y<<") r="<<after[0].distance<<"\n";
  // Bearing-only picks a point under the plate on the goal side (the car turns back toward it).
  expect(before[0].point.x<.55,"previous score picks the goal-side point under the plate");
  // Obstacle-aware progress picks a point toward the plate's open end, and not behind the car.
  expect(after[0].point.x>=.65,"goal cost picks the open side of the plate");
  expect(after[0].point.y>start.y,"goal cost does not pick a point behind the car");
  // Without obstacles the score is unchanged.
  auto open_before=pick(false,false,start),open_after=pick(true,false,start);
  bool same=open_before.size()==open_after.size();
  for(std::size_t i=0;same&&i<open_before.size();++i)same=std::abs(open_before[i].score-open_after[i].score)<1e-6&&distance(open_before[i].point,open_after[i].point)<1e-9;
  expect(same,"open space scores identical");
  auto bad=config(true);bad.corridor.goal_cost_clearance_weight=-1;bool threw=false;
  try{CorridorSelector s(bad.corridor);}catch(const std::invalid_argument&){threw=true;}
  expect(threw,"negative goal cost clearance weight rejected");
  std::cout<<"goal_cost_scoring: "<<checks<<" checks PASS\n";
}

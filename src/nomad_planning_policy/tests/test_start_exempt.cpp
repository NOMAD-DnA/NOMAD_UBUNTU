// 2026-11-25 start_clearance_exempt: a vehicle already inside the min_clearance band
// (run 20:39, min_clearance 0.20) beside an obstacle end must still plan its way out
// instead of stopping with "start_blocked_or_outside_map". (Alongside a long wall a 0.2 m
// radius is not enough to leave the band; that case still stops.)
#include "corner_profile.hpp"
#include <iostream>
#include <stdexcept>
namespace {
int checks=0;
void expect(bool ok,const char* what){++checks;if(!ok){std::cerr<<"FAIL: "<<what<<"\n";std::exit(1);}}
Layers corridor(){Layers l;auto& g=l.grid;g.resolution=.125;g.width=40;g.height=48;g.origin_x=-2.5;g.origin_y=-.5;g.stamp_ns=1;g.data.assign(g.width*g.height,0);
  // Plate end (as at the 20:03/20:39 corner): a wall column near x=-0.2 that ends at
  // y=0.45; the vehicle stands 0.08 m beside its end (inside the 0.20 band), heading on.
  for(int y=0;y<g.height;++y){const double wy=g.origin_y+(y+.5)*g.resolution;if(wy>=.45)continue;const int x=static_cast<int>((-0.2-g.origin_x)/g.resolution);g.data[y*g.width+x]=100;}
  l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;}
}
int main(){
  const auto l=corridor();const double wall_edge=-2.5+static_cast<int>((-0.2+2.5)/.125)*.125+.125;
  const Pose start{wall_edge+.08,0.5,kPi/2},goal{0.6,3.5,0};
  for(double exempt:{0.,.2}){auto c=profile();c.corridor.waypoint_radius=2;c.corridor.min_clearance=.20;c.corridor.start_clearance_exempt=exempt;
    Navigator nav(c);nav.set_goal(goal);auto out=nav.step(l,l.grid,start,1);
    std::cout<<"exempt="<<exempt<<": state="<<out.state<<" reason="<<out.reason<<"\n";
    if(exempt>0){expect(out.motion_valid,"exempt 0.2: vehicle inside the band still plans");
      // Beyond the exempt radius the route respects the 0.20 m band.
      auto d=blocked_clearances(l.grid);bool ok=true;for(const auto& p:out.path){if(distance(p,start)<=.2+.13)continue;auto cl=world_to_cell(l.grid,p.x,p.y);ok=ok&&d[cl.y*l.grid.width+cl.x]>=.20-.0625-1e-6-.07;}
      expect(ok,"exempt 0.2: route leaves the band and keeps min_clearance afterwards");}
    else expect(!out.motion_valid&&out.reason.find("start_blocked")!=std::string::npos,"no exempt: previous behaviour stops with start_blocked");}
  auto c=profile();c.corridor.start_clearance_exempt=-1;bool threw=false;try{CorridorSelector s(c.corridor);}catch(const std::invalid_argument&){threw=true;}
  expect(threw,"negative exempt radius rejected");
  std::cout<<"start_exempt: "<<checks<<" checks PASS\n";
}

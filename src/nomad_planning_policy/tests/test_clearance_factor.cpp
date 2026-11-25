// 2026-11-25 cost.clearance_factor: relative clearance cost (corner hugging fix).
#include "corner_profile.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {
int checks=0;
void expect(bool ok,const char* what){++checks;if(!ok){std::cerr<<"FAIL: "<<what<<"\n";std::exit(1);}}
// L corridor 1.3m wide at 0.125m (field costmap resolution), turning left at x=4..5.5.
Layers corner(){
  Layers l;auto& g=l.grid;g.resolution=.125;g.width=60;g.height=60;g.stamp_ns=1;g.data.assign(g.width*g.height,100);
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){const double wx=(x+.5)*g.resolution,wy=(y+.5)*g.resolution;
    if((wx>=.2&&wx<5.5&&wy>=.2&&wy<1.5)||(wx>=4&&wx<5.5&&wy>=.2&&wy<6.5))g.data[y*g.width+x]=0;}
  l.slope_cost.assign(g.data.size(),0);l.terrain_cost.assign(g.data.size(),0);return l;}
CostConfig field_costs(double factor){auto c=profile().costs;c.clearance_weight=40;c.clearance_radius=.58;c.clearance_factor=factor;return c;}
double min_clearance(const Grid& g,const Path& p){auto d=blocked_clearances(g);double m=kInf,s=0;for(std::size_t i=1;i<p.size();++i){s+=distance(p[i-1],p[i]);if(s<.3)continue;auto c=world_to_cell(g,p[i].x,p[i].y);m=std::min(m,d[c.y*g.width+c.x]);}return m;}
}
int main(){
  auto l=corner();
  // factor 0 keeps the previous additive weights exactly.
  auto base=field_costs(0);base.clearance_weight=0;auto add=field_costs(0);
  auto f_base=compose_costs(l,base),f_add=compose_costs(l,add),f_rel=compose_costs(l,field_costs(2));
  auto d=blocked_clearances(l.grid);bool exact=true,relative=true;
  for(std::size_t i=0;i<d.size();++i){if(!std::isfinite(f_base.weights[i]))continue;const double q=std::pow(std::max(0.,1-d[i]/.58),2);
    exact=exact&&std::abs(f_add.weights[i]-(f_base.weights[i]+40*q))<1e-9;
    relative=relative&&std::abs(f_rel.weights[i]-(f_base.weights[i]*(1+2*q)+40*q))<1e-9;}
  expect(exact,"factor 0 equals previous additive clearance cost");
  expect(relative,"factor 2 scales the base cost near blocked cells");
  // Hybrid A* around the corner keeps more clearance with the relative cost.
  auto c=profile();c.search.seconds=1;c.search.primitive_length=.125;c.search.goal_tolerance=.09;
  const Pose start{3.,.85,0},goal{4.75,2.6,kPi/2};
  auto p0=hybrid_astar(f_add,start,goal,c.vehicle,c.search),p2=hybrid_astar(f_rel,start,goal,c.vehicle,c.search);
  expect(!p0.path.empty()&&!p2.path.empty(),"both corner searches succeed");
  const double m0=min_clearance(l.grid,p0.path),m2=min_clearance(l.grid,p2.path);
  std::cout<<"corner min clearance: additive="<<m0<<" relative="<<m2<<"\n";
  expect(m2>=m0+.05,"relative clearance cost keeps the corner path further from the wall");
  auto bad=field_costs(-1);bool threw=false;try{compose_costs(l,bad);}catch(const std::invalid_argument&){threw=true;}
  expect(threw,"negative clearance factor rejected");
  std::cout<<"clearance_factor: "<<checks<<" checks PASS\n";
}

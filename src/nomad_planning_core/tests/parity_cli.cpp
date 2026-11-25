// Test-only stdin harness. No ROS, sensors or actuator I/O.
// COST/HYBRID: mode w h resolution ox oy oyaw [data N] [slope N] [terrain N]
//   obstacle_w slope_w terrain_w radius missing unknown_surcharge
//   cluster_w cluster_radius cluster_influence cluster_saturation
// HYBRID then: sx sy syaw gx gy gyaw wheelbase steer offset rear(0/1) budget
// ADVANCE: mode x y yaw curvature distance reference_offset
// Floating tokens may be nan/inf. Output has 17 digits for Python comparison.
#include "nomad_planning_cpp/core.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace nomad_planning_cpp;
namespace{double number(){std::string s;if(!(std::cin>>s))throw std::invalid_argument("Truncated input");std::size_t n=0;double x=std::stod(s,&n);if(n!=s.size())throw std::invalid_argument("Bad floating token");return x;}Pose pose(){return {number(),number(),number()};}}
int main(){try{std::string mode;std::cin>>mode;std::cout<<std::setprecision(17);
  if(mode=="ADVANCE"){auto p=pose();const double k=number(),d=number(),r=number();p=advance(p,k,d,r);std::cout<<p.x<<' '<<p.y<<' '<<p.yaw<<'\n';return 0;}
  if(mode!="COST"&&mode!="HYBRID")throw std::invalid_argument("Unknown mode");
  Grid g;std::cin>>g.width>>g.height;g.resolution=number();g.origin_x=number();g.origin_y=number();g.origin_yaw=number();
  if(g.width<=0||g.height<=0||g.width>10000||g.height>10000)throw std::invalid_argument("Bad dimensions");
  const auto n=static_cast<std::size_t>(g.width)*g.height;g.data.resize(n);for(auto& v:g.data){int a;if(!(std::cin>>a)||a < -1||a>100)throw std::invalid_argument("Bad occupancy");v=static_cast<std::int8_t>(a);}
  Layers l;l.grid=g;l.slope_cost.resize(n);l.terrain_cost.resize(n);for(auto&v:l.slope_cost)v=number();for(auto&v:l.terrain_cost)v=number();
  CostConfig c;c.obstacle_weight=number();c.slope_weight=number();c.terrain_weight=number();c.radius=number();c.missing_cost=number();c.unknown_surcharge=number();c.cluster_weight=number();c.cluster_radius=number();c.cluster_influence=number();c.cluster_saturation=number();
  auto f=compose_costs(l,c);if(mode=="COST"){for(double w:f.weights)std::cout<<w<<' ';std::cout<<'\n';return 0;}
  const Pose start=pose(),goal=pose();VehicleConfig v;v.wheelbase=number();v.max_steer=number();v.reference_offset=number();v.rear_steer=number()!=0;SearchConfig s;s.max_expansions=static_cast<int>(number());
  const auto out=hybrid_astar(f,start,goal,v,s);std::cout<<out.path.size()<<' '<<out.expansions<<' '<<out.cost<<'\n';for(auto p:out.path)std::cout<<p.x<<' '<<p.y<<' '<<p.yaw<<'\n';std::cerr<<out.detail<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}

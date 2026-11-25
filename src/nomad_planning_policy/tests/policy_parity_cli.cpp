#include "nomad_planning_cpp/policy.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
using namespace nomad_planning_cpp;
Grid read_grid(){Grid g;std::cin>>g.width>>g.height>>g.resolution>>g.origin_x>>g.origin_y;g.data.resize(g.width*g.height);int v;for(auto& q:g.data){std::cin>>v;q=v;}return g;}
Path read_path(){std::size_t n;std::cin>>n;Path p(n);for(auto& q:p)std::cin>>q.x>>q.y>>q.yaw;return p;}
int main(){try{std::cout<<std::setprecision(17);std::string mode;std::cin>>mode;
  Grid grid=read_grid();Pose start;std::cin>>start.x>>start.y>>start.yaw;
  VehicleConfig v;
  if(mode=="rpp"){
    RppConfig c;int adaptive,rear;std::cin>>v.wheelbase>>v.max_steer>>v.reference_offset>>c.speed>>c.lookahead>>adaptive>>rear;v.rear_steer=rear;c.adaptive_lookahead=adaptive;
    auto r=regulated_pure_pursuit(start,read_path(),grid,v,c);std::cout<<r.success<<" "<<r.motion.speed<<" "<<r.motion.steering<<" "<<r.score<<" "<<r.lookahead<<" "<<r.trajectory.size()<<"\n";for(auto p:r.trajectory)std::cout<<p.x<<" "<<p.y<<" "<<p.yaw<<"\n";
  }else if(mode=="prefix"){
    auto p=known_prefix(grid,start,read_path());std::cout<<p.size()<<"\n";for(auto q:p)std::cout<<q.x<<" "<<q.y<<" "<<q.yaw<<"\n";
  }else if(mode=="guard"){
    double steer,hl,hw;std::cin>>v.wheelbase>>v.reference_offset>>steer>>hl>>hw;std::cout<<observed_distance(grid,start,steer,v,hl,hw)<<"\n";
  }else if(mode=="corridor"){
    Pose goal;std::cin>>goal.x>>goal.y;auto field=occupancy_costs(grid);CorridorConfig c;c.seconds=10;c.continuation_distance=3;CorridorSelector selector(c);auto options=selector.candidates(field,start,goal,100);
    std::cout<<options.size()<<"\n";for(auto q:options)std::cout<<q.point.x<<" "<<q.point.y<<" "<<q.score<<" "<<q.distance<<" "<<q.clearance<<" "<<q.heading<<" "<<q.final<<" "<<q.risk<<"\n";
  }else throw std::invalid_argument("unknown parity mode");
  return std::cin.fail()?2:0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}

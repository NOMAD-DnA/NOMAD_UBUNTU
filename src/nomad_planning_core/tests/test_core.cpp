#include "nomad_planning_cpp/core.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace nomad_planning_cpp;
namespace {
int checks=0;
void require(bool value,const char* what){++checks;if(!value)throw std::runtime_error(what);}
void near(double a,double b,double eps=1e-10){require(std::abs(a-b)<=eps,"numerical mismatch");}
template<class F>void rejects(F f){bool caught=false;try{f();}catch(const std::invalid_argument&){caught=true;}require(caught,"invalid input accepted");}
Grid grid(int w,int h,double r=.2){Grid g;g.width=w;g.height=h;g.resolution=r;g.data.resize(w*h,0);return g;}
}
int main(){try{
  const auto p=advance({1,2,.3},.4,1.2,-.36);
  const auto back=advance(p,.4,-1.2,-.36);near(back.x,1);near(back.y,2);near(wrap(back.yaw-.3),0);
  near(advance({0,0,0},0,2,-.36).x,2);
  require(advance({0,0,0},.4,.2,-.36).yaw>0,"rear-steer yaw convention");
  VehicleConfig v;validate_vehicle(v);v.reference_offset=.36;rejects([&]{validate_vehicle(v);});
  v=VehicleConfig{};v.max_steer=1;rejects([&]{validate_vehicle(v);});
  // Provisional +/-35 degree equivalent command retains the Ackermann check.
  // The linked inner-wheel value is derived model data, not measured geometry.
  VehicleConfig steering35;steering35.wheelbase=.335;steering35.reference_offset=-.1675;
  steering35.max_steer=35*kPi/180.;steering35.joint_limit=1.0813901513620499;
  validate_vehicle(steering35);near(equivalent_steering_limit(steering35),steering35.max_steer);
  const double curvature35=std::tan(steering35.max_steer)/steering35.wheelbase;
  const auto left35=advance({},curvature35,.1,steering35.reference_offset);
  const auto right35=advance({},-curvature35,.1,steering35.reference_offset);
  near(left35.x,right35.x);near(left35.y,-right35.y);near(left35.yaw,-right35.yaw);
  require(left35.yaw>0&&right35.yaw<0,"35 degree command preserves rear-steer yaw sign");
  steering35.joint_limit-=1e-6;rejects([&]{validate_vehicle(steering35);});
  auto g=grid(10,10);g.origin_x=-1;g.origin_y=2;g.origin_yaw=kPi/2;
  for(Cell c:{Cell{0,0},Cell{4,7},Cell{9,9}}){auto p1=cell_to_world(g,c);require(world_to_cell(g,p1.x,p1.y)==c,"grid rotation mismatch");}
  g.data.pop_back();rejects([&]{validate_grid(g);});g=grid(8,7);g.data[0]=101;rejects([&]{validate_grid(g);});
  // Distances must originate only at confirmed obstacle cells, never inflation.
  g=grid(11,9);g.data[4*11+5]=100;g.data[4*11+6]=80;auto d=obstacle_distances(g);
  near(d[0],std::hypot(5.,4.)*.2);near(d[4*11+6],.2);
  std::mt19937 gen(9245);for(int run=0;run<12;++run){g=grid(13,11,.13);for(auto& n:g.data)if(gen()%17==0)n=100;d=obstacle_distances(g);
    for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){double brute=kInf;for(int yy=0;yy<g.height;++yy)for(int xx=0;xx<g.width;++xx)if(g.data[yy*g.width+xx]==100)brute=std::min(brute,std::hypot(double(x-xx),double(y-yy))*g.resolution);near(d[y*g.width+x],brute);}}
  g=grid(7,7,1);g.data[3*7+3]=100;auto density=cluster_density(g,1,0,1);near(density[3*7+3],.2);near(density[3*7+4],.2);near(density[0],0);
  auto influenced=cluster_density(g,1,1,1);near(influenced[3*7+3],1);
  g=grid(3,2,1);g.data={0,-1,100,79,80,0};Layers l{g,{0,0,0,NAN,0,1},{.25,0,0,.5,0,NAN}};CostConfig c;c.obstacle_weight=0;
  auto f=compose_costs(l,c);near(f.weights[0],1.25);near(f.weights[1],7.5);require(std::isinf(f.weights[2]),"blocked softened");near(f.weights[3],2.5);require(std::isinf(f.weights[4]),"inflation softened");near(f.weights[5],3);
  l.slope_cost[0]=INFINITY;rejects([&]{compose_costs(l,c);});l.slope_cost[0]=-.01;rejects([&]{compose_costs(l,c);});
  g=grid(2,2,1);g.data={0,100,100,0};f=occupancy_costs(g);require(std::isinf(f.segment({.5,.5,0},{1.5,1.5,0})),"corner cutting accepted");
  g=grid(80,50);f=occupancy_costs(g);const Pose start{1,3,0},goal{10,3,0};auto result=hybrid_astar(f,start,goal);
  require(!result.path.empty(),"open-space route missing");require(distance(result.path.back(),goal)<=.29,"goal tolerance violated");require(std::isfinite(f.path_cost(result.path)),"invalid returned path");
  f.gates.push_back([](const Pose&a,const Pose&b){return (a.x<5&&b.x>=5)||(a.x>=5&&b.x<5);});SearchConfig budget;budget.max_expansions=200;
  result=hybrid_astar(f,start,goal,VehicleConfig{},budget);require(result.path.empty(),"hard gate crossed");require(result.detail=="planning budget exhausted","budget not reported");
  g=grid(80,50);for(int y=0;y<g.height;++y)g.data[y*g.width+25]=100;f=occupancy_costs(g);result=hybrid_astar(f,start,goal,VehicleConfig{},budget);require(result.path.empty(),"wall crossed");
  result=hybrid_astar(f,start,{5.1,3,0});require(result.path.empty()&&result.expansions==0,"blocked goal accepted");
  g=grid(80,50);f=occupancy_costs(g);result=hybrid_astar(f,start,goal,VehicleConfig{},SearchConfig{},[](Cell c){return c.y<5;});require(result.path.empty(),"corridor constraint ignored");
  f.weights[0]=0;rejects([&]{hybrid_astar(f,start,goal);});
  // Goal tolerance must not finish on an unknown neighbouring cell in the
  // opt-in transit profile, even though its traversal weight is finite.
  g=grid(80,50,.1);g.origin_x=g.origin_y=-.05;g.data.assign(g.data.size(),-1);
  g.data[world_to_cell(g,1.2,0).y*g.width+world_to_cell(g,1.2,0).x]=0;
  f=occupancy_costs(g);SearchConfig short_search;short_search.require_known_endpoint=true;short_search.primitive_length=.1;short_search.seconds=.5;
  v=VehicleConfig{};v.wheelbase=.335;v.wheel_radius=.055;v.reference_offset=-.1675;v.max_steer=.3;
  result=hybrid_astar(f,{},Pose{1.2,0,0},v,short_search);
  require(!result.path.empty(),"short known endpoint unreachable through unknown gap");
  auto cell=world_to_cell(g,result.path.back().x,result.path.back().y);require(g.data[cell.y*g.width+cell.x]==0,"goal tolerance accepted unknown final cell");
  std::cout<<"PASS "<<checks<<" core assertions\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}

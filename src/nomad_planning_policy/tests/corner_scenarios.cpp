#include "nomad_planning_cpp/policy.hpp"
#include "nomad_planning_cpp/result_validation.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <map>
using namespace nomad_planning_cpp;
#include "corner_profile.hpp"
ResultGridView view(const Grid& g){return {g.resolution,g.width,g.height,g.origin_x,g.origin_y,g.origin_yaw,g.data};}
struct Scenario{std::string name;Layers layers;Pose start,goal;};
Scenario scenario(std::string name,double resolution){
 Scenario s;s.name=name;auto& g=s.layers.grid;g.resolution=resolution;g.width=int(std::round(7.5/resolution));g.height=g.width;g.data.assign(g.width*g.height,100);g.stamp_ns=1000000000;
 s.start={.75,.75,0};s.goal={4.75,5.75,0};
 for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){double wx=(x+.5)*g.resolution,wy=(y+.5)*g.resolution;bool free=false;
  if(name=="open_diagonal"||name=="open_right")free=wx>=.2&&wx<7&&wy>=.2&&wy<7;
  else if(name=="left_corner"||name=="right_corner")free=(wx>=.2&&wx<5.5&&wy>=.2&&wy<1.5)||(wx>=4.&&wx<5.5&&wy>=.2&&wy<6.5);
  else if(name=="narrow_straight")free=wx>=.2&&wx<6.5&&wy>=.4&&wy<1.1;
  else if(name=="straight")free=wx>=.2&&wx<6.5&&wy>=.2&&wy<1.5;
  else if(name=="gentle_curve")free=std::hypot(wx-.75,wy-3.75)>=2.25&&std::hypot(wx-.75,wy-3.75)<3.75;
  if(free)g.data[y*g.width+x]=0;
 }
 if(name=="straight"||name=="narrow_straight")s.goal={5.75,.75,0};
 if(name=="open_diagonal")s.goal={5.5,3.5,0};
 if(name=="open_right"){s.start.y=6.75;s.goal={5.5,3.5,0};}
 if(name=="gentle_curve")s.goal={3.75,3.75,0};
 if(name=="right_corner"){
  for(int y=0;y<g.height/2;++y)for(int x=0;x<g.width;++x)std::swap(g.data[y*g.width+x],g.data[(g.height-y-1)*g.width+x]);
  s.start.y=7.5-s.start.y;s.goal.y=7.5-s.goal.y;
 }
 s.layers.slope_cost.assign(g.data.size(),0);s.layers.terrain_cost.assign(g.data.size(),0);return s;
}
int main(int argc,char** argv){
 const bool strict=argc>1&&std::string(argv[1])=="--require-success";bool all=true;const double resolution=argc>2?std::stod(argv[2]):.1;
 for(const auto& name:{"left_corner","right_corner","straight","narrow_straight","gentle_curve","open_diagonal","open_right"}){
  auto s=scenario(name,resolution);auto c=profile();Navigator nav(c);nav.set_goal(s.goal);Pose pose=s.start;double now=1,max_ms=0,total_ms=0,travel=0;int moves=0,stops=0,changes=0;bool arrived=false;std::map<std::string,int> reasons;
  std::ofstream trace(std::string(name)+"_trace.csv");trace<<"time,x,y,yaw,speed,steer,target_x,target_y,reason\n";
  for(int step=0;step<900;++step){now=1+step*.2;s.layers.grid.stamp_ns=static_cast<std::int64_t>(now*1e9);auto began=std::chrono::steady_clock::now();auto out=nav.step(s.layers,s.layers.grid,pose,now);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();max_ms=std::max(max_ms,ms);total_ms+=ms;if(out.path_changed)++changes;
   if(out.arrived){arrived=true;break;}
#ifdef NOMAD_CORNER_ENABLED
   if(!out.path.empty()&&out.motion_valid){auto why=validate_remaining_result_path(view(s.layers.grid),out.path,pose,true,true,c.guard_half_length,c.guard_half_width,&s.start,c.guard_center_offset);if(!why.empty())throw std::runtime_error("unsafe GPP: "+why);}
#endif
   auto why=out.motion_valid?validate_remaining_result_path(view(s.layers.grid),out.trajectory,pose,true,false,c.guard_half_length,c.guard_half_width,&s.start,c.guard_center_offset):out.reason;
   const bool valid=out.motion_valid&&why.empty();
   trace<<now<<','<<pose.x<<','<<pose.y<<','<<pose.yaw<<','<<(valid?out.motion.speed:0)<<','<<out.motion.steering<<','<<out.target.x<<','<<out.target.y<<','<<why<<'\n';
   if(valid){auto next=advance(pose,std::tan(out.motion.steering)/c.vehicle.wheelbase,out.motion.speed*.2,c.vehicle.reference_offset);
    auto collision=validate_remaining_result_path(view(s.layers.grid),{pose,next},pose,true,false,c.guard_half_length,c.guard_half_width,&s.start,c.guard_center_offset);
    if(!collision.empty())throw std::runtime_error(name+std::string(": executed collision ")+collision);
    travel+=distance(pose,next);pose=next;++moves;
   }else{++stops;++reasons[why];}
   if(stops>200&&moves<10)break;
  }
  std::cout<<name<<" arrived="<<arrived<<" remaining="<<distance(pose,s.goal)<<" moves="<<moves<<" stops="<<stops<<" replans="<<changes<<" travel="<<travel<<" max_ms="<<max_ms<<" mean_ms="<<total_ms/std::max(1,moves+stops)<<'\n';
  for(auto& [why,count]:reasons)std::cout<<"  "<<count<<" "<<why<<'\n';
  all=all&&arrived;
 }
 return strict&&!all?1:0;
}

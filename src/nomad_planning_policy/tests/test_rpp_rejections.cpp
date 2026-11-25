#include "nomad_planning_cpp/policy.hpp"
#include "nomad_planning_cpp/result_validation.hpp"
#include <iostream>
#include <stdexcept>
using namespace nomad_planning_cpp;
int count=0;
void check(bool b,const char* msg){++count;if(!b)throw std::runtime_error(msg);}
Grid grid(){Grid g;g.width=80;g.height=80;g.resolution=.1;g.origin_x=-2;g.origin_y=-4;g.data.assign(6400,0);return g;}
void paint(Grid& g,double x,double y,int v){auto c=world_to_cell(g,x,y);g.data[c.y*g.width+c.x]=v;}
int main(){try{
 VehicleConfig v;RppConfig cfg;cfg.lookahead_candidates={.8,1.,.6,1.2,1.5};Path p{{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
 auto g=grid();auto ok=regulated_pure_pursuit({},p,g,v,cfg);check(ok.success&&ok.rejection_key.empty(),"normal motion polluted");
 auto bad=g;paint(bad,.5,0,100);auto r=regulated_pure_pursuit({},p,bad,v,cfg);
 check(!r.success&&r.rejection_key.find("PATH_KNOWN_OBSTACLE")!=std::string::npos,"obstacle not classified");
 check(r.rejection_detail.find("cell_x=")!=std::string::npos,"missing collision cell");
 check(r.rejection_key.find("1.500:")!=std::string::npos,"not all candidates captured");
 bad=g;paint(bad,.5,0,-1);r=regulated_pure_pursuit({},p,bad,v,cfg);
 check(!r.success&&r.rejection_key.find("PATH_UNOBSERVED")!=std::string::npos,"unknown not classified");
 cfg.allow_unknown=true;check(regulated_pure_pursuit({},p,bad,v,cfg).success,"diagnostics changed unknown policy");
 bad=g;bad.width=25;bad.data.assign(bad.width*bad.height,0);r=regulated_pure_pursuit({},p,bad,v,cfg);
 check(!r.success&&r.rejection_key.find("PATH_OUTSIDE_MAP")!=std::string::npos,"boundary not classified");
 Path tight{{0,0,0},{.1,.5,0},{.2,1.,0},{.3,1.5,0}};r=regulated_pure_pursuit({},tight,g,v,cfg);
 check(!r.success&&r.rejection_key.find("GEOMETRY_STEERING_LIMIT")!=std::string::npos,"steering not classified");
 check(r.rejection_detail.find("steer_deg=")!=std::string::npos,"missing steering evidence");
 r=regulated_pure_pursuit({},Path{{0,0,0},{-2,0,0}},g,v,cfg);
 check(!r.success&&r.rejection_key.find("GEOMETRY_ROUTE_BEHIND")!=std::string::npos,"behind not classified");
 // Real swept-footprint validation rejects a side obstacle while centreline is clear.
 bad=g;paint(bad,.5,.2,100);std::string reason;
 auto footprint=[&](const Path& path,const Motion&){reason=validate_remaining_result_path(
  {bad.resolution,bad.width,bad.height,bad.origin_x,bad.origin_y,bad.origin_yaw,bad.data},path,{},true,false,.3,.3,nullptr,0);return reason.empty();};
 r=regulated_pure_pursuit({},p,bad,v,cfg,footprint,[&]{return std::make_pair("FOOTPRINT:"+reason,std::string{});});
 check(!r.success&&r.rejection_key.find("FOOTPRINT:FOOTPRINT_KNOWN_OBSTACLE")!=std::string::npos,"footprint not classified");
 auto braking=[&](const Path&,const Motion&){return observation_speed_limit(.1,.3,.4,.1)>=.02;};
 r=regulated_pure_pursuit({},p,g,v,cfg,braking,[]{return std::make_pair(std::string("BRAKING_INSUFFICIENT"),std::string("cap_mps=0"));});
 check(!r.success&&r.rejection_key.find("BRAKING_INSUFFICIENT")!=std::string::npos,"braking not classified");
 int tries=0;r=regulated_pure_pursuit({},p,g,v,cfg,[&](const Path&,const Motion&){return ++tries==2;});
 check(r.success&&r.lookahead==1&&tries==2,"first valid candidate changed");
 check(r.rejection_key.empty(),"recovered result leaked failure diagnostic");
 std::cout<<count<<" rejection checks passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

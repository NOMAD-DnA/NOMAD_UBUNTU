#include "nomad_planning_cpp/policy.hpp"
#include "nomad_planning_cpp/result_validation.hpp"
#include <iostream>
#include <stdexcept>

namespace npc=nomad_planning_cpp;
static int checks=0;
static void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
static npc::ResultGridView view(const npc::Grid& g){return {g.resolution,g.width,g.height,g.origin_x,g.origin_y,g.origin_yaw,g.data};}
static npc::Grid free_grid(){npc::Grid g;g.resolution=.02;g.width=200;g.height=200;g.origin_x=-2;g.origin_y=-2;g.data.assign(40000,0);return g;}
static void put(npc::Grid& g,double x,double y,int v){auto c=npc::world_to_cell(g,x,y);g.data.at(c.y*g.width+c.x)=v;}
int main(){try{
 const npc::NavigatorConfig cfg;
 const double hl=cfg.guard_half_length,hw=cfg.guard_half_width,offset=cfg.guard_center_offset;
 check(std::abs(hl+offset-.05)<1e-12&&std::abs(hl-offset-.35)<1e-12&&hw==.1,"selected front/rear/side extents differ from request");
 check(cfg.guard_margin==.1,"stop margin changed with footprint geometry");
 // Obstacles just inside/outside each requested edge, repeated after arbitrary
 // translation and rotation. Neither checker may fall back to a centred box.
 struct Probe{double forward,left;bool blocked;};
 const Probe probes[]={{.025,.05,true},{.11,.05,false},{-.31,.05,true},{-.41,.05,false},
                       {-.15,.065,true},{-.15,.17,false},{-.15,-.065,true},{-.15,-.17,false}};
 for(double yaw:{0.,npc::kPi/2,-npc::kPi/2,npc::kPi,.63})for(const auto& p:probes){
  auto g=free_grid();const npc::Pose pose{.13,-.07,yaw};
  put(g,pose.x+std::cos(yaw)*p.forward-std::sin(yaw)*p.left,
        pose.y+std::sin(yaw)*p.forward+std::cos(yaw)*p.left,100);
  const auto why=npc::validate_remaining_result_path(view(g),{pose},pose,true,false,hl,hw,nullptr,offset);
  check(p.blocked?why=="FOOTPRINT_KNOWN_OBSTACLE":why.empty(),"asymmetric final footprint edge or rotation incorrect");
  const double known=npc::observed_distance(g,pose,0,cfg.vehicle,hl,hw,.01,true,nullptr,offset);
  check(p.blocked?known==0:known>0,"observed guard disagrees with asymmetric final check");
 }
 auto g=free_grid();put(g,-.21,-.21,100);
 const npc::Pose start{},turned{0,0,npc::kPi/2};
 check(npc::validate_remaining_result_path(view(g),{start},start,true,false,hl,hw,nullptr,offset).empty(),"rotation fixture overlaps starting footprint");
 check(npc::validate_remaining_result_path(view(g),{turned},turned,true,false,hl,hw,nullptr,offset).empty(),"rotation fixture overlaps ending footprint");
 check(npc::validate_remaining_result_path(view(g),{start,turned},start,true,false,hl,hw,nullptr,offset)=="FOOTPRINT_KNOWN_OBSTACLE","long rear corner swept through obstacle between orientations");
 // The longer rear may initially protrude outside the map, but the exemption
 // remains the original world-fixed rectangle and never excuses known obstacles.
 auto edge=free_grid();edge.origin_x=-.2;const npc::Path forward{start,{.2,0,0}};
 check(npc::validate_remaining_result_path(view(edge),forward,start,true,false,hl,hw,nullptr,offset)=="FOOTPRINT_OUTSIDE_MAP","long rear extent outside map was missed");
 check(npc::validate_remaining_result_path(view(edge),forward,start,true,false,hl,hw,&start,offset).empty(),"initial asymmetric outside region failed departure");
 check(npc::observed_distance(edge,start,0,cfg.vehicle,hl,hw,.2,true,&start,offset)>.19,"observed guard lost fixed asymmetric departure exception");
 const npc::Pose shifted{0,.5,0};
 check(npc::validate_remaining_result_path(view(edge),{shifted,{.2,.5,0}},shifted,true,false,hl,hw,&start,offset)=="FOOTPRINT_OUTSIDE_MAP","asymmetric exception moved with reference pose");
 check(npc::observed_distance(edge,shifted,0,cfg.vehicle,hl,hw,.2,true,&start,offset)==0,"observed asymmetric exception moved with reference pose");
 put(edge,-.1,.06,80);
 check(npc::validate_remaining_result_path(view(edge),forward,start,true,false,hl,hw,&start,offset)=="FOOTPRINT_KNOWN_OBSTACLE","asymmetric initial exception hid a known obstacle");
 check(npc::observed_distance(edge,start,0,cfg.vehicle,hl,hw,.2,true,&start,offset)==0,"observed asymmetric exception hid a known obstacle");
 for(double bad:{hl,-hl,std::numeric_limits<double>::quiet_NaN()}){
  auto invalid=cfg;invalid.guard_center_offset=bad;bool rejected=false;
  try{npc::Navigator n(invalid);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"invalid footprint offset accepted by navigator");
  check(npc::validate_remaining_result_path(view(g),{start},start,true,false,hl,hw,nullptr,bad)=="FOOTPRINT_GEOMETRY_INVALID","invalid footprint offset accepted by final checker");
  check(npc::observed_distance(g,start,0,cfg.vehicle,hl,hw,.1,true,nullptr,bad)==0,"invalid footprint offset accepted by observed guard");
 }
 std::cout<<"asymmetric_footprint: "<<checks<<" checks PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<"asymmetric_footprint: FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}

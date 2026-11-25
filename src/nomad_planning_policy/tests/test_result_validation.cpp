#include "nomad_planning_cpp/result_validation.hpp"
#include "nomad_planning_cpp/policy.hpp"
#include <iostream>
#include <stdexcept>

namespace npc=nomad_planning_cpp;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static npc::ResultGridView view(const npc::Grid& g){return {g.resolution,g.width,g.height,g.origin_x,g.origin_y,g.origin_yaw,g.data};}
static void put(npc::Grid& g,double x,double y,int value){auto c=npc::world_to_cell(g,x,y);g.data[c.y*g.width+c.x]=static_cast<std::int8_t>(value);}
int main(){try{
 npc::Grid g;g.resolution=.1;g.width=60;g.height=40;g.origin_x=-2;g.origin_y=-2;g.data.assign(2400,0);
 const npc::Pose start{0,0,0};const npc::Path path{{0,0,0},{.6,0,0},{1.2,0,0}};
 auto validate=[&](bool unknown=false,bool endpoint=true){return npc::validate_remaining_result_path(view(g),path,start,unknown,endpoint);};
 check(validate().empty(),"free route rejected");
 put(g,-1,1,100);check(validate().empty(),"unrelated map change invalidated route");
 put(g,.6,0,-1);check(validate()=="PATH_UNOBSERVED","strict unknown transit changed");
 check(validate(true).empty(),"approved unknown transit rejected");
 put(g,1.2,0,-1);check(validate(true)=="PATH_ENDPOINT_UNOBSERVED","global endpoint must stay known free");
 check(validate(true,false).empty(),"local endpoint incorrectly requires second observation");
 put(g,1.2,0,0);put(g,.6,0,80);check(validate(true)=="PATH_KNOWN_OBSTACLE","new obstacle did not stop retained route");
 put(g,.6,0,79);check(validate(true).empty(),"nonblocking cost was treated as occupied");
 put(g,.6,0,100);
 check(npc::validate_remaining_result_path(view(g),path,{.9,0,0},true,true).empty(),"passed obstacle behind vehicle invalidated remaining path");
 put(g,.6,0,0);put(g,.65,.35,100);
 check(validate(true).empty(),"reference centre unexpectedly used vehicle footprint");
 check(npc::validate_remaining_result_path(view(g),path,start,true,false,.52,.42)=="FOOTPRINT_KNOWN_OBSTACLE","swept local footprint missed known side obstacle");
 const npc::NavigatorConfig selected;
 check(npc::validate_remaining_result_path(view(g),path,start,true,false,selected.guard_half_length,selected.guard_half_width,nullptr,selected.guard_center_offset).empty(),"selected 0.40x0.20 footprint rejected side clearance");
 check(npc::observed_distance(g,start,0,selected.vehicle,selected.guard_half_length,selected.guard_half_width,2,true,nullptr,selected.guard_center_offset)>1.9,"default observed guard disagrees with selected footprint clearance");
 put(g,.65,-.05,100);
 check(npc::validate_remaining_result_path(view(g),path,start,true,false,selected.guard_half_length,selected.guard_half_width,nullptr,selected.guard_center_offset)=="FOOTPRINT_KNOWN_OBSTACLE","selected footprint missed an obstacle inside its width");
 check(npc::observed_distance(g,start,0,selected.vehicle,selected.guard_half_length,selected.guard_half_width,2,true,nullptr,selected.guard_center_offset)<.65,"selected observed guard ignored an obstacle inside its width");
 put(g,.65,-.05,0);
 put(g,.65,.35,0);
 check(npc::validate_remaining_result_path(view(g),{{0,0,0},{1.2,2.1,0}},start,true,false)=="PATH_OUTSIDE_MAP","outside path accepted");
 check(npc::validate_remaining_result_path(view(g),{{0,0,0},{1e300,0,0}},start,true,false)=="PATH_OUTSIDE_MAP","huge finite coordinate did not fail safely");
 check(npc::validate_remaining_result_path(view(g),path,{0,1.9,0},true,false,.52,.42)=="FOOTPRINT_OUTSIDE_MAP","outside footprint accepted");
 put(g,-.25,.25,-1);
 check(npc::validate_remaining_result_path(view(g),path,start,false,false,.52,.42).empty(),"strict initial unknown footprint exemption changed");
 g.data.pop_back();check(validate(true)=="MAP_GEOMETRY_INVALID","invalid grid shape accepted");
 npc::Grid edge;edge.resolution=.1;edge.width=40;edge.height=30;edge.origin_x=-2;edge.origin_y=-.1;edge.data.assign(1200,0);
 const npc::Pose anchor{0,0,npc::kPi/2};const npc::Path departure{{0,0,npc::kPi/2},{0,.4,npc::kPi/2},{0,1,npc::kPi/2}};
 check(npc::validate_remaining_result_path(view(edge),departure,anchor,true,false,.52,.42)=="FOOTPRINT_OUTSIDE_MAP","outside exception enabled by default");
 check(npc::validate_remaining_result_path(view(edge),departure,anchor,true,false,.52,.42,&anchor).empty(),"fixed initial outside footprint did not permit departure");
 const npc::Pose shifted{.6,0,npc::kPi/2};const npc::Path shift_path{shifted,{.6,1,npc::kPi/2}};
 check(npc::validate_remaining_result_path(view(edge),shift_path,shifted,true,false,.52,.42,&anchor)=="FOOTPRINT_OUTSIDE_MAP","moving vehicle expanded the initial outside exception");
 const npc::Pose turned{0,0,0};const npc::Path turn_path{turned,{1,0,0}};
 check(npc::validate_remaining_result_path(view(edge),turn_path,turned,true,false,.52,.42,&anchor)=="FOOTPRINT_OUTSIDE_MAP","rotation admitted new outside footprint cells");
 put(edge,.3,.2,100);
 check(npc::validate_remaining_result_path(view(edge),departure,anchor,true,false,.52,.42,&anchor)=="FOOTPRINT_KNOWN_OBSTACLE","initial exception bypassed a known in-map obstacle");
 put(edge,.3,.2,0);
 check(npc::validate_remaining_result_path(view(edge),departure,{0,-.2,npc::kPi/2},true,false,.52,.42,&anchor)=="PATH_OUTSIDE_MAP","initial exception admitted an outside vehicle centre");
 check(npc::validate_remaining_result_path(view(edge),{{0,0,0},{0,-.2,0}},anchor,true,true,.52,.42,&anchor)=="PATH_ENDPOINT_OUTSIDE_MAP","initial exception admitted an outside endpoint");
 edge.origin_y=-.05;
 check(npc::validate_remaining_result_path(view(edge),departure,anchor,true,false,.52,.42,&anchor).empty(),"fixed world anchor failed reprojection after map origin changed");
 std::cout<<"result_validation: PASS (24 retained-map/unknown/obstacle/fixed-start cases)\n";return 0;
}catch(const std::exception& e){std::cerr<<"result_validation: FAIL: "<<e.what()<<'\n';return 1;}}

#include "nomad_costmap/indoor.hpp"
#include <fstream>
#include <iostream>
#include <functional>
#include <random>
using namespace nomad;
static int checks=0;
static void check(bool ok,const std::string& name){++checks;if(!ok)throw std::runtime_error(name);}
static void fails(const std::function<void()>& f,const std::string& name){bool thrown=false;try{f();}catch(const std::exception&){thrown=true;}check(thrown,name);}
static Pose pose(int64_t t=1000000000){Pose p;p.stamp=t;p.p=V3(0,.0625,0);return p;}
static Frame frame(int64_t t,Points points){Frame f;f.stamp=t;f.source=0;f.pose=pose(t);f.points=points;for(size_t i=0;i<points.size();++i){f.stamps.push_back(t+int64_t(i));f.ray_origins.emplace_back(0,.0625,.37);f.capture_poses.push_back(pose(f.stamps.back()));}return f;}
static IndoorMap fresh(const IndoorOptions& o){IndoorMap m(o);m.initialize(pose(),M4::Identity());return m;}
static size_t index(const CostSnapshot& s,Key k){return size_t((k.second-int64_t(std::llround(s.origin_y/s.resolution)))*s.width+k.first-int64_t(std::llround(s.origin_x/s.resolution)));}
static bool blocked(const IndoorMap& m,Key k){auto it=m.cells.find(k);return it!=m.cells.end()&&it->second.obstacle;}
// Independent oracle: slab-intersect each candidate cell, then minimize ray distance
// to vehicle over that interval. No DDA or circle-root clipping is used here.
static std::set<Key> oracle(const V3& origin,const V3& hit,const V3& centre,double r,double radius){
 std::set<Key> keys;Eigen::Vector2d a=origin.head<2>(),d=hit.head<2>()-a;
 if(d.squaredNorm()<=1e-20)return keys;
 for(int64_t x=int64_t(std::floor((centre.x()-radius)/r))-1;x<=int64_t(std::floor((centre.x()+radius)/r))+1;++x)
 for(int64_t y=int64_t(std::floor((centre.y()-radius)/r))-1;y<=int64_t(std::floor((centre.y()+radius)/r))+1;++y){
  double enter=0,leave=1;bool valid=true;
  for(int axis=0;axis<2;++axis){double lo=double(axis?y:x)*r,hi=lo+r;if(std::abs(d[axis])<1e-15){if(a[axis]<lo||a[axis]>=hi)valid=false;}else{double u=(lo-a[axis])/d[axis],v=(hi-a[axis])/d[axis];if(u>v)std::swap(u,v);enter=std::max(enter,u);leave=std::min(leave,v);}}
  if(!valid||leave<=enter+1e-12)continue;
  const double t=std::clamp((centre.head<2>()-a).dot(d)/d.squaredNorm(),enter,leave);
  if((a+t*d-centre.head<2>()).squaredNorm()<radius*radius-1e-12)keys.insert({x,y});
 }
 return keys;
}
int main(int argc,char**argv){try{
 require(argc==3,"group config required");std::ifstream file(argv[2]);Json j;file>>j;auto o=IndoorOptions::load(j);o.camera_forward_axis_imu=V3::UnitX();o.fusion.semantic["moving_front_free_region"]["enabled"]=false;
 std::string group=argv[1];
 if(group=="geometry"){
  std::mt19937_64 rng(20261124);std::uniform_real_distribution<double> place(-1,1),end(-5,5),offset(-.3,.3);
  for(int n=0;n<1200;++n){V3 c(place(rng),place(rng),0),a=c+V3(offset(rng),offset(rng),.37),b=c+V3(end(rng),end(rng),end(rng));auto traced=trace_ray_cells(a,b,c,.125,2.);std::set<Key> actual(traced.begin(),traced.end());check(actual==oracle(a,b,c,.125,2.),"DDA vs independent cell-intersection oracle case "+std::to_string(n));check(actual.size()==traced.size(),"No duplicate traversal cell");}
  auto straight=trace_ray_cells(V3(0,.0625,.37),V3(3.,.0625,0.),V3(0,.0625,0),.125,2.);
  check(straight.size()==16,"2m from aligned grid origin is16 cells");check(straight.front()==Key{0,0}&&straight.back()==Key{15,0},"No cells beyond2m");
  check(trace_ray_cells(V3(-3,2,0),V3(3,2,0),V3::Zero(),.125,2.).empty(),"Tangent touches no positive path");
  for(double angle:{-180.,-90.,-80.001,-80.,0.,80.,80.001,90.,180.}){auto m=fresh(o);double a=angle*pi/180.;m.ingest(frame(2000000000,{V3(1.9*std::cos(a),.0625+1.9*std::sin(a),-1.)}));check((m.rays_used==1)==(std::abs(angle)<=80.),"Inclusive horizontal +-80 FOV");}
  auto m=fresh(o);Frame motion=frame(2000000000,{V3(1.5,.0625,0),V3(0,1.5625,0)});motion.capture_poses[1].q=Eigen::Quaterniond(Eigen::AngleAxisd(pi/2,V3::UnitZ()));motion.pose.p=V3(99,99,0);m.ingest(motion);check(m.rays_used==2&&blocked(m,{12,0})&&blocked(m,{0,12}),"Per-return heading and position, not latest processing pose");
  auto translated=fresh(o);Frame moving=frame(2000000000,{V3(3.5625,.0625,0)});moving.capture_poses[0].p.x()=2.;moving.ray_origins[0].x()=2.;translated.ingest(moving);check(blocked(translated,{28,0}),"2m disk follows acquisition-time vehicle, not startup anchor");
 }else if(group=="updates"){
  for(double z:{-100.,-.1,0.,.05,.1,100.}){auto m=fresh(o);m.ingest(frame(2000000000,{V3(1.5625,.0625,z)}));check(blocked(m,{12,0}),"Endpoint always obstacle regardless Z");for(int x=0;x<12;++x)check(!m.cells.at({x,0}).obstacle,"All cells before endpoint are paved");check(!m.cells.count({13,0}),"Never clear behind endpoint");auto s=m.snapshot(pose(2000000000),M4::Identity());auto i=index(s,{12,0});check(s.global[i]==100&&s.local[i]==100&&std::isnan(s.slope_cost[i])&&s.terrain_cost[i]==1.,"Obstacle NaN slope and final100");}
  auto m=fresh(o);m.ingest(frame(2000000000,{V3(1.5625,.0625,0.)}));m.ingest(frame(3000000000,{V3(3.,.0625,-100.)}));check(!blocked(m,{12,0})&&m.cells.at({12,0}).ray_observed==3000000000,"Crossing clears old obstacle and refreshes actual pass time");check(!m.cells.count({24,0})&&!m.cells.count({16,0}),"Outside endpoint ignored and path stops at2m");
  m.ingest(frame(4000000000,{V3(3.,.0625,100.)}));check(m.cells.at({12,0}).ray_observed==4000000000,"Already-free crossing cells are reobserved too");
  m.ingest(frame(5000000000,{V3(1.5625,.0625,-5.)}));check(blocked(m,{12,0}),"Reobserved endpoint turns paved into occupied");
  for(bool reverse:{false,true}){auto same=fresh(o);Points p{V3(3.,.0625,-5.),V3(1.5625,.0625,-5.)};if(reverse)std::reverse(p.begin(),p.end());same.ingest(frame(2000000000,p));check(blocked(same,{12,0}),"Same scan endpoint wins over crossing irrespective order");}
  auto exact=fresh(o);exact.ingest(frame(2000000000,{V3(2.,.0625,0.)}));check(blocked(exact,{16,0}),"Endpoint exactly2m is occupied");
  auto outside=fresh(o);outside.ingest(frame(2000000000,{V3(2.00001,.0625,0.)}));check(!blocked(outside,{16,0})&&!outside.cells.count({16,0}),"Endpoint just beyond2m not occupied");
  auto border=fresh(o);Frame f=frame(2000000000,{V3(2.07,.0625,0)});f.capture_poses[0].p.x()=.0625;f.ray_origins[0].x()=.0625;border.ingest(f);check(border.cells.count({16,0})&&!blocked(border,{16,0}),"Boundary cell with an inside-circle path segment is paved even if endpoint outside");
  auto before=m.provenance();Frame invalid=frame(6000000000,{V3(2,.0625,0),V3(3,.0625,nomad::nan)});fails([&]{m.ingest(invalid);},"Malformed frame rejected");check(m.provenance()==before,"Invalid input cannot partly update map");fails([&]{m.ingest(frame(5000000000,{V3(3,.0625,0)}));},"Duplicate scan rejected");
  auto moved=o;moved.fusion.semantic["moving_front_free_region"]["enabled"]=true;auto protect=fresh(moved);protect.ingest(frame(2000000000,{V3(.25,.0625,-2.)}));auto s=protect.snapshot(pose(2000000000),M4::Identity());check(s.global[index(s,{2,0})]==100,"Endpoint overrides initial and moving-free assumptions");
 }else if(group=="retention"){
  o.fusion.cost_known_cells=false;auto m=fresh(o);m.ingest(frame(2000000000,{V3(1.5625,.0625,0)}));m.ingest(frame(3000000000,{V3(3.,.0625,0)}));const auto old=m.cells.at({12,0});
  m.ingest(frame(70000000000,{V3(.3,1.8,0)}));auto s=m.snapshot(pose(70000000000),M4::Identity());auto i=index(s,{12,0});check(s.global[i]==0&&s.local[i]==0&&s.slope_cost[i]==0&&s.terrain_cost[i]==0,"Unreobserved paved state survives67 seconds in global and local");check(m.cells.at({12,0}).ray_observed==old.ray_observed&&m.cells.at({12,0}).last_point==old.last_point,"No fabricated refresh for unobserved cell");check(s.global[index(s,{24,24})]==-1&&s.local[index(s,{24,24})]==-1,"Never-seen cells remain unknown");
  m.ingest(frame(71000000000,{V3(1.5625,.0625,-5.)}));m.ingest(frame(140000000000,{V3(.3,1.8,0.)}));s=m.snapshot(pose(140000000000),M4::Identity());check(s.global[index(s,{12,0})]==100&&s.local[index(s,{12,0})]==100,"Unreobserved obstacles do not expire");
  auto skipped=fresh(o);skipped.ingest(frame(2000000000,{V3(3.,.0625,0.)}));skipped.ingest(frame(70000000000,{V3(-3.,.0625,0.)}));s=skipped.snapshot(pose(70000000000),M4::Identity());check(s.capture==70000000000&&s.local[index(s,{12,0})]==0,"Outside-FOV scan is not a cell reobservation; snapshot capture remains actual scan time");
  for(auto support:s.support)check(support==0,"No fake camera evidence");
  Pose far=pose(150000000000);far.p.x()=30.;m.prune(make_window(o.fusion,far.p.head<2>()));check(m.cells.empty(),"10m rolling map window still evicts old cells");
 }else if(group=="poses"){
  Pose b=pose();b.p=V3(1,2,900);b.q=Eigen::Quaterniond(Eigen::AngleAxisd(.3,V3::UnitZ())*Eigen::AngleAxisd(.2,V3::UnitX()));M4 mount=M4::Identity();mount.block<3,1>(0,3)=V3(.6,.1,.2);mount.block<3,3>(0,0)=Eigen::AngleAxisd(.4,V3::UnitY()).toRotationMatrix();
  Pose imu=indoor_imu_pose(b,mount,.25);b.p.z()=-100.;auto again=indoor_imu_pose(b,mount,.25);check((imu.p-again.p).norm()==0,"Incoming VIO Z ignored");M3 rb=imu.q.toRotationMatrix()*mount.block<3,3>(0,0).transpose();check((imu.p-rb*mount.block<3,1>(0,3)-V3(1,2,.25)).norm()<1e-12,"Vehicle centre recovers fixed base Z and lever arm");check(Eigen::Quaterniond(rb).angularDistance(b.q)<1e-12,"Full VIO rotation retained");
  std::vector<Pose> history{pose(1000000000),pose(1100000000)};ScanPair pair;pair.stamp_ns=1000000000;pair.points_lidar_m={V3(1,0,.5),V3(1,0,1.)};pair.point_time_offsets_s={.01,.02};auto f=deskew(pair,history,M4::Identity());check(f.capture_poses.size()==2&&f.capture_poses[1].stamp==1020000000,"Deskew exports acquisition-time pose per point");check(f.points[0].z()==.5&&f.points[1].z()==1.,"3D point heights not flattened");
  auto configured=o;configured.camera_forward_axis_imu=mount.block<3,3>(0,0).transpose()*V3::UnitX();IndoorMap m(configured);Pose body=pose(2000000000);body.p=V3(5,.0625,0);auto ip=indoor_imu_pose(body,mount,0);m.initialize(ip,mount);Frame r=frame(2000000000,{V3(7.01,.0625,.3)});r.capture_poses[0]=ip;r.ray_origins[0]=V3(5,.0625,.37);m.ingest(r);check(!blocked(m,{56,0}),"Range centre is vehicle, not offset IMU");
 }else if(group=="config"){
  check(o.fusion.resolution==.125&&o.fusion.retain_last_observed_costs&&o.half_fov_deg==80.&&o.ray_range_m==2.,"Requested constants and persistent local output");
  auto low=o,high=o;low.floor_z=-100.;high.floor_z=100.;auto a=fresh(low),b=fresh(high);Frame f=frame(2000000000,{V3(1.5625,.0625,0.)});a.ingest(f);b.ingest(f);check(a.cells.at({12,0}).obstacle&&b.cells.at({12,0}).obstacle,"Legacy floor Z no longer influences classification");
  for(const auto& v:std::vector<std::pair<std::string,Json>>{{"resolution_m",0},{"camera_forward_axis_imu",{0,0,0}},{"camera_forward_axis_imu",{1,0}},{"oak_max_horizontal_distance_m",2},{"max_pending_frames",0}}){auto bad=j;bad[v.first]=v.second;fails([&]{IndoorOptions::load(bad);},"Invalid setting rejected: "+v.first);}
  auto legacy=j;legacy.erase("lidar_half_fov_deg");check(IndoorOptions::load(legacy).half_fov_deg==80.,"Missing angle defaults to80");
  for(double half:{10.,20.25,40.,55.25,72.5,80.,90.,100.}) {
   auto settings=j;settings["lidar_half_fov_deg"]=half;auto configured=IndoorOptions::load(settings);configured.camera_forward_axis_imu=V3::UnitX();
   check(configured.half_fov_deg==half,"Fractional angle parsed exactly");
   for(double sign:{-1.,1.})for(double delta:{-.001,0.,.001}) {
    auto m=fresh(configured);double angle=sign*(half+delta)*pi/180.;
    V3 hit(1.9*std::cos(angle),.0625+1.9*std::sin(angle),-.8);m.ingest(frame(2000000000,{hit}));
    check(blocked(m,key_of(hit,.125))==(delta<=0.),"Configured sector includes boundary and rejects outside on both sides");
    check(m.provenance().at("horizontal_fov_deg")==2.*half,"Metadata uses selected total angle");
   }
  }
  for(const Json& value:std::vector<Json>{9.999,100.001,true,"80",nullptr}){auto bad=j;bad["lidar_half_fov_deg"]=value;fails([&]{IndoorOptions::load(bad);},"Invalid angle rejected");}
  for(bool enabled:{false,true}) {
   auto settings=j;settings["lock_obstacle_cells"]=enabled;auto configured=IndoorOptions::load(settings);configured.camera_forward_axis_imu=V3::UnitX();configured.fusion.semantic["moving_front_free_region"]["enabled"]=true;
   auto m=fresh(configured);m.ingest(frame(2000000000,{V3(.8125,.0625,-2.)}));const auto original=m.cells.at({6,0});
   for(int64_t t:{3000000000LL,70000000000LL}) {
    m.ingest(frame(t,{V3(3.,.0625,-5.)}));check(blocked(m,{6,0})==enabled,"Toggle selects obstacle retention versus ray clearing");
    const auto& c=m.cells.at({6,0});check(c.ray_observed==t,"Both modes record actual ray reobservation");check(c.last_obstacle==original.last_obstacle&&c.last_point==original.last_point,"Ray does not fabricate endpoint time");
    auto s=m.snapshot(pose(t),M4::Identity());auto i=index(s,{6,0});check(s.global[i]==(enabled?100:0)&&s.local[i]==(enabled?100:0),"Latch survives startup/moving priors and time in global/local");
    check((enabled?std::isnan(s.slope_cost[i]):s.slope_cost[i]==0.)&&s.terrain_cost[i]==(enabled?1.:0.),"Latched obstacle slope unknown and terrain100; unlocked paved0");
    check(m.cells.count({14,0})&&!blocked(m,{14,0}),"Unoccupied ray cells remain paved in either mode");
    if(enabled)check(m.rays_cleared==0&&c.cleared==original.cleared,"Latch never reports obstacle clearing");
   }
   check(m.provenance().at("lock_obstacle_cells")==enabled,"Latch mode metadata");
   auto reset=fresh(configured);check(!blocked(reset,{6,0}),"New map session starts without old obstacles");
   Pose far=pose(80000000000LL);far.p.x()=30.;m.prune(make_window(configured.fusion,far.p.head<2>()));check(!m.cells.count({6,0}),"Existing rolling-window eviction still applies to latched cells");
  }
  auto missing=j;missing.erase("lock_obstacle_cells");check(!IndoorOptions::load(missing).lock_obstacle_cells,"Older configs default latch off");
  for(const Json& value:std::vector<Json>{0,1,"true",nullptr}) {auto bad=j;bad["lock_obstacle_cells"]=value;fails([&]{IndoorOptions::load(bad);},"Latch requires a real boolean");}
  check(std::string(indoor_build_id)=="nomad-indoor-mapper-20261124-obstacle-lock-1","New build identity");
 }else throw std::runtime_error("Unknown group");
 std::cout<<group<<": "<<checks<<" checks passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';return 1;}}

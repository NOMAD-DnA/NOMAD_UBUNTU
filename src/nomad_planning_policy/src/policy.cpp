#include "nomad_planning_cpp/policy.hpp"
#include "nomad_planning_cpp/result_validation.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <sstream>
#include <iomanip>
#include <locale>

namespace nomad_planning_cpp {
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point began){return std::chrono::duration<double>(Clock::now()-began).count();}
ResultGridView view(const Grid& g){return {g.resolution,g.width,g.height,g.origin_x,g.origin_y,g.origin_yaw,g.data};}
// All planning checks use the execution validator and the same geometry.
// The caller supplies the fixed original anchor; no candidate may move it.
bool footprint_clear(const Grid& grid,const Path& path,const Pose& current,
                     const NavigatorConfig& config,const Pose* anchor,bool known_endpoint=false){
  return validate_remaining_result_path(view(grid),path,current,
      config.corridor.allow_unknown_transit,known_endpoint,config.guard_half_length,
      config.guard_half_width,anchor,config.guard_center_offset).empty();
}
double guard_speed_cap(const Grid& grid,const Pose& pose,double steering,
                       const NavigatorConfig& config,double map_age,const Pose* anchor){
  const double distance=observed_distance(grid,pose,steering,config.vehicle,
      config.guard_half_length,config.guard_half_width,2,
      config.corridor.allow_unknown_transit,anchor,config.guard_center_offset);
  return observation_speed_limit(distance,config.guard_latency+std::max(0.,map_age),
      config.guard_deceleration,config.guard_margin);
}
double clamp(double v,double a,double b){return std::max(a,std::min(b,v));}
bool same_point(const Pose& a,const Pose& b){return distance(a,b)<1e-6;}
int index_of(const Grid& g,Cell c){return c.y*g.width+c.x;}
int value_at(const Grid& g,Cell c){return in_bounds(g,c)?g.data[index_of(g,c)]:-1;}
bool known_free(const Grid& g,Cell c){return in_bounds(g,c)&&value_at(g,c)>=0&&value_at(g,c)<80;}
bool known_free(const Grid& g,const Pose& p){return known_free(g,world_to_cell(g,p.x,p.y));}
bool traversable(const Grid& g,Cell c,bool allow_unknown){return in_bounds(g,c)&&value_at(g,c)<80&&(allow_unknown||value_at(g,c)>=0);}
bool traversable(const Grid& g,const Pose& p,bool allow_unknown){return traversable(g,world_to_cell(g,p.x,p.y),allow_unknown);}
std::size_t nearest(const Pose& p,const Path& path){
  std::size_t best=0;double d=kInf;
  for(std::size_t i=0;i<path.size();++i){double q=distance(p,path[i]);if(q<d){best=i;d=q;}}
  return best;
}
double path_length(const Path& p){double d=0;for(std::size_t i=1;i<p.size();++i)d+=distance(p[i-1],p[i]);return d;}
bool same_grid(const Grid& a,const Grid& b){
  return a.frame==b.frame&&a.stamp_ns==b.stamp_ns&&a.width==b.width&&a.height==b.height&&
    a.resolution==b.resolution&&a.origin_x==b.origin_x&&a.origin_y==b.origin_y&&a.origin_yaw==b.origin_yaw;
}
// Exact separable squared Euclidean transform (same centre-distance semantics
// as OpenCV DIST_L2/DIST_MASK_PRECISE). No image/ROS dependency on the board.
std::vector<double> edt_1d(const std::vector<double>& f){
  int n=static_cast<int>(f.size());std::vector<int> v(n);std::vector<double> z(n+1),d(n);
  int k=0;v[0]=0;z[0]=-kInf;z[1]=kInf;
  for(int q=1;q<n;++q){
    double s=0;
    do{s=((f[q]+double(q)*q)-(f[v[k]]+double(v[k])*v[k]))/(2.0*(q-v[k]));
      if(s<=z[k])--k;else break;
    }while(k>=0);
    if(k<0){k=0;v[0]=q;z[0]=-kInf;z[1]=kInf;}
    else{++k;v[k]=q;z[k]=s;z[k+1]=kInf;}
  }
  k=0;for(int q=0;q<n;++q){while(z[k+1]<q)++k;d[q]=(q-v[k])*double(q-v[k])+f[v[k]];}
  return d;
}
std::vector<double> free_distances(const Grid& g,bool unknown_only,bool pad){
  int off=pad?1:0,w=g.width+2*off,h=g.height+2*off;
  const double far=1e15;std::vector<double> a(w*h,0),tmp(w*h),out(g.data.size());
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){
    int value=g.data[y*g.width+x];bool available=unknown_only?value>=0:value>=0&&value<80;
    a[(y+off)*w+x+off]=available?far:0;
  }
  for(int y=0;y<h;++y){std::vector<double> row(a.begin()+y*w,a.begin()+(y+1)*w);auto d=edt_1d(row);std::copy(d.begin(),d.end(),tmp.begin()+y*w);}
  for(int x=0;x<w;++x){std::vector<double> col(h);for(int y=0;y<h;++y)col[y]=tmp[y*w+x];auto d=edt_1d(col);
    for(int y=off;y<h-off;++y)if(x>=off&&x<w-off)out[(y-off)*g.width+x-off]=std::sqrt(d[y])*g.resolution;}
  return out;
}
CostField observed_field(CostField f){for(std::size_t i=0;i<f.grid.data.size();++i)if(f.grid.data[i]<0){f.grid.data[i]=100;f.weights[i]=kInf;}return f;}
CostField traversal_field(CostField f,bool allow_unknown){return allow_unknown?std::move(f):observed_field(std::move(f));}
std::optional<double> path_slope(const Layers& l,const Path& p){
  if(l.slope_cost.size()!=l.grid.data.size()||p.size()<2)return std::nullopt;
  double best=-kInf;
  for(std::size_t i=1;i<p.size();++i){auto a=p[i-1],b=p[i];int n=std::max(1,int(std::ceil(distance(a,b)/(l.grid.resolution*.25))));
    for(int j=0;j<=n;++j){double t=double(j)/n;Cell c=world_to_cell(l.grid,a.x+t*(b.x-a.x),a.y+t*(b.y-a.y));
      if(in_bounds(l.grid,c)){double v=l.slope_cost[index_of(l.grid,c)];if(std::isfinite(v))best=std::max(best,v);}}}
  return std::isfinite(best)?std::optional<double>(best):std::nullopt;
}
// 2026-11-25 goal_cost_scoring: remaining distance to the goal for every cell.
// clearance==nullptr: free-space grid distance (no obstacles), used to cancel the
// 8-neighbour grid anisotropy. Otherwise cells >=80 block (except within 0.3m of the
// start, which may sit inside the min_clearance band) and cells nearer than
// preferred clearance cost more. Unknown cells cost the same as free cells.
std::vector<double> goal_field(const Grid& g,const Pose& start,const Pose& goal,const std::vector<double>* clearance,double preferred,double weight){
  std::vector<double> d(static_cast<std::size_t>(g.width)*g.height,kInf);
  const bool weighted=clearance!=nullptr;
  auto open=[&](Cell c){if(!in_bounds(g,c))return false;if(!weighted||value_at(g,c)<80)return true;return distance(cell_to_world(g,c),start)<=.3;};
  auto factor=[&](int i){return weighted?1+weight*std::pow(std::max(0.,1-(*clearance)[i]/preferred),2):1.;};
  using E=std::pair<double,int>;std::priority_queue<E,std::vector<E>,std::greater<E>> queue;
  const Cell target=world_to_cell(g,goal.x,goal.y);
  if(in_bounds(g,target)){if(!open(target))return d;d[index_of(g,target)]=0;queue.emplace(0,index_of(g,target));}
  else{
    // Seed only map edges that face the goal: the straight line from such an edge
    // leaves the map monotonically, so it never cuts back through observed cells.
    const bool left=target.x<0,right=target.x>=g.width,bottom=target.y<0,top=target.y>=g.height;
    for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x)
      if(((left&&x==0)||(right&&x==g.width-1)||(bottom&&y==0)||(top&&y==g.height-1))&&open(Cell{x,y})){
        const int i=index_of(g,Cell{x,y});d[i]=distance(cell_to_world(g,Cell{x,y}),goal);queue.emplace(d[i],i);}
  }
  while(!queue.empty()){
    const auto [value,i]=queue.top();queue.pop();if(value>d[i])continue;const int x=i%g.width,y=i/g.width;
    for(Cell k:std::vector<Cell>{{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}}){
      const Cell m{x+k.x,y+k.y};if(!open(m))continue;
      if(k.x&&k.y&&(!open(Cell{x+k.x,y})||!open(Cell{x,y+k.y})))continue;
      const int j=index_of(g,m);const double next=value+g.resolution*std::hypot(k.x,k.y)*.5*(factor(i)+factor(j));
      if(next<d[j]){d[j]=next;queue.emplace(next,j);}
    }
  }
  return d;
}
// 2026-11-25 exit_check_length: true when at least one forward arc of the given length
// (left, half-left, straight, half-right, right at v.max_steer) from the pose stays on
// cells below 80 (min_clearance band included). Cells outside the map count as unknown.
bool has_exit(const Grid& g,const Pose& end,const VehicleConfig& v,double length,bool allow_unknown){
  const double k=std::tan(v.max_steer)/v.wheelbase;const int n=std::max(2,static_cast<int>(std::ceil(length/(g.resolution*.5))));
  for(double r:{0.,-.5,.5,-1.,1.}){
    bool ok=true;
    for(int i=1;i<=n&&ok;++i){const Pose p=advance(end,r*k,length*i/n,v.reference_offset);const Cell c=world_to_cell(g,p.x,p.y);
      if(!in_bounds(g,c)){ok=allow_unknown;continue;}const int value=value_at(g,c);ok=value<80&&(allow_unknown||value>=0);}
    if(ok)return true;
  }
  return false;
}
bool support_valid(const Grid& grid,const Grid* support){
  if(!support||!same_grid(grid,*support)||support->data.size()!=grid.data.size())return false;
  return std::all_of(support->data.begin(),support->data.end(),[](int x){return x>=0&&x<=100;});
}
void add_direction_cost(CostField& f,const Grid* support,const Pose& start,const NavigatorConfig& c){
  for(int y=0;y<f.grid.height;++y)for(int x=0;x<f.grid.width;++x){
    int i=y*f.grid.width+x;auto p=cell_to_world(f.grid,{x,y});double dx=p.x-start.x,dy=p.y-start.y;
    double angle=std::abs(wrap(std::atan2(dy,dx)-start.yaw));
    if(f.grid.data[i]<0&&c.unknown_direction_weight>0)
      f.weights[i]+=c.unknown_direction_weight*clamp((angle-c.unknown_direction_free_angle)/(kPi/2-c.unknown_direction_free_angle),0,1)*clamp(1-std::hypot(dx,dy)/c.unknown_direction_radius,0,1);
    if(support&&c.corridor.observation_turn_weight>0)
      f.weights[i]+=c.corridor.observation_turn_weight*clamp((angle-15*kPi/180)/(60*kPi/180),0,1)*(1-support->data[i]/100.0);
  }
}
void add_clearance(CostField& f,double margin){
  if(margin<=0)return;
  Grid mask=f.grid;bool blocked=false;
  for(auto& v:mask.data){bool b=v>=80;blocked=blocked||b;v=b?100:0;}
  if(!blocked)return;
  auto distances=free_distances(mask,false,false);
  for(std::size_t i=0;i<f.grid.data.size();++i)if(f.grid.data[i]>=0&&distances[i]<=margin+1e-7){f.grid.data[i]=std::max<int>(f.grid.data[i],80);f.weights[i]=kInf;}
}
bool near_path_support(const Path& path,const Grid& grid,const Grid& support){
  double travelled=0;int count=0,good=0;
  for(std::size_t i=1;i<path.size();++i){travelled+=distance(path[i-1],path[i]);if(travelled<1.1)continue;if(travelled>2)break;
    ++count;Cell c=world_to_cell(grid,path[i].x,path[i].y);if(in_bounds(grid,c)&&support.data[index_of(grid,c)]>=35)++good;}
  return !count||double(good)/count>=.6;
}
std::set<std::pair<int,int>> footprint_cells(const Grid& g,const Pose& p,double hl,double hw,double offset){
  double c=std::cos(p.yaw),s=std::sin(p.yaw),r=g.resolution;
  const double cx=p.x+offset*c,cy=p.y+offset*s;
  double ex=std::abs(c)*hl+std::abs(s)*hw,ey=std::abs(s)*hl+std::abs(c)*hw,extent=.5*r*(std::abs(c)+std::abs(s));
  std::set<std::pair<int,int>> out;
  int ymin=int(std::floor((cy-ey-g.origin_y)/r)),ymax=int(std::floor((cy+ey-g.origin_y)/r));
  int xmin=int(std::floor((cx-ex-g.origin_x)/r)),xmax=int(std::floor((cx+ex-g.origin_x)/r));
  for(int y=ymin;y<=ymax;++y)for(int x=xmin;x<=xmax;++x){
    double dx=g.origin_x+(x+.5)*r-cx,dy=g.origin_y+(y+.5)*r-cy;
    if(std::abs(c*dx+s*dy)<=hl+extent&&std::abs(-s*dx+c*dy)<=hw+extent)out.emplace(x,y);}
  return out;
}
} // namespace

bool finite_pose(const Pose& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);}
bool route_is_behind(const Pose& p,const Path& path,double lookahead,double margin){
  if(path.empty())return false;
  if(!finite_pose(p)||!std::all_of(path.begin(),path.end(),finite_pose))return true;
  double best=kInf;std::size_t best_i=0;Pose projected;bool found=false;
  for(std::size_t i=1;i<path.size();++i){auto a=path[i-1],b=path[i];double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;if(l2<1e-12)continue;
    double t=clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0,1);Pose q{a.x+t*dx,a.y+t*dy,0};double d=distance(p,q);
    if(d<best){best=d;best_i=i-1;projected=q;found=true;}}
  Pose target=path.back();
  if(found){double remaining=lookahead;Pose current=projected;
    for(std::size_t i=best_i+1;i<path.size();++i){double l=distance(current,path[i]);
      if(l>=remaining&&l>0){double t=remaining/l;target={current.x+t*(path[i].x-current.x),current.y+t*(path[i].y-current.y),0};break;}
      remaining-=l;current=path[i];}}
  return (target.x-p.x)*std::cos(p.yaw)+(target.y-p.y)*std::sin(p.yaw)<-margin;
}
Path known_prefix(const Grid& g,const Pose& start,const Path& path,bool allow_unknown){
  if(path.size()<2||!finite_pose(start)||!std::all_of(path.begin(),path.end(),finite_pose)||!traversable(g,start,allow_unknown))return {};
  double best=kInf;std::size_t index=0;Pose projected;
  for(std::size_t i=1;i<path.size();++i){auto a=path[i-1],b=path[i];double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
    double t=den?clamp(((start.x-a.x)*dx+(start.y-a.y)*dy)/den,0,1):0;Pose q{a.x+t*dx,a.y+t*dy,a.yaw+t*wrap(b.yaw-a.yaw)};
    double error=distance(start,q);if(error<best){best=error;index=i-1;projected=q;}}
  Path output{start},ends{projected};ends.insert(ends.end(),path.begin()+index+1,path.end());
  for(const Pose& b:ends){auto a=output.back();double d=distance(a,b);if(d<1e-9)continue;
    int n=std::max(1,int(std::ceil(d/(g.resolution*.25))));double dyaw=wrap(b.yaw-a.yaw);
    for(int j=1;j<=n;++j){double t=double(j)/n;Pose p{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*dyaw};if(!traversable(g,p,allow_unknown))return allow_unknown?Path{}:output;output.push_back(p);}}
  return output;
}
std::vector<double> rpp_lookaheads(const RppConfig& cfg){
  if(!cfg.lookahead_candidates.empty()){
    if(cfg.lookahead_candidates.size()>32)throw std::invalid_argument("too many RPP lookahead candidates");
    std::set<double> unique;
    for(double x:cfg.lookahead_candidates)
      if(!std::isfinite(x)||x<.1||x>3||!unique.insert(x).second)
        throw std::invalid_argument("RPP lookahead candidates must be unique finite distances in [0.1,3.0]");
    return cfg.lookahead_candidates;
  }
  std::vector<double> lookaheads{cfg.lookahead};
  if(cfg.adaptive_lookahead){std::vector<double> extra;for(double x:{.6,.8,1.,1.2,1.5})if(std::abs(x-cfg.lookahead)>1e-9)extra.push_back(x);
    std::stable_sort(extra.begin(),extra.end(),[&](double a,double b){return std::abs(a-cfg.lookahead)<std::abs(b-cfg.lookahead);});lookaheads.insert(lookaheads.end(),extra.begin(),extra.end());}
  return lookaheads;
}
RppResult regulated_pure_pursuit(const Pose& start,const Path& path,const Grid& grid,const VehicleConfig& v,const RppConfig& cfg,const MotionCheck& permitted,const std::function<std::pair<std::string,std::string>()>& rejection_reason){
  RppResult failed;
  auto reject=[&](double ld,const std::string& cause,const std::string& detail){
    std::ostringstream key,out;key.imbue(std::locale::classic());out.imbue(std::locale::classic());
    key<<std::fixed<<std::setprecision(3)<<ld<<":"<<cause;
    out<<"{Ld="<<std::fixed<<std::setprecision(3)<<ld<<" cause="<<cause;
    if(!detail.empty())out<<" "<<detail;
    out<<"}";
    if(!failed.rejection_key.empty()){failed.rejection_key+=";";failed.rejection_detail+=" ";}
    failed.rejection_key+=key.str();failed.rejection_detail+=out.str();
  };
  auto number=[](double x){std::ostringstream o;o.imbue(std::locale::classic());o<<std::fixed<<std::setprecision(6)<<x;return o.str();};
  validate_grid(grid);validate_vehicle(v);
  if(!finite_pose(start)||path.empty()||!std::all_of(path.begin(),path.end(),finite_pose)){
    reject(0,"INPUT_INVALID","");return failed;}
  if(route_is_behind(start,path)){reject(0,"GEOMETRY_ROUTE_BEHIND","");return failed;}
  if(!std::isfinite(cfg.lookahead)||!std::isfinite(cfg.lateral_accel)||!std::isfinite(cfg.speed)||!std::isfinite(cfg.dt)||!std::isfinite(cfg.horizon)||
      cfg.lookahead<=0||cfg.lateral_accel<=0||cfg.speed<=0||cfg.dt<=0||cfg.horizon<=0||cfg.obstacle_threshold!=80||
      !std::isfinite(cfg.unknown_speed)||cfg.unknown_speed<0||cfg.unknown_speed>cfg.speed)
    throw std::invalid_argument("invalid RPP parameters");
  double limit=std::min(v.max_steer,equivalent_steering_limit(v));
  const auto lookaheads=rpp_lookaheads(cfg);
  Path rear;for(auto p:path)rear.push_back({p.x-v.reference_offset*std::cos(p.yaw),p.y-v.reference_offset*std::sin(p.yaw),0});
  Pose axle{start.x-v.reference_offset*std::cos(start.yaw),start.y-v.reference_offset*std::sin(start.yaw),start.yaw};
  std::size_t closest=nearest(axle,rear);auto field=occupancy_costs(grid);
  for(double lookahead:lookaheads){
    Pose target=rear.back();double remaining=lookahead;
    for(std::size_t i=closest+1;i<rear.size();++i){double length=distance(rear[i-1],rear[i]);if(length>=remaining&&length>0){double t=remaining/length;target={rear[i-1].x+t*(rear[i].x-rear[i-1].x),rear[i-1].y+t*(rear[i].y-rear[i-1].y),0};break;}remaining-=length;}
    double dx=target.x-axle.x,dy=target.y-axle.y;
    double forward=dx*std::cos(start.yaw)+dy*std::sin(start.yaw),lateral=-dx*std::sin(start.yaw)+dy*std::cos(start.yaw);
    auto carrot=[&]{return "xc="+number(forward)+" yc="+number(lateral);};
    if(forward<=0||dx*dx+dy*dy<1e-8){reject(lookahead,"GEOMETRY_CARROT_BEHIND_OR_DEGENERATE",carrot());continue;}
    double steer=std::atan(v.wheelbase*2*lateral/(dx*dx+dy*dy));
    if(std::abs(steer)>limit+1e-5){reject(lookahead,"GEOMETRY_STEERING_LIMIT",carrot()+" steer_deg="+number(steer*180/kPi)+" limit_deg="+number(limit*180/kPi));continue;}
    steer=clamp(steer,-limit,limit);
    double curvature=std::tan(steer)/v.wheelbase,d=distance(path.back(),start);
    double speed=std::min({cfg.speed,std::sqrt(cfg.lateral_accel/std::max(std::abs(curvature),1e-9)),std::max(.05,d*.5)});
    double weight=field.value(start.x,start.y);if(!std::isfinite(weight)){reject(lookahead,"PATH_CURRENT_CELL_BLOCKED_OR_OUTSIDE","");continue;}
    const Cell current_cell=world_to_cell(grid,start.x,start.y);
    const bool current_unknown=grid.data[index_of(grid,current_cell)]<0;
    const double cost_speed=current_unknown&&cfg.unknown_speed>0?cfg.unknown_speed:cfg.speed/std::max(1.,weight);
    speed=std::min(speed,cost_speed);
    double dt=std::min(cfg.dt,grid.resolution*.25/std::max(speed,.01));
    int steps=std::max(1,int(std::ceil(std::min(cfg.horizon,d)/(speed*dt))));
    Path trajectory{start};double score=0;bool valid=true;std::string path_cause,path_detail;
    for(int i=0;i<steps;++i)trajectory.push_back(advance(trajectory.back(),curvature,speed*dt,v.reference_offset));
    for(auto p:trajectory){Cell c=world_to_cell(grid,p.x,p.y);auto cell_detail=[&]{return "cell_x="+std::to_string(c.x)+" cell_y="+std::to_string(c.y)+" world_x="+number(p.x)+" world_y="+number(p.y);};
      if(!in_bounds(grid,c)){valid=false;path_cause="PATH_OUTSIDE_MAP";path_detail=cell_detail();break;}int value=grid.data[index_of(grid,c)];
      if(value>=cfg.obstacle_threshold||(value<0&&!cfg.allow_unknown)){valid=false;path_cause=value>=cfg.obstacle_threshold?"PATH_KNOWN_OBSTACLE":"PATH_UNOBSERVED";path_detail=cell_detail()+" value="+std::to_string(value);break;}score+=value<0?cfg.unknown_penalty:1+2.0*value/79;}
    if(!valid){reject(lookahead,path_cause,path_detail);continue;}
    if(!std::isfinite(field.path_cost(trajectory))){reject(lookahead,"PATH_SEGMENT_BLOCKED","");continue;}
    if(permitted&&!permitted(trajectory,{speed,steer})){
      const auto why=rejection_reason?rejection_reason():std::make_pair(std::string("MOTION_GUARD_REJECTED"),std::string{});
      reject(lookahead,why.first,why.second+" steer_deg="+number(steer*180/kPi)+" speed="+number(speed));continue;}
    return {true,{speed,steer},trajectory,score/trajectory.size(),lookahead,{}, {}};
  }
  return failed;
}
double observed_distance(const Grid& g,const Pose& pose,double steer,const VehicleConfig& v,double hl,double hw,double horizon,bool allow_unknown,const Pose* initial_footprint,double center_offset){
  if(!finite_pose(pose)||!std::isfinite(steer)||hl<=0||hw<=0||horizon<=0)return 0;
  if(!std::isfinite(center_offset)||std::abs(center_offset)>=hl)return 0;
  if(initial_footprint&&!finite_pose(*initial_footprint))return 0;
  // Reproject the original world pose onto this grid. This never follows the
  // current pose, so turning or drifting cannot grow the exempt world region.
  const auto outside_exempt=initial_footprint?footprint_cells(g,*initial_footprint,hl,hw,center_offset):std::set<std::pair<int,int>>{};
  auto initial=footprint_cells(g,pose,hl,hw,center_offset);std::set<std::pair<int,int>> exempt;
  for(auto c:initial)if(in_bounds(g,{c.first,c.second})&&value_at(g,{c.first,c.second})<0)exempt.insert(c);
  auto permitted=[&](const Pose& p){Cell c=world_to_cell(g,p.x,p.y);if(!in_bounds(g,c)||value_at(g,c)>=80)return false;
    for(auto q:footprint_cells(g,p,hl,hw,center_offset)){
      if(!in_bounds(g,{q.first,q.second})){if(!outside_exempt.count(q))return false;continue;}
      const int value=value_at(g,{q.first,q.second});
      if((allow_unknown||initial_footprint)&&value>=80)return false;
      if(!allow_unknown&&value<0&&!exempt.count(q))return false;
    }
    return true;};
  if(!permitted(pose))return 0;
  double step=std::min(.02,g.resolution*.1),walked=0,k=std::tan(steer)/v.wheelbase;Pose p=pose;
  while(walked<horizon-1e-9){double ds=std::min(step,horizon-walked);p=advance(p,k,ds,v.reference_offset);if(!permitted(p))return std::max(0.,walked-step);walked+=ds;}
  return std::max(0.,walked-step);
}
double observation_speed_limit(double d,double latency,double deceleration,double margin){
  if(!std::isfinite(d)||!std::isfinite(latency)||!std::isfinite(deceleration)||!std::isfinite(margin)||deceleration<=0||latency<0||margin<0)return 0;
  return std::max(0.,std::sqrt(std::pow(deceleration*latency,2)+2*deceleration*std::max(0.,d-margin))-deceleration*latency);
}

CorridorSelector::CorridorSelector(CorridorConfig c):config_(c){
  if(!std::isfinite(c.goal_cost_clearance_weight)||c.goal_cost_clearance_weight<0)throw std::invalid_argument("invalid goal cost clearance weight");
  for(double v:{c.final_goal_tolerance,c.goal_approach_radius,c.exit_check_length,c.start_clearance_exempt})if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid goal approach/exit parameters");
  if(!std::isfinite(c.gpp_steer_ratio)||c.gpp_steer_ratio<=0||c.gpp_steer_ratio>1)throw std::invalid_argument("invalid gpp steer ratio");
  for(double v:{c.horizon,c.inset,c.arrival,c.seconds,c.risk_scale,c.reselect_half_width,c.reselect_lookahead,c.waypoint_radius,c.preferred_clearance})if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid corridor dimensions/budget");
  for(double v:{c.goal_weight,c.length_weight,c.narrow_weight,c.heading_weight,c.visit_weight,c.clearance_weight,c.risk_weight,c.bearing_weight,c.final_approach_radius,c.continuation_distance,c.retained_slope_increase,c.observation_turn_weight,c.min_clearance,c.transit_clearance_weight})if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid corridor weight");
  if(c.max_expansions<1)throw std::invalid_argument("invalid corridor expansion budget");
  reset();
}
void CorridorSelector::reset(){active_.reset();final_goal_.reset();active_path_.clear();accepted_slope_.reset();visited_.clear();rejected_.clear();last_probe_=-kInf;previous_blocked_.clear();have_previous_blocked_=false;previous_resolution_=0;detail.clear();}
void CorridorSelector::reject(const Pose& p,double now){if(final_goal_&&same_point(p,*final_goal_)){if(!config_.allow_unknown_transit)return;final_goal_.reset();}rejected_.emplace_back(p,now+3);if(active_&&same_point(*active_,p))active_.reset();}
void CorridorSelector::accept(const CorridorCandidate& c,const Path& path,const Layers& l){if(config_.observed_goal_lock&&c.final&&!path.empty())final_goal_=c.point;active_=c.point;active_path_=path;accepted_slope_=path_slope(l,path);}
std::optional<CorridorCandidate> CorridorSelector::observed_goal_candidate(const CostField& f,const Pose& start,const Pose& goal,double now){
  if(!config_.observed_goal_lock||(!config_.allow_unknown_transit&&final_goal_)||now-last_probe_<2)return std::nullopt;
  last_probe_=now;
  const auto& g=f.grid;const auto end=final_point(g,goal);if(!end)return std::nullopt;
  Cell s=world_to_cell(g,start.x,start.y),target=world_to_cell(g,end->x,end->y);
  if(!traversable(g,s,config_.allow_unknown_transit)||!known_free(g,target)||
     (config_.allow_unknown_transit&&distance(start,*end)>config_.waypoint_radius))return std::nullopt;
  auto began=Clock::now();std::deque<Cell> queue{s};std::vector<bool> seen(g.data.size());seen[index_of(g,s)]=true;int count=1;
  while(!queue.empty()){if(elapsed(began)>.03||count>20000)return std::nullopt;Cell c=queue.front();queue.pop_front();if(c==target)return CorridorCandidate{*end,0,distance(start,*end),0,0,0,true};
    for(Cell d:std::vector<Cell>{{1,0},{-1,0},{0,1},{0,-1}}){Cell n{c.x+d.x,c.y+d.y};if(traversable(g,n,config_.allow_unknown_transit)&&!seen[index_of(g,n)]){seen[index_of(g,n)]=true;++count;queue.push_back(n);}}}
  return std::nullopt;
}
std::optional<Pose> CorridorSelector::final_point(const Grid& g,const Pose& goal)const{
  const Cell c=world_to_cell(g,goal.x,goal.y);
  if(known_free(g,c))return goal;
  // 2026-11-25: a goal inside the min_clearance band (or unknown) is replaced by the
  // nearest known-free cell that still leaves 0.75 cell of search tolerance inside
  // final_goal_tolerance, so the vehicle can still finish within that tolerance.
  const double reach=config_.final_goal_tolerance-g.resolution*.75;
  if(reach<=0)return std::nullopt;
  const int n=static_cast<int>(std::ceil(reach/g.resolution))+1;std::optional<Pose> best;double best_d=kInf;
  for(int dy=-n;dy<=n;++dy)for(int dx=-n;dx<=n;++dx){const Cell m{c.x+dx,c.y+dy};if(!known_free(g,m))continue;
    Pose p=cell_to_world(g,m);const double d=distance(p,goal);if(d<=reach&&d<best_d){best_d=d;p.yaw=goal.yaw;best=p;}}
  return best;
}
bool CorridorSelector::continuation_blocked(const Grid& g,const Pose& start,const Pose& point,const Pose& goal)const{
  double radius=config_.continuation_distance;if(radius<=0||distance(point,goal)<=std::max(radius,config_.final_approach_radius))return false;
  double dx=point.x-start.x,dy=point.y-start.y,norm=std::hypot(dx,dy);if(norm<.1)return false;dx/=norm;dy/=norm;
  Cell s=world_to_cell(g,point.x,point.y);if(!known_free(g,s))return false;
  auto began=Clock::now();std::deque<Cell> queue{s};std::set<std::pair<int,int>> seen{{s.x,s.y}};
  while(!queue.empty()){if(seen.size()>5000||elapsed(began)>.02)return false;Cell c=queue.front();queue.pop_front();
    double px=(c.x-s.x)*g.resolution,py=(c.y-s.y)*g.resolution;if(px*dx+py*dy>=radius*.5)return false;
    for(Cell d:std::vector<Cell>{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}){Cell n{c.x+d.x,c.y+d.y};if(seen.count({n.x,n.y}))continue;
      double vx=(n.x-s.x)*g.resolution,vy=(n.y-s.y)*g.resolution;
      if(std::hypot(vx,vy)>radius||vx*dx+vy*dy<-.3||value_at(g,n)>=80)continue;
      if(d.x&&d.y&&(value_at(g,{c.x+d.x,c.y})>=80||value_at(g,{c.x,c.y+d.y})>=80))continue;
      if(value_at(g,n)<0)return false;
      seen.emplace(n.x,n.y);queue.push_back(n);}}
  return true;
}
bool CorridorSelector::new_direction_block(const Grid& g,const Pose& start,const Pose& goal){
  if(!config_.reselect_new_blocked)return false;
  std::set<std::pair<int,int>> current;std::vector<std::tuple<std::pair<int,int>,Pose>> cells;
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x)if(g.data[y*g.width+x]>=80){Pose p=cell_to_world(g,{x,y});auto key=std::make_pair(int(std::floor(p.x/g.resolution+1e-7)),int(std::floor(p.y/g.resolution+1e-7)));current.insert(key);cells.emplace_back(key,p);}
  auto previous=std::move(previous_blocked_);previous_blocked_=std::move(current);bool prior=have_previous_blocked_&&previous_resolution_==g.resolution;have_previous_blocked_=true;previous_resolution_=g.resolution;
  if(!prior||!active_||final_goal_||distance(*active_,goal)<.3||(config_.final_approach_radius>0&&distance(start,goal)<=config_.final_approach_radius))return false;
  double dx=active_->x-start.x,dy=active_->y-start.y,length=std::hypot(dx,dy);if(length<.1)return false;dx/=length;dy/=length;
  for(auto item:cells){auto key=std::get<0>(item);if(previous.count(key))continue;auto p=std::get<1>(item);double vx=p.x-start.x,vy=p.y-start.y,along=vx*dx+vy*dy,across=std::abs(vx*dy-vy*dx);
    if(along>=0&&along<=length+config_.reselect_lookahead&&across<=config_.reselect_half_width)return true;}
  return false;
}
std::string CorridorSelector::invalidate(const CostField& field,const Layers& layers,const Pose& start,const Pose& goal,double now){
  bool changed=new_direction_block(field.grid,start,goal);if(!active_)return {};
  auto observed=traversal_field(field,config_.allow_unknown_transit);std::string reason;
  if(!known_free(field.grid,*active_)||!std::isfinite(observed.value(active_->x,active_->y)))reason="target_blocked_or_unknown";
  else if(!active_path_.empty()){
    auto i=nearest(start,active_path_);Path remaining{start};remaining.insert(remaining.end(),active_path_.begin()+i,active_path_.end());
    if(!std::isfinite(observed.path_cost(remaining)))reason="remaining_path_blocked_or_unknown";
    else{auto slope=path_slope(layers,remaining);if(slope&&accepted_slope_&&*slope-*accepted_slope_>=config_.retained_slope_increase)reason="remaining_path_slope_increased";}}
  if(reason.empty()&&continuation_blocked(field.grid,start,*active_,goal))reason="target_continuation_blocked";
  if(!reason.empty()){Pose target=*active_;reject(target,now);active_.reset();active_path_.clear();accepted_slope_.reset();}
  else if(changed){active_.reset();active_path_.clear();accepted_slope_.reset();reason="new_blocked_in_target_direction_reselect";}
  return reason;
}
std::optional<std::pair<CorridorCandidate,Path>> CorridorSelector::retained_path(const CostField& f,const Pose& start,const Pose& goal){
  auto decline=[&](std::string why)->std::optional<std::pair<CorridorCandidate,Path>>{detail=std::move(why);return std::nullopt;};
  if(!active_||active_path_.size()<2)return decline("no_reference");
  if(continuation_blocked(f.grid,start,*active_,goal))return decline("target_continuation_blocked");
  bool final=same_point(*active_,goal);
  if(!config_.allow_unknown_transit&&((final_goal_&&!final)||(config_.final_approach_radius>0&&distance(start,goal)<=config_.final_approach_radius&&!final)))return decline("final_target_transition");
  if(config_.allow_unknown_transit&&distance(start,*active_)>config_.waypoint_radius+1e-9)return decline("waypoint_outside_radius");
  if(!final&&distance(start,*active_)<config_.arrival)return decline("waypoint_reached");
  auto i=nearest(start,active_path_);double delta=std::abs(wrap(active_path_[i].yaw-start.yaw));
  if(distance(start,active_path_[i])>.25||delta>15*kPi/180)return decline("pose_alignment_exceeded");
  Path remaining(active_path_.begin()+i,active_path_.end());if(remaining.size()<2)return decline("reference_exhausted");
  auto observed=traversal_field(f,config_.allow_unknown_transit);if(!known_free(f.grid,*active_)||!std::isfinite(observed.value(active_->x,active_->y)))return decline("target_blocked_or_unknown");
  Path checked{start};checked.insert(checked.end(),remaining.begin(),remaining.end());if(!std::isfinite(observed.path_cost(checked)))return decline("remaining_path_blocked_or_unknown");
  detail="retained_safe_path";return std::make_pair(CorridorCandidate{*active_,0,path_length(remaining),0,delta,0,final},std::move(remaining));
}
std::vector<CorridorCandidate> CorridorSelector::candidates(const CostField& f,const Pose& start,const Pose& goal,double now){
  auto began=Clock::now();const auto& g=f.grid;const auto& c=config_;
  if(c.allow_unknown_transit){
    // Do not relabel the input grid. Unknown is a finite transit cost; only
    // actually observed global cells may be selected as stopping waypoints.
    if(!finite_pose(start)||!finite_pose(goal)||!traversable(g,start,true)){detail="start_blocked_or_outside_map";return {};}
    if(final_goal_&&(!same_point(*final_goal_,goal)||!known_free(g,goal)||distance(start,goal)>c.waypoint_radius))final_goal_.reset();
    if(active_&&distance(start,*active_)<c.arrival){visited_.push_back(*active_);if(visited_.size()>100)visited_.erase(visited_.begin());active_.reset();active_path_.clear();}
    rejected_.erase(std::remove_if(rejected_.begin(),rejected_.end(),[&](auto p){return p.second<=now;}),rejected_.end());
    const double separation=std::max(.12,g.resolution*.75);
    auto was_rejected=[&](const Pose& p){return std::any_of(rejected_.begin(),rejected_.end(),[&](auto q){return distance(p,q.first)<separation;});};
    fresh_active_score_=kInf;
    const auto active_cell=active_?std::optional<Cell>(world_to_cell(g,active_->x,active_->y)):std::nullopt;
    if(!skip_active_shortcut_&&c.incremental&&active_&&!active_path_.empty()&&!was_rejected(*active_)&&known_free(g,*active_)&&distance(start,*active_)<=c.waypoint_radius+1e-9){
      auto i=nearest(start,active_path_);const double heading=std::abs(wrap(active_path_[i].yaw-start.yaw));
      Path remaining{start};remaining.insert(remaining.end(),active_path_.begin()+i,active_path_.end());
      if(distance(start,active_path_[i])<=.5&&heading<=kPi/6&&std::isfinite(f.path_cost(remaining))){detail="active_transit_path_revalidated";return {{*active_,-kInf,distance(start,*active_),0,heading,0,same_point(*active_,goal)}};}
    }
    const auto end=final_point(g,goal);
    const Cell s=world_to_cell(g,start.x,start.y),target=end?world_to_cell(g,end->x,end->y):world_to_cell(g,goal.x,goal.y);
    const bool final_reachable=end&&distance(start,*end)<=c.waypoint_radius+1e-9;
    // 2026-11-25 goal_approach_radius: near the goal, waypoints must close in on it.
    const double start_goal=distance(start,goal);const bool approach=c.goal_approach_radius>0&&start_goal<=c.goal_approach_radius;
    std::vector<bool> approaching;
    std::vector<double> costs(g.data.size(),kInf);std::vector<bool> seen(g.data.size());
    using Entry=std::tuple<double,int,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    costs[index_of(g,s)]=0;queue.emplace(0,s.x,s.y);std::vector<CorridorCandidate> options;int expanded=0;bool exhausted=false;
    const double goal_bearing=std::atan2(goal.y-start.y,goal.x-start.x);
    const auto clearance=c.corner_aware?blocked_clearances(g):std::vector<double>{};
    std::vector<double> to_goal,free_to_goal;double start_to_goal=kInf,start_free=kInf;
    if(c.corner_aware&&c.goal_cost_scoring){
      to_goal=goal_field(g,start,goal,&clearance,c.preferred_clearance,c.goal_cost_clearance_weight);
      free_to_goal=goal_field(g,start,goal,nullptr,1,0);start_to_goal=to_goal[index_of(g,s)];start_free=free_to_goal[index_of(g,s)];
    }
    while(!queue.empty()){
      if(expanded>=c.max_expansions||elapsed(began)>c.seconds){exhausted=true;break;}
      auto [cost,x,y]=queue.top();queue.pop();Cell cell{x,y};const int i=index_of(g,cell);if(seen[i])continue;seen[i]=true;++expanded;
      Pose p=cell_to_world(g,cell);
      const bool final=cell==target&&final_reachable;
      if(final)p=*end;
      const double radial=distance(start,p);
      if(known_free(g,cell)&&(final||radial>=std::max(.15,std::min(.35,c.arrival+.05)))&&radial<=c.waypoint_radius+1e-9&&!was_rejected(p)){
        const double bearing=std::atan2(p.y-start.y,p.x-start.x),heading=std::abs(wrap(bearing-start.yaw));
        if(heading<=kPi/2+1e-9){
          double visit=0;for(auto q:visited_)visit+=std::exp(-std::pow(distance(p,q),2)/.25);
          const double error=std::abs(wrap(bearing-goal_bearing));
          // Direction first (10-degree bands), then furthest progress. Keep
          // neighbouring alternatives so one infeasible curve cannot blacklist
          // the entire 1.5m region as the former 1.5m rejection radius did.
          const double clear=c.corner_aware?clearance[i]:0;
          double direction=2*error/kPi;
          if(std::isfinite(start_to_goal)&&std::isfinite(to_goal[i])&&radial>1e-6){
            // Straight-line progress pf vs obstacle-aware progress po (detour = weighted
            // minus free-space distance). Identical scores when no obstacle is involved.
            const double pf=distance(start,goal)-distance(p,goal);
            const double po=pf+std::max(0.,start_to_goal-start_free)-std::max(0.,to_goal[i]-free_to_goal[i]);
            direction=std::max(0.,direction+2/kPi*(std::acos(clamp(po/radial,-1,1))-std::acos(clamp(pf/radial,-1,1))));
          }
          const double score=c.corner_aware?
            direction+(approach?distance(p,goal):c.waypoint_radius-radial)/c.waypoint_radius+.05*visit+.001*cost+
              c.transit_clearance_weight*std::pow(std::max(0.,1-clear/c.preferred_clearance),2):
            10*std::floor(error/(10*kPi/180))+(c.waypoint_radius-radial)/c.waypoint_radius+.05*visit+.001*cost;
          options.push_back({p,score,radial,clear,heading,cost/std::max(radial,g.resolution),final});
          approaching.push_back(final||distance(p,goal)<start_goal-.05);
          if(active_cell&&cell==*active_cell)fresh_active_score_=std::min(fresh_active_score_,score);
        }
      }
      for(Cell d:std::vector<Cell>{{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}}){
        Cell n{x+d.x,y+d.y};if(!traversable(g,n,true))continue;
        if(d.x&&d.y&&(!traversable(g,Cell{x+d.x,y},true)||!traversable(g,Cell{x,y+d.y},true)))continue;
        Pose q=cell_to_world(g,n);if(distance(start,q)>c.waypoint_radius+g.resolution*.01&&!(n==target&&final_reachable))continue;
        const double edge=f.segment(p,q);if(!std::isfinite(edge))continue;
        const int ni=index_of(g,n);if(cost+edge<costs[ni]){costs[ni]=cost+edge;queue.emplace(cost+edge,n.x,n.y);}
      }
    }
    std::vector<std::size_t> order(options.size());for(std::size_t k=0;k<order.size();++k)order[k]=k;
    std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b){return options[a].final!=options[b].final?options[a].final>options[b].final:options[a].score<options[b].score;});
    std::vector<CorridorCandidate> selected;
    // Near the goal, up to 3 candidates that close in on it (goal included) are tried first;
    // the others keep the remaining attempts and budget, e.g. when the vehicle faces away
    // from the goal and has to loop around (no reverse).
    for(int pass=approach?0:1;pass<2;++pass)for(auto k:order){
      if(pass==0&&!approaching[k])continue;
      if(pass==1&&approach&&approaching[k])continue;
      const auto& candidate=options[k];
      if(std::all_of(selected.begin(),selected.end(),[&](auto q){return distance(candidate.point,q.point)>=separation;}))selected.push_back(candidate);
      if(selected.size()>=(pass==0?3u:16u))break;
    }
    detail=selected.empty()?(exhausted?"selection_budget_exhausted":"no_known_waypoint_within_radius"):(exhausted?"partial_transit_candidates":"known_waypoints_unknown_transit");
    return selected;
  }
  if(active_&&continuation_blocked(g,start,*active_,goal)){reject(*active_,now);active_.reset();active_path_.clear();accepted_slope_.reset();}
  if(final_goal_&&!same_point(*final_goal_,goal))reset();
  if(c.final_approach_radius>0&&distance(start,goal)<=c.final_approach_radius)final_goal_=goal;
  Cell s=world_to_cell(g,start.x,start.y),target=world_to_cell(g,goal.x,goal.y);detail="searching";
  if(!known_free(g,s)){detail="start_not_observed_free";return {};}
  if(final_goal_){if(!known_free(g,target)){detail="final_goal_not_observed_free";return {};}detail="final_goal_locked";return {{goal,0,distance(start,goal),0,0,0,true}};}
  if(c.incremental&&active_&&!active_path_.empty()&&distance(start,*active_)>=c.arrival){
    auto i=nearest(start,active_path_);double delta=std::abs(wrap(active_path_[i].yaw-start.yaw));Path rem{start};rem.insert(rem.end(),active_path_.begin()+i,active_path_.end());
    Cell a=world_to_cell(g,active_->x,active_->y);bool endpoint=known_free(g,a);int radius=int(std::ceil((c.inset+g.resolution)/g.resolution));
    if(endpoint&&distance(*active_,goal)>.3)for(int y=a.y-radius;y<=a.y+radius;++y)for(int x=a.x-radius;x<=a.x+radius;++x)if(std::hypot(x-a.x,y-a.y)*g.resolution<c.inset&&!known_free(g,Cell{x,y}))endpoint=false;
    auto observed=observed_field(f);
    if(endpoint&&distance(start,active_path_[i])<=.5&&delta<=kPi/6&&std::isfinite(observed.path_cost(rem))){detail="active_path_revalidated";return {{*active_,-kInf,path_length(rem),c.inset,delta,0,distance(*active_,goal)<.3}};}}
  auto clearance=free_distances(g,false,true),unknown_distance=free_distances(g,true,true);
  std::vector<double> risk(g.data.size());
  if(c.risk_weight>0)for(std::size_t i=0;i<risk.size();++i)if(g.data[i]>=0&&g.data[i]<80){if(!std::isfinite(f.weights[i])||f.weights[i]<1)throw std::invalid_argument("invalid known-cell planning weights");risk[i]=clamp((f.weights[i]-1)/c.risk_scale,0,4);}
  if(active_&&distance(start,*active_)<c.arrival&&distance(*active_,goal)>.3){visited_.push_back(*active_);if(visited_.size()>100)visited_.erase(visited_.begin());active_.reset();}
  rejected_.erase(std::remove_if(rejected_.begin(),rejected_.end(),[&](auto p){return p.second<=now;}),rejected_.end());
  int n=g.width*g.height;std::vector<double> costs(n,kInf),lengths(n),risks(n);std::vector<Pose> first(n);std::vector<bool> have_first(n),finalized(n);
  using Entry=std::tuple<double,int,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
  costs[index_of(g,s)]=0;queue.emplace(0,s.x,s.y);int expanded=0;bool exhausted=false;
  std::vector<CorridorCandidate> options,short_options;
  while(!queue.empty()){
    if(expanded>=c.max_expansions||elapsed(began)>c.seconds){exhausted=true;if(!c.incremental){detail="selection_budget_exhausted";return {};}break;}
    auto [cost,x,y]=queue.top();queue.pop();Cell cell{x,y};int i=index_of(g,cell);if(finalized[i])continue;finalized[i]=true;++expanded;double d=lengths[i];Pose p=cell_to_world(g,cell);
    if(!(cell==s)&&(d>=.4||cell==target)){
      Pose a=have_first[i]?first[i]:p;double heading=std::abs(wrap(std::atan2(a.y-start.y,a.x-start.x)-start.yaw)),clear=clearance[i];bool final=cell==target;
      bool frontier=c.inset<=unknown_distance[i]&&unknown_distance[i]<=c.inset+1.2,horizon_edge=d>=c.horizon-1;
      if(heading<=kPi/2&&(final||(clear>=c.inset&&(frontier||horizon_edge)))){
        bool rejected=std::any_of(rejected_.begin(),rejected_.end(),[&](auto q){return distance(p,q.first)<1.5;});
        if(!rejected){double visit=0;for(auto q:visited_)visit+=std::exp(-std::pow(distance(p,q),2)/4);double bearing=std::abs(wrap(std::atan2(p.y-start.y,p.x-start.x)-start.yaw));
          double score=c.bearing_weight*bearing/kPi+c.goal_weight*(distance(p,goal)-distance(start,goal))/c.horizon+c.length_weight*d/c.horizon+
            c.narrow_weight*((cost-c.risk_weight*risks[i])/std::max(d,g.resolution)-1)+c.risk_weight*risks[i]/std::max(d,g.resolution)+c.heading_weight*heading/kPi+c.visit_weight*visit-c.clearance_weight*std::min(clear/2,1.);
          // Prefer the original >=2m candidates. Short known frontiers are a
          // fallback for the approved 1.5m startup region, with identical
          // connectivity, width, observation and later motion checks.
          auto& bucket=(final||d>=2)?options:short_options;
          bucket.push_back({final?goal:p,score,d,clear,heading,risks[i]/std::max(d,g.resolution),final});}}}
    for(Cell delta:std::vector<Cell>{{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}}){
      Cell next{x+delta.x,y+delta.y};if(!known_free(g,next)||(delta.x&&delta.y&&(!known_free(g,Cell{x+delta.x,y})||!known_free(g,Cell{x,y+delta.y}))))continue;
      bool gated=false;Pose q=cell_to_world(g,next);for(auto& gate:f.gates)if(gate(p,q)){gated=true;break;}if(gated)continue;
      int ni=index_of(g,next);double ds=g.resolution*std::hypot(delta.x,delta.y),length=d+ds;if(length>c.horizon+.01)continue;
      double edge=ds*.5*(risk[i]+risk[ni]),nc=cost+ds*(1+3*std::pow(std::max(0.,1-clearance[ni]/1.5),2))+c.risk_weight*edge;
      if(nc<costs[ni]){costs[ni]=nc;lengths[ni]=length;risks[ni]=risks[i]+edge;have_first[ni]=true;first[ni]=(d>=.8&&have_first[i])?first[i]:q;queue.emplace(nc,next.x,next.y);}}
  }
  if(active_){Cell a=world_to_cell(g,active_->x,active_->y);if(known_free(g,a)&&finalized[index_of(g,a)]){int i=index_of(g,a);if(clearance[i]>=c.inset||distance(*active_,goal)<.3){
    bool final=distance(*active_,goal)<.3;auto& bucket=(final||lengths[i]>=2)?options:short_options;
    bucket.insert(bucket.begin(),{*active_,-kInf,lengths[i],clearance[i],0,0,final});}}}
  const auto rank=[](const auto& a,const auto& b){return a.final!=b.final?a.final>b.final:a.score<b.score;};
  std::stable_sort(options.begin(),options.end(),rank);
  std::stable_sort(short_options.begin(),short_options.end(),rank);
  std::vector<CorridorCandidate> selected;auto continuation_began=Clock::now();
  bool short_fallback=false;
  for(const auto* bucket:{&options,&short_options}){
    if(!selected.empty())break;
    short_fallback=bucket==&short_options;
    for(auto candidate:*bucket){if(c.continuation_distance>0&&elapsed(continuation_began)>.12){exhausted=true;break;}
      if(continuation_blocked(g,start,candidate.point,goal))continue;
      if(std::all_of(selected.begin(),selected.end(),[&](auto q){return distance(candidate.point,q.point)>1.5;}))selected.push_back(candidate);
      if(selected.size()>=6)break;}}
  detail=exhausted?(selected.empty()?"selection_budget_exhausted":"partial_candidates"):(selected.empty()?(rejected_.empty()?"no_observed_corridor":"candidate_retry_cooldown"):(short_fallback?"short_observed_frontier":"candidates"));return selected;
}

std::vector<CorridorCandidate> CorridorSelector::reissue_candidates(const CostField& f,const Pose& start,const Pose& goal,double now,double* active_score){
  skip_active_shortcut_=true;
  std::vector<CorridorCandidate> out;
  try{out=candidates(f,start,goal,now);}catch(...){skip_active_shortcut_=false;throw;}
  skip_active_shortcut_=false;
  if(active_score)*active_score=active_?fresh_active_score_:kInf;
  return out;
}

Navigator::Navigator(NavigatorConfig c):config_(std::move(c)),selector_(config_.corridor){
  validate_vehicle(config_.vehicle);
  if(config_.frame.empty())throw std::invalid_argument("empty planning frame");
  if(!std::isfinite(config_.guard_center_offset)||std::abs(config_.guard_center_offset)>=config_.guard_half_length)throw std::invalid_argument("footprint reference must be inside its longitudinal extent");
  for(double v:{config_.arrival,config_.replan_period,config_.unknown_direction_radius,config_.guard_half_length,config_.guard_half_width,config_.guard_deceleration})if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid navigation dimensions");
  for(double v:{config_.unknown_direction_weight,config_.guard_latency,config_.guard_margin})if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid navigation weight");
  if(!std::isfinite(config_.unknown_direction_free_angle)||config_.unknown_direction_free_angle<0||config_.unknown_direction_free_angle>=kPi/2)throw std::invalid_argument("invalid unknown-direction angle");
  if(config_.corridor.allow_unknown_transit)config_.rpp.allow_unknown=true;
  {const auto& k=config_.corridor;
   for(double v:{k.reissue_travel_ratio,k.reissue_min_travel,k.reissue_end_margin,k.reissue_margin,k.reissue_min_interval})
     if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid waypoint reissue parameter");
   if(k.reissue_travel_ratio>1)throw std::invalid_argument("waypoint reissue travel ratio must be in [0,1]");
   if(!std::isfinite(k.reissue_search_seconds)||k.reissue_search_seconds<=0||k.reissue_search_seconds>.8)throw std::invalid_argument("waypoint reissue search seconds must be in (0,0.8]");}
}
void Navigator::adopt_plan(const Path& path,const Layers& layers,const Pose&,double now){
  plan_path_=path;plan_cumulative_.assign(path.size(),0);
  for(std::size_t i=1;i<path.size();++i)plan_cumulative_[i]=plan_cumulative_[i-1]+distance(path[i-1],path[i]);
  progress_index_=0;plan_map_stamp_=layers.grid.stamp_ns;last_reissue_=now;
}
bool Navigator::reissue_due(const Layers& layers,const Pose& pose,double now,bool previous_route_clear){
  const auto& k=config_.corridor;
  if(!k.reissue_enabled||!k.allow_unknown_transit||!goal_||!target_||plan_path_.size()<2)return false;
  if(!previous_route_clear||retry_||std::isfinite(failed_since_))return false;
  if(distance(*target_,*goal_)<.30)return false;                       // final target locked
  if(k.goal_approach_radius>0&&distance(pose,*goal_)<=k.goal_approach_radius)return false;  // 2026-11-25 approach zone
  if(layers.grid.stamp_ns==plan_map_stamp_)return false;              // wait for a new map
  if(now-last_reissue_<k.reissue_min_interval)return false;
  // Progress along the adopted GPP path; search forward only so S-curves cannot jump back.
  const double s_prev=plan_cumulative_[progress_index_];
  std::size_t best=progress_index_;double best_d=distance(pose,plan_path_[best]);
  for(std::size_t i=progress_index_+1;i<plan_path_.size()&&plan_cumulative_[i]<=s_prev+1.0;++i){
    const double d=distance(pose,plan_path_[i]);if(d<best_d){best_d=d;best=i;}}
  progress_index_=best;
  const double total=plan_cumulative_.back(),s=plan_cumulative_[progress_index_];
  reissue_progress_=s;
  if(s<k.reissue_min_travel)return false;
  return s>=std::max(k.reissue_min_travel,k.reissue_travel_ratio*total)||total-s<=k.reissue_end_margin;
}
void Navigator::set_goal(const Pose& goal){if(!finite_pose(goal))throw std::invalid_argument("invalid goal pose");if(goal_&&same_point(*goal_,goal))return;cancel();goal_=goal;}
void Navigator::cancel(){goal_.reset();target_.reset();start_footprint_anchor_.reset();reference_.clear();selector_.reset();last_plan_=-kInf;failed_since_=-kInf;execution_failed_since_=-kInf;retry_=true;}
void Navigator::request_motion_replan(double now){
  retry_=true;last_plan_=-kInf;
  if(config_.corridor.allow_unknown_transit&&target_){selector_.reject(*target_,now);reference_.clear();target_.reset();}
  // Preserve the original retry timing for profiles that have not opted in.
  if(config_.footprint_planning){failed_since_=-kInf;execution_failed_since_=-kInf;}
}
void Navigator::motion_failed(double now){
  if(!std::isfinite(failed_since_))failed_since_=now;
  if(now-failed_since_>=1)request_motion_replan(now);
}
void Navigator::execution_feedback(bool blocked,double now){
  if(!config_.footprint_planning||!goal_||!std::isfinite(now))return;
  if(!blocked){execution_failed_since_=-kInf;return;}
  if(!std::isfinite(execution_failed_since_))execution_failed_since_=now;
  if(now-execution_failed_since_>=1)request_motion_replan(now);
}
NavigationOutput Navigator::stop(std::string reason,bool clear){NavigationOutput out;out.reason=std::move(reason);out.state="STOP";
  if(start_footprint_anchor_){out.start_footprint=*start_footprint_anchor_;out.has_start_footprint=true;}
  if(clear){reference_.clear();target_.reset();selector_.reset();retry_=true;last_plan_=-kInf;out.path_changed=true;}
  else{out.path=reference_;if(target_){out.target=*target_;out.has_target=true;}}return out;}
NavigationOutput Navigator::step(const Layers& layers,const Grid& local,const Pose& pose,double now,const Grid* support){
  const auto began=Clock::now();
  if(!goal_)return stop("choose a goal",true);
  try{
    validate_grid(layers.grid);validate_grid(local);
    if(!finite_pose(pose)||!std::isfinite(now)||layers.grid.frame!=config_.frame||local.frame!=config_.frame||std::abs(layers.grid.origin_yaw)>1e-9||std::abs(local.origin_yaw)>1e-9)return stop("invalid pose/map frame contract",true);
    double global_age=now-layers.grid.stamp_ns*1e-9,local_age=now-local.stamp_ns*1e-9;
    if(global_age<-.05||global_age>3||local_age<-.05||local_age>2)return stop("input map stale",true);
    bool need_support=config_.corridor.observation_guard||config_.corridor.observation_turn_weight>0;
    if(need_support&&!support_valid(layers.grid,support))return stop("matching camera observation support unavailable",true);
    if(config_.allow_initial_footprint_outside&&!start_footprint_anchor_&&
       traversable(layers.grid,pose,config_.corridor.allow_unknown_transit)&&traversable(local,pose,config_.corridor.allow_unknown_transit))start_footprint_anchor_=pose;
    if(distance(pose,*goal_)<config_.arrival){NavigationOutput out=stop("mission reached",true);out.arrived=true;out.state="ARRIVED";return out;}
    CostField field=compose_costs(layers,config_.costs);add_direction_cost(field,need_support?support:nullptr,pose,config_);
    {
      // 2026-11-25 start_clearance_exempt: cells within this radius of the vehicle keep
      // their pre-band state, so a vehicle already inside the min_clearance band can still
      // plan its way out (otherwise "start_blocked_or_outside_map" until moved by hand).
      const double exempt=config_.corridor.start_clearance_exempt;
      std::optional<CostField> before;if(exempt>0&&config_.corridor.min_clearance>0)before=field;
      add_clearance(field,config_.corridor.min_clearance);
      if(before)for(int y=0;y<field.grid.height;++y)for(int x=0;x<field.grid.width;++x){const int i=y*field.grid.width+x;
        if(before->grid.data[i]<80&&field.grid.data[i]!=before->grid.data[i]&&distance(cell_to_world(field.grid,Cell{x,y}),pose)<=exempt){
          field.grid.data[i]=before->grid.data[i];field.weights[i]=before->weights[i];}}
    }
    CostField motion_field=field;
    const Pose* anchor=config_.allow_initial_footprint_outside&&start_footprint_anchor_?&*start_footprint_anchor_:nullptr;
    if(config_.footprint_planning){
      motion_field.gates.push_back([&](const Pose& a,const Pose& b){
        return !footprint_clear(layers.grid,Path{a,b},a,config_,anchor);
      });
    }
    MotionCheck permitted;
    std::pair<std::string,std::string> motion_rejection;
    if(config_.footprint_planning){permitted=[&](const Path& trajectory,const Motion& motion){
      motion_rejection={};
      const auto why=validate_remaining_result_path(view(local),trajectory,pose,
          config_.corridor.allow_unknown_transit,false,config_.guard_half_length,
          config_.guard_half_width,anchor,config_.guard_center_offset);
      if(!why.empty()){motion_rejection={"FOOTPRINT:"+why,""};return false;}
      if(!config_.observed_guard)return true;
      const double cap=guard_speed_cap(local,pose,motion.steering,config_,local_age,anchor);
      if(!(cap>=.02)){std::ostringstream detail;detail.imbue(std::locale::classic());detail<<std::fixed<<std::setprecision(6)
          <<"cap_mps="<<cap<<" min_mps=0.020000 map_age_s="<<local_age;
        motion_rejection={"BRAKING_INSUFFICIENT",detail.str()};return false;}
      return true;
    };}
    // A search/reselection request does not make the previous route unsafe.
    // Retain only a route still traversable on this current global snapshot;
    // the ordinary prefix, RPP and swept-footprint checks below must also pass.
    const Path previous_reference=reference_;
    const auto previous_target=target_;
    bool previous_route_clear=false;
    if(previous_target&&previous_reference.size()>=2&&known_free(layers.grid,*previous_target)&&
       (!config_.corridor.allow_unknown_transit||distance(pose,*previous_target)<=config_.corridor.waypoint_radius+1e-9)&&
       (distance(*previous_target,*goal_)<.30||distance(pose,*previous_target)>=config_.corridor.arrival)){
      const auto at=nearest(pose,previous_reference);Path remaining{pose};
      remaining.insert(remaining.end(),previous_reference.begin()+at,previous_reference.end());
      const auto observed=traversal_field(motion_field,config_.corridor.allow_unknown_transit);
      previous_route_clear=std::isfinite(observed.path_cost(remaining));
    }
    auto gpp_vehicle=config_.vehicle;gpp_vehicle.max_steer*=config_.corridor.gpp_steer_ratio;
    // 2026-11-25: a retained intermediate waypoint that newly faces a dead end (no forward
    // exit arc) is dropped and replanned now instead of being driven into.
    bool dead_end=false;
    if(previous_route_clear&&config_.corridor.exit_check_length>0&&distance(*previous_target,*goal_)>=.30&&
       !has_exit(field.grid,previous_reference.back(),gpp_vehicle,config_.corridor.exit_check_length,config_.corridor.allow_unknown_transit)){
      previous_route_clear=false;dead_end=true;selector_.reject(*previous_target,now);
    }
    auto invalid=selector_.invalidate(motion_field,layers,pose,*goal_,now);
    if(dead_end&&invalid.empty())invalid="waypoint_endpoint_dead_end";
    // The selector may have dropped its active candidate during an earlier
    // reselection. The retained execution reference still needs its own check.
    if(!reference_.empty()&&!previous_route_clear&&invalid.empty())invalid="remaining_reference_no_longer_valid";
    if(config_.corridor.allow_unknown_transit&&!reference_.empty()&&!known_free(layers.grid,reference_.back())){
      invalid="reference_endpoint_no_longer_known_free";selector_.reset();
    }
    if(!invalid.empty()){
      retry_=true;
      if(!previous_route_clear){reference_.clear();target_.reset();}
    }
    bool path_changed=!invalid.empty()&&!previous_route_clear;
    std::string replan_pending=!invalid.empty()&&previous_route_clear?"; replan pending: "+invalid:std::string{};
    // 2026-11-25: reaching an intermediate waypoint (and nothing else invalidating the
    // route) plans the next waypoint in this step instead of stopping until
    // replan_period elapses. Obstacle/unknown invalidation and failed searches keep
    // the previous replan_period behaviour.
    const bool handover=previous_target&&!previous_reference.empty()&&reference_.empty()&&
      invalid=="remaining_reference_no_longer_valid"&&distance(*previous_target,*goal_)>=.30&&
      distance(pose,*previous_target)<config_.corridor.arrival;
    bool due=handover||dead_end||now-last_plan_>=config_.replan_period;
    if(due){
      last_plan_=now;auto probe=selector_.observed_goal_candidate(field,pose,*goal_,now);
      const bool reissue=!probe&&reissue_due(layers,pose,now,previous_route_clear);
      const auto reissue_began=Clock::now();
      std::optional<std::pair<CorridorCandidate,Path>> retained;
      if(!reissue&&!probe&&config_.corridor.retain_safe_path&&!retry_&&!std::isfinite(failed_since_))retained=selector_.retained_path(motion_field,pose,*goal_);
      std::vector<CorridorCandidate> choices;
      ReissueEvent event;
      if(reissue){
        // Evaluated once per new map: keep the active route unless a clearly better waypoint exists.
        plan_map_stamp_=layers.grid.stamp_ns;last_reissue_=now;double active_score=kInf;
        choices=selector_.reissue_candidates(field,pose,*goal_,now,&active_score);
        const double margin=config_.corridor.reissue_margin;
        choices.erase(std::remove_if(choices.begin(),choices.end(),[&](const auto& q){
          return !q.final&&!(q.score+margin<active_score);}),choices.end());
        event.seq=reissue_event_.seq+1;event.from=previous_target?*previous_target:Pose{};event.to=event.from;
        event.progress=reissue_progress_;event.length=plan_cumulative_.empty()?0:plan_cumulative_.back();
        event.active_score=active_score;event.candidates=static_cast<int>(choices.size());
        for(const auto& q:choices)event.best_score=std::min(event.best_score,q.score);
        event.outcome="kept_no_better";
        if(choices.empty()){reference_=previous_reference;target_=previous_target;retained=std::make_pair(CorridorCandidate{},previous_reference);}
      }
      if(retained){if(!reissue){reference_=retained->second;target_=retained->first.point;}}
      else{
        if(!reissue)choices=selector_.candidates(field,pose,*goal_,now);
        if(probe){choices.erase(std::remove_if(choices.begin(),choices.end(),[&](auto q){return same_point(q.point,probe->point);}),choices.end());choices.insert(choices.begin(),*probe);}
        auto observed=traversal_field(motion_field,config_.corridor.allow_unknown_transit);PlanResult result;std::optional<CorridorCandidate> target;
        std::string reason=selector_.detail;
        const auto attempts=std::min<std::size_t>(config_.corridor.allow_unknown_transit?16:3,choices.size());
        // 2026-11-25: first pass at max_steer*gpp_steer_ratio (RPP tracking margin); only when
        // no candidate succeeds, a second pass at full max_steer uses the remaining budget
        // (tight spots / looping back). RPP feasibility is checked for every candidate.
        for(int steer_pass=0;steer_pass<2&&!target;++steer_pass){
        if(steer_pass==1&&config_.corridor.gpp_steer_ratio>=1)break;
        const auto& search_vehicle=steer_pass==0?gpp_vehicle:config_.vehicle;
        for(std::size_t i=0;i<attempts;++i){
          // All candidate searches share one fresh-result deadline. A new
          // candidate must not restart the complete wall-clock allowance.
          // Keep current-pose RPP updates timely while a usable route exists.
          // With a full-steer fallback pending, the first pass keeps 40% of the budget for it.
          const double share=steer_pass==0&&config_.corridor.gpp_steer_ratio<1?.6:1.;
          const double remaining=reissue?share*config_.corridor.reissue_search_seconds-elapsed(reissue_began):share*(previous_route_clear?.15:.98)-elapsed(began);
          if(remaining<=0){reason="candidate search deadline exhausted";break;}
          auto search=config_.search;
          search.seconds=search.seconds>0?std::min(search.seconds,remaining):remaining;
          if(config_.corridor.allow_unknown_transit){
            search.seconds=std::min(search.seconds,std::min(.16,remaining/std::min<double>(4,attempts-i)));
            search.require_known_endpoint=true;search.primitive_length=std::min(.2,layers.grid.resolution);
            search.goal_tolerance=std::min(search.goal_tolerance,layers.grid.resolution*.75);
          }
          auto candidate=choices[i];
          // 2026-11-25: the final goal may finish anywhere within final_goal_tolerance of the goal.
          if(candidate.final&&config_.corridor.final_goal_tolerance>0)
            search.goal_tolerance=std::max(search.goal_tolerance,config_.corridor.final_goal_tolerance-distance(candidate.point,*goal_));
          auto trial=hybrid_astar(observed,pose,candidate.point,search_vehicle,search);
          double length=path_length(trial.path),horizon=candidate.final?std::max(config_.corridor.horizon,candidate.distance):config_.corridor.horizon;
          bool short_path=!trial.path.empty()&&std::isfinite(length)&&length<=std::min(1.5*horizon,1.8*candidate.distance+1);
          bool valid=short_path&&distance(trial.path.back(),candidate.point)<=.30&&std::isfinite(observed.path_cost(trial.path));
          if(valid&&candidate.final&&config_.corridor.final_goal_tolerance>0)valid=distance(trial.path.back(),*goal_)<=config_.corridor.final_goal_tolerance+1e-9;
          if(valid&&config_.corridor.allow_unknown_transit)valid=trial.path.size()>=2&&known_free(layers.grid,trial.path.back())&&distance(pose,trial.path.back())<=config_.corridor.waypoint_radius+1e-9&&
            (candidate.final||length>=std::max(.1,config_.corridor.arrival*.6));
          if(valid&&config_.corridor.observation_guard)valid=near_path_support(trial.path,field.grid,*support);
          // Reject a GPP candidate that cannot produce even its first safe
          // local command. Keep trying alternative waypoints within the budget.
          std::string candidate_rejection;
          if(valid&&!candidate.final&&config_.corridor.exit_check_length>0&&
             !has_exit(field.grid,trial.path.back(),gpp_vehicle,config_.corridor.exit_check_length,config_.corridor.allow_unknown_transit)){
            valid=false;candidate_rejection="candidate endpoint has no forward exit";
          }
          if(valid&&config_.footprint_planning){
            valid=footprint_clear(layers.grid,trial.path,pose,config_,anchor,true);
            if(!valid)candidate_rejection="candidate swept footprint blocked";
            auto prefix=known_prefix(local,pose,trial.path,config_.corridor.allow_unknown_transit);
            valid=valid&&!prefix.empty()&&regulated_pure_pursuit(pose,prefix,local,config_.vehicle,config_.rpp,permitted).success;
            if(!valid&&candidate_rejection.empty())candidate_rejection="candidate has no footprint-safe RPP motion";
          }
          if(valid){result=std::move(trial);target=candidate;if(config_.corridor.allow_unknown_transit&&!candidate.final)target->point=result.path.back();break;}
          reason=!candidate_rejection.empty()?candidate_rejection:(trial.detail.empty()?"invalid candidate path":trial.detail);
          if(steer_pass==(config_.corridor.gpp_steer_ratio<1?1:0)&&(config_.corridor.allow_unknown_transit||!probe||!same_point(candidate.point,probe->point)))selector_.reject(candidate.point,now);
        }
        }
        if(!target&&reissue){
          event.outcome="kept_search_failed";
          // A failed reissue never disturbs the safe active route or its waypoint.
          reference_=previous_reference;target_=previous_target;path_changed=false;
        }else if(!target){
          retry_=true;
          if(previous_route_clear){
            reference_=previous_reference;target_=previous_target;path_changed=false;
            replan_pending="; replan pending: "+reason;
          }else{
            reference_.clear();target_.reset();NavigationOutput out=stop("observed corridor: "+reason,false);out.path_changed=true;return out;
          }
        }else{
          reference_=result.path;target_=target->point;selector_.accept(*target,reference_,layers);retry_=false;path_changed=true;
          adopt_plan(reference_,layers,pose,now);
          if(reissue){event.outcome="switched";event.to=*target_;}
        }
      }
      if(reissue)reissue_event_=event;
    }
    const double computation_age=elapsed(began);
    if(computation_age>1||global_age+computation_age>3||local_age+computation_age>2)
      return stop("solution input age exceeded during bounded search",true);
    if(reference_.empty()||!target_)return stop("waiting for global path retry interval",false);
    Path prefix=config_.observed_guard?known_prefix(local,pose,reference_,config_.corridor.allow_unknown_transit):reference_;
    bool terminal=!prefix.empty()&&!reference_.empty()&&same_point(prefix.back(),reference_.back())&&distance(reference_.back(),*goal_)<.30;
    if(prefix.empty()||path_length(prefix)<(terminal?.01:.1)){
      if(config_.footprint_planning)motion_failed(now);
      return stop("no connected known route prefix",false);
    }
    auto rpp=regulated_pure_pursuit(pose,prefix,local,config_.vehicle,config_.rpp,permitted,[&]{return motion_rejection;});
    if(!rpp.success){motion_failed(now);
      auto out=stop(route_is_behind(pose,prefix)?"forward route points behind; recovery disabled":"no collision-free RPP motion; recovery disabled",false);
      out.rpp_rejection_key=std::move(rpp.rejection_key);out.rpp_rejection_detail=std::move(rpp.rejection_detail);return out;}
    if(!config_.footprint_planning)failed_since_=-kInf;
    double speed=std::min(rpp.motion.speed,std::max(.10,distance(pose,*goal_)*.5));
    double speed_cap=kInf;
    if(config_.observed_guard){
      const double cap=guard_speed_cap(local,pose,rpp.motion.steering,config_,local_age,anchor);
      if(cap<.02){if(config_.footprint_planning)motion_failed(now);return stop("observation boundary",false);}
      speed=std::min(speed,cap);speed_cap=cap;}
    failed_since_=-kInf;
    NavigationOutput out;out.path=reference_;out.trajectory=std::move(rpp.trajectory);out.motion={speed,rpp.motion.steering};out.observed_speed_cap=speed_cap;out.target=*target_;out.has_target=true;out.path_changed=path_changed;out.motion_valid=true;
    if(start_footprint_anchor_){out.start_footprint=*start_footprint_anchor_;out.has_start_footprint=true;}
    out.state="DRIVING";out.reason="Hybrid A* + RPP"+replan_pending;return out;
  }catch(const std::exception& e){return stop(std::string("invalid planning input: ")+e.what(),true);}
}
} // namespace nomad_planning_cpp

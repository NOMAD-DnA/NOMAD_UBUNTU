#include "nomad_planning_cpp/core.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace nomad_planning_cpp {
namespace {
void positive(double v,const char* msg) { if(!std::isfinite(v)||v<=0) throw std::invalid_argument(msg); }
void nonnegative(double v,const char* msg) { if(!std::isfinite(v)||v<0) throw std::invalid_argument(msg); }
void finite_pose(const Pose& p) {
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)) throw std::invalid_argument("Non-finite pose");
}
std::size_t offset(const Grid& g,Cell c) { return static_cast<std::size_t>(c.y)*g.width+c.x; }
// Lower envelope of parabolas, ignoring +inf sites. Exact squared L2 EDT.
std::vector<double> edt_1d(const std::vector<double>& f) {
  const int n=static_cast<int>(f.size()); std::vector<double> d(n,kInf),z(n+1); std::vector<int> v(n); int k=-1;
  for(int q=0;q<n;++q) {
    if(!std::isfinite(f[q])) continue;
    double s=-kInf;
    while(k>=0) {
      const int p=v[k]; s=((f[q]+double(q)*q)-(f[p]+double(p)*p))/(2.*(q-p));
      if(s>z[k]) break;
      --k;
    }
    ++k; v[k]=q; z[k]=(k==0?-kInf:s); z[k+1]=kInf;
  }
  if(k<0) return d;
  int j=0; for(int q=0;q<n;++q) {while(z[j+1]<q) ++j; const double dx=q-v[j]; d[q]=dx*dx+f[v[j]];}
  return d;
}
long long py_round(double v) {
  // Python round() uses ties-to-even; llround() uses ties-away-from-zero.
  const double f=std::floor(v),r=v-f;
  if(r<.5) return static_cast<long long>(f);
  if(r>.5) return static_cast<long long>(f+1);
  const auto i=static_cast<long long>(f); return i%2==0?i:i+1;
}
using Clock=std::chrono::steady_clock;
struct BudgetExceeded {};
struct Budget {
  Clock::time_point deadline; int max_expansions,count=0;
  explicit Budget(const SearchConfig& c):deadline(c.seconds>0?Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c.seconds)):Clock::time_point::max()),max_expansions(c.max_expansions){}
  void time() const {if(Clock::now()>deadline) throw BudgetExceeded{};}
  void tick(){++count; if(count>max_expansions)throw BudgetExceeded{};time();}
};
class CostToGoal {
 const CostField& field_; Budget& budget_;
 using Q=std::tuple<double,int,int>; std::priority_queue<Q,std::vector<Q>,std::greater<Q>> queue_;
 std::vector<double> best_,closed_; std::vector<bool> done_;
 public:
 CostToGoal(const CostField& f,const Pose& goal,Budget& b):field_(f),budget_(b),best_(f.weights.size(),kInf),closed_(f.weights.size(),kInf),done_(f.weights.size(),false) {
   const Cell c=world_to_cell(f.grid,goal.x,goal.y);queue_.emplace(0,c.x,c.y);best_[offset(f.grid,c)]=0;
 }
 double operator()(const Pose& p) {
   const auto& g=field_.grid; const Cell target=world_to_cell(g,p.x,p.y);
   if(!in_bounds(g,target))return kInf;
   const auto ti=offset(g,target);
   static constexpr int dxs[]={1,-1,0,0,1,1,-1,-1};
   static constexpr int dys[]={0,0,1,-1,1,-1,1,-1};
   while(!done_[ti]&&!queue_.empty()) {
     const auto [cost,x,y]=queue_.top();queue_.pop();const auto i=offset(g,{x,y}); if(done_[i])continue;
     budget_.time();done_[i]=true;closed_[i]=cost;const double weight=field_.weights[i];
     for(int k=0;k<8;++k) {
       const int dx=dxs[k],dy=dys[k];const Cell n{x+dx,y+dy};if(!in_bounds(g,n))continue;
       const auto ni=offset(g,n);const double other=field_.weights[ni];if(!std::isfinite(other))continue;
       if(dx&&dy&&(!std::isfinite(field_.weights[offset(g,{n.x,y})])||!std::isfinite(field_.weights[offset(g,{x,n.y})])))continue;
       const double candidate=cost+(weight+other)*.5*g.resolution*(dx&&dy?std::sqrt(2.):1.);
       if(candidate<best_[ni]){best_[ni]=candidate;queue_.emplace(candidate,n.x,n.y);}
     }
   }
   return closed_[ti];
 }
};
}  // namespace

double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
double distance(const Pose& a,const Pose& b){return std::hypot(a.x-b.x,a.y-b.y);}
Pose advance(const Pose& p,double k,double d,double ref) {
  finite_pose(p);if(!std::isfinite(k)||!std::isfinite(d)||!std::isfinite(ref))throw std::invalid_argument("Non-finite motion");
  double x=p.x-ref*std::cos(p.yaw),y=p.y-ref*std::sin(p.yaw);const double end=p.yaw+k*d;
  if(std::abs(k)<1e-10){x+=d*std::cos(p.yaw);y+=d*std::sin(p.yaw);}
  else{x+=(std::sin(end)-std::sin(p.yaw))/k;y+=(std::cos(p.yaw)-std::cos(end))/k;}
  return {x+ref*std::cos(end),y+ref*std::sin(end),wrap(end)};
}
void validate_grid(const Grid& g) {
  positive(g.resolution,"Invalid grid resolution");
  if(g.width<=0||g.height<=0||static_cast<std::size_t>(g.width)>std::numeric_limits<std::size_t>::max()/static_cast<std::size_t>(g.height)||g.data.size()!=static_cast<std::size_t>(g.width)*g.height)throw std::invalid_argument("Grid dimensions/data mismatch");
  if(!std::isfinite(g.origin_x)||!std::isfinite(g.origin_y)||!std::isfinite(g.origin_yaw))throw std::invalid_argument("Non-finite grid origin");
  for(auto v:g.data)if(v < -1||v>100)throw std::invalid_argument("Occupancy must be [-1,100]");
}
double equivalent_steering_limit(const VehicleConfig& v){const double t=std::tan(v.joint_limit);return std::atan(v.wheelbase*t/(v.wheelbase+v.track*.5*t));}
void validate_vehicle(const VehicleConfig& v) {
  for(double x:{v.wheelbase,v.track,v.wheel_radius,v.joint_limit,v.max_steer,v.body_length,v.body_width})positive(x,"Invalid vehicle geometry");
  if(v.joint_limit>=kPi/2||v.max_steer>equivalent_steering_limit(v)+1e-9)throw std::invalid_argument("Steering exceeds inner-wheel limit");
  const double ref=(v.rear_steer?-1.:1.)*v.wheelbase*.5;
  if(!std::isfinite(v.reference_offset)||std::abs(v.reference_offset-ref)>1e-8)throw std::invalid_argument("Reference offset disagrees with steering axle");
}
Cell world_to_cell(const Grid& g,double x,double y) {
  const double dx=x-g.origin_x,dy=y-g.origin_y,c=std::cos(g.origin_yaw),s=std::sin(g.origin_yaw);
  const double cx=std::floor((c*dx+s*dy)/g.resolution),cy=std::floor((-s*dx+c*dy)/g.resolution);
  if(!std::isfinite(cx)||!std::isfinite(cy)||cx<std::numeric_limits<int>::min()||cx>std::numeric_limits<int>::max()||cy<std::numeric_limits<int>::min()||cy>std::numeric_limits<int>::max())return {std::numeric_limits<int>::min(),std::numeric_limits<int>::min()};
  return {static_cast<int>(cx),static_cast<int>(cy)};
}
Pose cell_to_world(const Grid& g,Cell p) {
  const double x=(p.x+.5)*g.resolution,y=(p.y+.5)*g.resolution,c=std::cos(g.origin_yaw),s=std::sin(g.origin_yaw);
  return {g.origin_x+c*x-s*y,g.origin_y+s*x+c*y,g.origin_yaw};
}
bool in_bounds(const Grid& g,Cell p){return p.x>=0&&p.y>=0&&p.x<g.width&&p.y<g.height;}
double CostField::value(double x,double y)const {
  const Cell c=world_to_cell(grid,x,y);return in_bounds(grid,c)&&offset(grid,c)<weights.size()?weights[offset(grid,c)]:kInf;
}
double CostField::segment(const Pose& a,const Pose& b)const {
  for(const auto& gate:gates)if(gate(a,b))return kInf;
  const double length=distance(a,b);if(!std::isfinite(length))return kInf;
  const double count_d=std::max(1.,std::ceil(length/(grid.resolution*.25)));
  if(count_d>std::numeric_limits<int>::max())return kInf;
  const int count=static_cast<int>(count_d);double sum=0;Cell prev=world_to_cell(grid,a.x,a.y);
  for(int i=0;i<=count;++i) {
    const double x=a.x+(b.x-a.x)*i/count,y=a.y+(b.y-a.y)*i/count,v=value(x,y);
    if(!std::isfinite(v))return kInf;
    sum+=v;
    const Cell cur=world_to_cell(grid,x,y);
    if(i&&cur.x!=prev.x&&cur.y!=prev.y) {
      for(const Cell c:{Cell{cur.x,prev.y},Cell{prev.x,cur.y}}) {const auto p=cell_to_world(grid,c);if(!std::isfinite(value(p.x,p.y)))return kInf;}
    }
    prev=cur;
  }
  return length*sum/(count+1);
}
double CostField::path_cost(const Path& path)const {
  if(path.empty()||!std::isfinite(value(path.front().x,path.front().y)))return kInf;
  double result=0;for(std::size_t i=1;i<path.size();++i){result+=segment(path[i-1],path[i]);if(!std::isfinite(result))return kInf;}return result;
}
std::vector<double> obstacle_distances(const Grid& g) {
  validate_grid(g);std::vector<double> tmp(g.data.size(),kInf),out(g.data.size(),kInf);
  for(int y=0;y<g.height;++y){std::vector<double> row(g.width,kInf);for(int x=0;x<g.width;++x)if(g.data[offset(g,{x,y})]==100)row[x]=0;row=edt_1d(row);for(int x=0;x<g.width;++x)tmp[offset(g,{x,y})]=row[x];}
  for(int x=0;x<g.width;++x){std::vector<double> col(g.height);for(int y=0;y<g.height;++y)col[y]=tmp[offset(g,{x,y})];col=edt_1d(col);for(int y=0;y<g.height;++y)out[offset(g,{x,y})]=std::sqrt(col[y])*g.resolution;}
  return out;
}
std::vector<double> blocked_clearances(const Grid& g) {
  Grid sources=g;
  for(auto& value:sources.data)value=value>=80?100:0;
  auto d=obstacle_distances(sources);
  // A cell is an area, not a point. Conservative distance to its square.
  for(auto& value:d)value=std::max(0.,value-g.resolution/std::sqrt(2.));
  return d;
}
std::vector<double> proximity_costs(const Grid& g,double radius,double weight) {
  validate_grid(g);positive(radius,"Invalid proximity radius");nonnegative(weight,"Invalid proximity weight");
  auto distances=obstacle_distances(g);for(std::size_t i=0;i<distances.size();++i){const double q=std::max(0.,1.-distances[i]/radius);distances[i]=(g.data[i]<0||g.data[i]>=80)?0.:weight*q*q;}return distances;
}
std::vector<double> cluster_density(const Grid& g,double radius,double influence,double saturation) {
  validate_grid(g);positive(radius,"Invalid cluster radius");nonnegative(influence,"Invalid cluster influence");
  if(!std::isfinite(saturation)||saturation<=0||saturation>1)throw std::invalid_argument("Invalid cluster saturation");
  const double nd=std::ceil(radius/g.resolution);if(nd>100000)throw std::invalid_argument("Cluster radius exceeds supported grid range");
  const int n=static_cast<int>(nd);std::vector<std::pair<int,int>> spans;double denominator=0;
  for(int dy=-n;dy<=n;++dy){int xmax=-1;for(int dx=0;dx<=n;++dx)if(double(dx)*dx*g.resolution*g.resolution+double(dy)*dy*g.resolution*g.resolution<=radius*radius+1e-12)xmax=dx;else break;if(xmax>=0){spans.emplace_back(dy,xmax);denominator+=2*xmax+1;}}
  const auto d=obstacle_distances(g);const int stride=g.width+1;std::vector<int> prefix(static_cast<std::size_t>(stride)*g.height,0);
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){const auto i=offset(g,{x,y});const bool source=influence>0?d[i]<=influence+1e-7:g.data[i]==100;prefix[static_cast<std::size_t>(y)*stride+x+1]=prefix[static_cast<std::size_t>(y)*stride+x]+source;}
  std::vector<double> out(g.data.size(),0);
  for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x){int count=0;for(auto [dy,xmax]:spans){const int yy=y+dy;if(yy<0||yy>=g.height)continue;const int lo=std::max(0,x-xmax),hi=std::min(g.width-1,x+xmax);count+=prefix[static_cast<std::size_t>(yy)*stride+hi+1]-prefix[static_cast<std::size_t>(yy)*stride+lo];}out[offset(g,{x,y})]=std::clamp(count/denominator/saturation,0.,1.);}
  return out;
}
CostField occupancy_costs(const Grid& g,double unknown,const std::vector<double>& prox) {
  validate_grid(g);if(!std::isfinite(unknown)||unknown<1)throw std::invalid_argument("Invalid unknown penalty");
  if(!prox.empty()&&prox.size()!=g.data.size())throw std::invalid_argument("Proximity size mismatch");
  CostField f;f.grid=g;f.weights.resize(g.data.size());
  for(std::size_t i=0;i<g.data.size();++i){const double p=prox.empty()?0:prox[i];nonnegative(p,"Invalid proximity cost");const int v=g.data[i];f.weights[i]=v>=80?kInf:v<0?unknown:1.+4.*v/79.+p;}return f;
}
CostField compose_costs(const Layers& l,const CostConfig& c) {
  const auto& g=l.grid;validate_grid(g);
  nonnegative(c.clearance_weight,"Invalid clearance weight");positive(c.clearance_radius,"Invalid clearance radius");
  if(!std::isfinite(c.clearance_factor)||c.clearance_factor<0)throw std::invalid_argument("Invalid clearance factor");
  std::vector<double> clearance(g.data.size(),0),relative(g.data.size(),1);
  if(c.clearance_weight>0||c.clearance_factor>0){
    const auto d=blocked_clearances(g);
    for(std::size_t i=0;i<d.size();++i){const double q=std::pow(std::max(0.,1-d[i]/c.clearance_radius),2);clearance[i]=c.clearance_weight*q;relative[i]=1+c.clearance_factor*q;}
  }
  if(!c.multi){auto f=occupancy_costs(g,c.unknown_penalty,proximity_costs(g,c.radius,c.obstacle_weight));
    for(std::size_t i=0;i<f.weights.size();++i)if(std::isfinite(f.weights[i]))f.weights[i]=f.weights[i]*relative[i]+clearance[i];
    return f;
  }
  for(double v:{c.obstacle_weight,c.slope_weight,c.terrain_weight,c.unknown_surcharge,c.cluster_weight})nonnegative(v,"Invalid multi-factor weight");
  positive(c.radius,"Invalid proximity radius");positive(c.cluster_radius,"Invalid cluster radius");nonnegative(c.cluster_influence,"Invalid cluster influence");
  if(!std::isfinite(c.missing_cost)||c.missing_cost<0||c.missing_cost>1||!std::isfinite(c.cluster_saturation)||c.cluster_saturation<=0||c.cluster_saturation>1)throw std::invalid_argument("Invalid feature prior/cluster saturation");
  if(l.slope_cost.size()!=g.data.size()||l.terrain_cost.size()!=g.data.size())throw std::invalid_argument("Cost layer size mismatch");
  auto prox=proximity_costs(g,c.radius,1.);std::vector<double> cluster(g.data.size(),0);
  if(c.cluster_weight>0)cluster=cluster_density(g,c.cluster_radius,c.cluster_influence,c.cluster_saturation);
  CostField f;f.grid=g;f.weights.resize(g.data.size());
  for(std::size_t i=0;i<g.data.size();++i){double s=l.slope_cost[i],t=l.terrain_cost[i];for(double a:{s,t})if(std::isinf(a)||(std::isfinite(a)&&(a<0||a>1)))throw std::invalid_argument("Feature must be [0,1] or NaN");if(std::isnan(s))s=c.missing_cost;if(std::isnan(t))t=c.missing_cost;
    double w=1.+c.obstacle_weight*prox[i]+c.slope_weight*s+c.terrain_weight*t;
    if(g.data[i]<0)w=1.+(c.slope_weight+c.terrain_weight)*c.missing_cost+c.unknown_surcharge;
    w=(w+c.cluster_weight*cluster[i])*relative[i]+clearance[i];f.weights[i]=g.data[i]>=80?kInf:w;
  }
  return f;
}
PlanResult hybrid_astar(const CostField& f,const Pose& start,const Pose& goal,const VehicleConfig& v,const SearchConfig& s,const std::function<bool(Cell)>& corridor) {
  validate_grid(f.grid);validate_vehicle(v);finite_pose(start);finite_pose(goal);nonnegative(s.seconds,"Invalid search seconds");positive(s.goal_tolerance,"Invalid goal tolerance");
  nonnegative(s.primitive_length,"Invalid primitive length");
  if(s.max_expansions<0||f.weights.size()!=f.grid.data.size())throw std::invalid_argument("Invalid search field/budget");
  for(std::size_t i=0;i<f.weights.size();++i){const double w=f.weights[i];if(f.grid.data[i]>=80?w!=kInf:!std::isfinite(w)||w<1)throw std::invalid_argument("Invalid cell weights");}
  PlanResult result;
  // The Python core assumes callers validate these. Fail closed at this boundary.
  if(!std::isfinite(f.value(start.x,start.y))||!std::isfinite(f.value(goal.x,goal.y))){result.detail="start or goal blocked/outside map";return result;}
  Budget budget(s);const double spatial=s.primitive_length>0?std::min(std::max(.12,f.grid.resolution*.7),s.primitive_length*.7):std::max(.12,f.grid.resolution*.7),curvature=std::tan(v.max_steer)/v.wheelbase;
  using Key=std::array<long long,3>;auto key=[spatial](const Pose&p){const long long a=py_round(wrap(p.yaw)*72/(2*kPi));return Key{py_round(p.x/spatial),py_round(p.y/spatial),(a%72+72)%72};};
  struct Record{Pose pose;int parent;Path edge;};std::vector<Record> records{{start,-1,{}}};std::map<Key,double> best{{key(start),0}};
  using Q=std::tuple<double,double,int>;std::priority_queue<Q,std::vector<Q>,std::greater<Q>> queue;
  try {
    CostToGoal guidance(f,goal,budget);auto heuristic=[&](const Pose&p){return std::max(distance(p,goal),guidance(p));};
    queue.emplace(heuristic(start),0,0);
    while(!queue.empty()) {
      const auto [_,cost,idx]=queue.top();queue.pop();const Pose pose=records[idx].pose;
      const auto it=best.find(key(pose));if(it==best.end()||cost!=it->second)continue;budget.tick();
      const auto end_cell=world_to_cell(f.grid,pose.x,pose.y);
      const bool known_endpoint=in_bounds(f.grid,end_cell)&&f.grid.data[offset(f.grid,end_cell)]>=0&&f.grid.data[offset(f.grid,end_cell)]<80;
      if(distance(pose,goal)<=s.goal_tolerance&&(!s.require_known_endpoint||known_endpoint)) {
        std::vector<int> chain;for(int i=idx;records[i].parent>=0;i=records[i].parent)chain.push_back(i);
        result.path.push_back(start);for(auto i=chain.rbegin();i!=chain.rend();++i)result.path.insert(result.path.end(),records[*i].edge.begin(),records[*i].edge.end());
        result.detail="HybridAStar";result.expansions=budget.count;result.cost=cost;return result;
      }
      const double length=s.primitive_length>0?s.primitive_length:std::max(.6,f.grid.resolution*2);const int steps=std::max(4,static_cast<int>(std::ceil(length/std::min(.08,f.grid.resolution*.4))));
      for(double ratio:{-1.,-.5,0.,.5,1.}) {
        Path edge;edge.reserve(steps);bool allowed=true;
        for(int i=1;i<=steps;++i){const Pose p=advance(pose,ratio*curvature,length*i/steps,v.reference_offset);edge.push_back(p);if(corridor&&!corridor(world_to_cell(f.grid,p.x,p.y))){allowed=false;break;}}
        if(!allowed)continue;
        Path with_start;with_start.reserve(edge.size()+1);with_start.push_back(pose);with_start.insert(with_start.end(),edge.begin(),edge.end());
        const double ec=f.path_cost(with_start);if(!std::isfinite(ec))continue;
        const Pose endpoint=edge.back();const Key nk=key(endpoint);const double candidate=cost+ec;const auto bit=best.find(nk);
        if(bit==best.end()||candidate+1e-8<bit->second){best[nk]=candidate;const int ni=static_cast<int>(records.size());records.push_back({endpoint,idx,std::move(edge)});queue.emplace(candidate+heuristic(endpoint),candidate,ni);}
      }
    }
    result.detail="no forward kinematic path";
  }catch(const BudgetExceeded&){result.detail="planning budget exhausted";}
  result.expansions=budget.count;return result;
}
}  // namespace nomad_planning_cpp

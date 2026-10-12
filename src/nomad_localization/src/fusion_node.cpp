#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nomad_interfaces/msg/gnss_quality.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <deque>
#include <array>
#include <optional>
#include <cmath>
#include <algorithm>
#include <chrono>
using V3=Eigen::Vector3d;
using Q=Eigen::Quaterniond;
using Odom=nav_msgs::msg::Odometry;
using Fix=sensor_msgs::msg::NavSatFix;
using Quality=nomad_interfaces::msg::GnssQuality;
static double stamp(const builtin_interfaces::msg::Time &s){return s.sec+s.nanosec*1e-9;}
struct Pose {V3 p=V3::Zero(); Q q=Q::Identity();};
static Pose compose(const Pose&a,const Pose&b){return {a.p+a.q*b.p,a.q*b.q};}
static Pose inverse(const Pose&a){Q q=a.q.conjugate();return {-(q*a.p),q};}
static Pose read(const geometry_msgs::msg::Pose &m){return {{m.position.x,m.position.y,m.position.z},{m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z}};}
static geometry_msgs::msg::Pose msg(const Pose&p){geometry_msgs::msg::Pose m;m.position.x=p.p.x();m.position.y=p.p.y();m.position.z=p.p.z();m.orientation.w=p.q.w();m.orientation.x=p.q.x();m.orientation.y=p.q.y();m.orientation.z=p.q.z();return m;}
struct GPS {double t;V3 p;V3 sigma;bool trusted;};
struct Frame {double t;Pose raw,local;std::array<double,3>p;std::array<double,4>q;Pose delta;double dt;std::optional<GPS>gps;};
static Pose estimate(const Frame&f){return {{f.p[0],f.p[1],f.p[2]},{f.q[0],f.q[1],f.q[2],f.q[3]}};}
static void set(Frame&f,const Pose&p){f.p={p.p.x(),p.p.y(),p.p.z()};f.q={p.q.w(),p.q.x(),p.q.y(),p.q.z()};}
// All rotations are Hamilton wxyz; poses map an IMU point into their world.
struct Relative {
 Pose d; double sp,sr;
 template<typename T> bool operator()(const T*pa,const T*qa,const T*pb,const T*qb,T*r)const{
  T qi[4]={qa[0],-qa[1],-qa[2],-qa[3]},diff[3]={pb[0]-pa[0],pb[1]-pa[1],pb[2]-pa[2]},v[3],rel[4];
  ceres::QuaternionRotatePoint(qi,diff,v);ceres::QuaternionProduct(qi,qb,rel);
  T invd[4]={T(d.q.w()),T(-d.q.x()),T(-d.q.y()),T(-d.q.z())},err[4],aa[3];
  ceres::QuaternionProduct(invd,rel,err);ceres::QuaternionToAngleAxis(err,aa);
  for(int k=0;k<3;++k){r[k]=(v[k]-T(d.p[k]))/T(sp);r[k+3]=aa[k]/T(sr);}return true;
 }
};
struct GPSCost {
 V3 p,lever,sigma;
 template<typename T>bool operator()(const T*x,const T*q,T*r)const{
  T l[3]={T(lever.x()),T(lever.y()),T(lever.z())},v[3];ceres::QuaternionRotatePoint(q,l,v);
  for(int k=0;k<3;++k){r[k]=(x[k]+v[k]-T(p[k]))/T(sigma[k]);}
  return true;
 }
};
struct Prior {
 Pose p;double sp,sr;
 template<typename T>bool operator()(const T*x,const T*q,T*r)const{
  T qi[4]={T(p.q.w()),T(-p.q.x()),T(-p.q.y()),T(-p.q.z())},e[4],a[3];
  ceres::QuaternionProduct(qi,q,e);ceres::QuaternionToAngleAxis(e,a);
  for(int k=0;k<3;++k){r[k]=(x[k]-T(p.p[k]))/T(sp);r[k+3]=a[k]/T(sr);}return true;
 }
};
class Fusion:public rclcpp::Node {
 std::deque<Frame>frames_;std::deque<GPS>gps_;std::deque<std::pair<Pose,GPS>>alignment_;
 Quality quality_;double quality_t_=-1;V3 datum_,ecef_;Eigen::Matrix3d enu_;V3 lever_;
 Pose map_raw_,map_local_,last_raw_,local_,last_good_global_;bool have_raw_=false,aligned_=false,quarantine_=false,hard_bad_=false,have_global_=false;
 bool gps_suspect_=false,restart_pending_=false;double last_seen_vio_=-1,last_vio_=-1,last_gps_=-1,last_solve_=-1,bad_since_=-1,last_restart_=-1e9;
 double gps_reacquire_since_=-1;V3 candidate_=V3::Zero();double candidate_t_=-1;
 uint32_t epoch_=0;int restarts_=0,rejected_gps_=0,rejected_vio_=0,solves_=0;double disagreement_=0,solve_ms_=0;
 std::string state_="WAITING",last_reason_="",frame_id_,local_id_,body_id_;
 rclcpp::Publisher<Odom>::SharedPtr odom_pub_,local_pub_,raw_pub_;
 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
 rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
 rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr restart_;
 rclcpp::Subscription<Odom>::SharedPtr vio_sub_;rclcpp::Subscription<Fix>::SharedPtr gps_sub_;
 rclcpp::Subscription<Quality>::SharedPtr quality_sub_;rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr epoch_sub_;
 rclcpp::TimerBase::SharedPtr timer_;nav_msgs::msg::Path path_;
 template<class T>T param(const std::string&n,T v){return declare_parameter<T>(n,v);}
 double get(const std::string&n)const{return get_parameter(n).as_double();}
 static V3 ecef(double lat,double lon,double alt){double p=lat*M_PI/180,l=lon*M_PI/180,s=sin(p),c=cos(p),N=6378137.0/sqrt(1-6.69437999014e-3*s*s);return {(N+alt)*c*cos(l),(N+alt)*c*sin(l),(N*(1-6.69437999014e-3)+alt)*s};}
 bool trusted(double t)const{return quality_t_>=0 && std::abs(t-quality_t_)<=get("quality_max_age_s") && (quality_.valid_fields&Quality::SOLUTION_VALID) && quality_.solution_type==Quality::RTK_FIXED;}
 bool usable_gps(double t)const{return !gps_suspect_&&!gps_.empty()&&t-gps_.back().t<=get("gps_max_age_s")&&t-gps_.back().t>=-0.3;}
 bool healthy_gps(double t)const{return usable_gps(t)&&trusted(t);}
 std::optional<GPS> gps_at(double t)const{
  if(gps_.empty())return {};
  auto it=std::lower_bound(gps_.begin(),gps_.end(),t,[](const GPS&g,double x){return g.t<x;});
  if(it==gps_.begin()){if(std::abs(it->t-t)<0.12)return *it;return {};}
  if(it==gps_.end()){if(t-gps_.back().t<0.12)return gps_.back();return {};}
  const auto &a=*(it-1),&b=*it;if(b.t-a.t>0.5)return {};double w=(t-a.t)/(b.t-a.t);
  return GPS{t,(1-w)*a.p+w*b.p,a.sigma.cwiseMax(b.sigma),a.trusted&&b.trusted};
 }
 void publish(double t,const Pose&global,const Pose&local,const std::string &mode){
  Odom m;m.header.stamp=rclcpp::Time(static_cast<int64_t>(t*1e9));m.header.frame_id=frame_id_;m.child_frame_id=body_id_;m.pose.pose=msg(global);
  // No claim of calibrated posterior covariance. Conservative configured fallback.
  for(int i=0;i<6;++i)m.pose.covariance[7*i]=mode=="FUSED"?1.0:1e6;
  for(int i=0;i<6;++i)m.twist.covariance[7*i]=1e6;
  odom_pub_->publish(m);m.header.frame_id=local_id_;m.pose.pose=msg(local);for(int i=0;i<6;++i)m.pose.covariance[7*i]=1e6;local_pub_->publish(m);
  geometry_msgs::msg::PoseStamped p;p.header.frame_id=frame_id_;p.header.stamp=m.header.stamp;p.pose=msg(global);
  if(path_.poses.empty()||t-stamp(path_.poses.back().header.stamp)>=0.19){path_.poses.push_back(p);if(path_.poses.size()>static_cast<size_t>(get_parameter("path_max_points").as_int()))path_.poses.erase(path_.poses.begin());}
  path_.header=p.header;path_pub_->publish(path_);last_good_global_=global;have_global_=true;
 }
 void status(double t){
  diagnostic_msgs::msg::DiagnosticArray a;a.header.stamp=rclcpp::Time(static_cast<int64_t>(std::max(0.0,t)*1e9));
  diagnostic_msgs::msg::DiagnosticStatus s;s.name="fusion";s.hardware_id="external_vio_gnss";s.message=state_;s.level=state_=="FUSED"?0:1;
  auto kv=[&](std::string k,std::string v){diagnostic_msgs::msg::KeyValue x;x.key=k;x.value=v;s.values.push_back(x);};
  kv("reason",last_reason_);kv("epoch",std::to_string(epoch_));kv("restarts",std::to_string(restarts_));kv("raw_vio_gps_disagreement_m",std::to_string(disagreement_));
  kv("rejected_gps",std::to_string(rejected_gps_));kv("rejected_vio",std::to_string(rejected_vio_));kv("solves",std::to_string(solves_));kv("solve_ms",std::to_string(solve_ms_));kv("gnss_healthy",healthy_gps(t)?"true":"false");
  kv("orientation_valid",(quarantine_||hard_bad_||!aligned_)?"false":"true");kv("covariance","conservative_placeholder_not_calibrated_posterior");a.status.push_back(s);status_pub_->publish(a);
 }
 void reset_epoch(uint32_t epoch){
  if(epoch==epoch_){return;}
  epoch_=epoch;quarantine_=false;hard_bad_=false;frames_.clear();alignment_.clear();have_raw_=false;aligned_=false;bad_since_=-1;last_seen_vio_=-1;last_vio_=-1;last_solve_=-1;restart_pending_=false;
  state_="REINITIALIZING";last_reason_="owned VIO process restarted; waiting for new alignment";status(now().seconds());
 }
 void request_restart(double t){
  quarantine_=true;state_="VIO_QUARANTINED";
  if(!get_parameter("auto_restart").as_bool()||restart_pending_||t-last_restart_<get("restart_cooldown_s")||restarts_>=get_parameter("max_restarts").as_int())return;
  if(!restart_->service_is_ready()){last_reason_="restart unavailable: only a managed VIO can be restarted";return;}
  restart_pending_=true;last_restart_=t;++restarts_;state_="RESTART_REQUESTED";
  restart_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),[this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result){
   auto r=result.get();restart_pending_=false;if(!r->success){last_reason_=r->message;state_="RESTART_FAILED";}else last_reason_="restart accepted; waiting for epoch and VIO initialization";
  });RCLCPP_WARN(get_logger(),"Restart request: %s (disagreement %.2f m)",last_reason_.c_str(),disagreement_);
 }
 void fault(double t,const std::string&reason,bool hard=false){
  ++rejected_vio_;last_reason_=reason;if(bad_since_<0)bad_since_=t;
  if(hard){hard_bad_=true;state_="VIO_SUSPECT";}
  if(healthy_gps(t)&&t-bad_since_>=get("restart_persistence_s"))request_restart(t);
 }
 void gps(const Fix&m){
  double t=stamp(m.header.stamp),age=now().seconds()-t;
  if(m.status.status<0||!std::isfinite(m.latitude)||!std::isfinite(m.longitude)||!std::isfinite(m.altitude)||std::abs(m.latitude)>90||std::abs(m.longitude)>180||age>get("gps_max_age_s")||age<-.5){++rejected_gps_;last_reason_="GNSS invalid or stale";return;}
  V3 p=enu_*(ecef(m.latitude,m.longitude,m.altitude)-ecef_);
  if(last_gps_>=0&&t<=last_gps_){++rejected_gps_;return;}
  V3 sigma(get("gps_horizontal_sigma_m"),get("gps_horizontal_sigma_m"),get("gps_vertical_sigma_m"));
  if(get_parameter("use_reported_gps_covariance").as_bool()&&m.position_covariance_type!=Fix::COVARIANCE_TYPE_UNKNOWN){
   // This version deliberately uses ENU diagonal variances; cross terms are not silently interpreted.
   for(int k=0;k<3;++k){double v=m.position_covariance[7*k];if(std::isfinite(v)&&v>0)sigma[k]=std::max(sigma[k],sqrt(v));}
  }
  if(!trusted(t)){sigma=sigma.cwiseMax(V3::Constant(get("gps_nonfixed_sigma_m")));}
  bool jump=false;
  if(!gps_.empty()){double dt=t-gps_.back().t;if(dt<=get("gps_max_age_s")*2)jump=(p-gps_.back().p).head<2>().norm()>get("gps_max_speed_mps")*dt+get("gps_jump_margin_m")+3*sigma.head<2>().norm();}
  if(jump||gps_suspect_){
   gps_suspect_=true;++rejected_gps_;last_reason_="GNSS continuity gate rejected measurement";
   // Reaccept immediately only on return to predicted VIO, or after a clean
   // continuity interval whose candidate positions agree with the VIO prediction.
   bool agree=false;if(aligned_&&have_raw_)agree=(p-(compose(map_raw_,last_raw_).p+compose(map_raw_,last_raw_).q*lever_)).head<2>().norm()<get("gps_reacquire_agreement_m");
   if(candidate_t_<0||t-candidate_t_>1.0||(p-candidate_).head<2>().norm()>get("gps_max_speed_mps")*(t-candidate_t_)+get("gps_jump_margin_m"))gps_reacquire_since_=t;
   candidate_=p;candidate_t_=t;
   if(!agree||t-gps_reacquire_since_<get("gps_reacquire_s"))return;
   gps_suspect_=false;gps_reacquire_since_=-1;
  }
  last_gps_=t;gps_.push_back({t,p,sigma,trusted(t)});while(gps_.size()>200)gps_.pop_front();
  // Associate delayed GNSS with stored poses at measurement time, not arrival time.
  for(auto &f:frames_){auto g=gps_at(f.t);if(g)f.gps=g;}
  if((quarantine_||hard_bad_||!aligned_)&&have_global_){Pose p=last_good_global_;p.p=gps_.back().p-p.q*lever_;publish(t,p,local_,"GNSS_ONLY");state_="GNSS_ONLY_ORIENTATION_HELD";}
  status(t);
 }
 bool align(double t,const Pose&raw){
  auto g=gps_at(t);if(!g||gps_suspect_||(get_parameter("alignment_require_rtk_fixed").as_bool()&&!g->trusted))return false;
  // Sample the alignment at keyframe rate; repeated high-rate VIO must not shrink the motion window.
  if(!alignment_.empty() && t-alignment_.back().second.t<1.0/get("keyframe_hz"))return false;
  alignment_.push_back({raw,*g});while(alignment_.size()>150)alignment_.pop_front();
  if(alignment_.size()<8)return false;
  V3 ar=V3::Zero(),ag=V3::Zero();for(auto &x:alignment_){ar+=x.first.p+x.first.q*lever_;ag+=x.second.p;}ar/=alignment_.size();ag/=alignment_.size();
  double a=0,b=0,span=0;for(auto &x:alignment_){V3 r=x.first.p+x.first.q*lever_-ar,gp=x.second.p-ag;a+=r.x()*gp.x()+r.y()*gp.y();b+=r.x()*gp.y()-r.y()*gp.x();span=std::max(span,(x.second.p-alignment_.front().second.p).head<2>().norm());}
  if(span<get("alignment_min_span_m"))return false;
  Q yaw(Eigen::AngleAxisd(atan2(b,a),V3::UnitZ()));V3 translation=ag-yaw*ar;double err=0;
  for(auto &x:alignment_)err+=(yaw*(x.first.p+x.first.q*lever_)+translation-x.second.p).head<2>().squaredNorm();
  err=sqrt(err/alignment_.size());
  if(err>get("alignment_max_rmse_m")){last_reason_="initial alignment inconsistent; no global pose published";return false;}
  map_raw_={translation,yaw};map_local_=compose(compose(map_raw_,raw),inverse(local_));aligned_=true;quarantine_=false;frames_.clear();bad_since_=-1;
  last_reason_="yaw/translation aligned using VIO and GNSS only";RCLCPP_INFO(get_logger(),"Aligned epoch %u: horizontal fit %.3f m",epoch_,err);return true;
 }
 void vio(const Odom&m){
  double t=stamp(m.header.stamp);last_seen_vio_=std::max(last_seen_vio_,t);Pose raw=read(m.pose.pose);double qnorm=raw.q.norm();
  if(!raw.p.allFinite()||!raw.q.coeffs().allFinite()||qnorm<.9||qnorm>1.1){fault(t,"non-finite/invalid VIO pose",true);return;}raw.q.normalize();
  if(last_vio_>=0&&t<=last_vio_)return;
  if(now().seconds()-t>get("vio_stale_s")||now().seconds()-t<-.5){fault(t,"stale VIO measurement",true);return;}
  if(quarantine_)return;
  double dt=have_raw_?t-last_vio_:0;Pose d=have_raw_?compose(inverse(last_raw_),raw):Pose{};
  if(have_raw_&&(d.p.norm()>get("vio_max_speed_mps")*std::min(dt,0.3)+get("vio_jump_margin_m")||Eigen::AngleAxisd(d.q).angle()>get("vio_max_rotation_rate_rps")*std::min(dt,0.3)+.05)){
   fault(t,"VIO motion continuity failure",true);return;
  }
  if(hard_bad_){hard_bad_=false;bad_since_=-1;last_reason_="transient VIO discontinuity cleared";}
  local_=have_raw_?compose(local_,d):local_;last_raw_=raw;last_vio_=t;have_raw_=true;
  if(!aligned_){state_="ALIGNING";align(t,raw);status(t);if(!aligned_)return;}
  Pose raw_global=compose(map_raw_,raw);Odom rawmsg=m;rawmsg.header.frame_id=frame_id_;rawmsg.pose.pose=msg(raw_global);raw_pub_->publish(rawmsg);
  auto g=gps_at(t);
  if(g&&g->trusted&&healthy_gps(t)){
   disagreement_=(raw_global.p+raw_global.q*lever_-g->p).head<2>().norm();
   if(disagreement_>get("restart_disagreement_m")){fault(t,"persistent raw VIO/GNSS position disagreement");if(quarantine_){status(t);return;}}
   else bad_since_=-1;
  }
  Pose global=compose(map_local_,local_);publish(t,global,local_,usable_gps(t)?"FUSED":"VIO_ONLY");state_=usable_gps(t)?(trusted(t)?"FUSED":"FUSED_GNSS_LOW_QUALITY"):"VIO_ONLY";
  if(frames_.empty()||t-frames_.back().t>=1.0/get("keyframe_hz")){
   Frame f;f.t=t;f.raw=raw;f.local=local_;f.gps=g;f.dt=frames_.empty()?.2:t-frames_.back().t;f.delta=frames_.empty()?Pose{}:compose(inverse(frames_.back().raw),raw);set(f,global);frames_.push_back(f);
   while(frames_.size()>2&&t-frames_.front().t>get("window_s"))frames_.pop_front();
  }
  status(t);
 }
 void tick(){
  double t=now().seconds();
  if(last_seen_vio_>=0&&t-last_seen_vio_>get("vio_stale_s")){fault(t,"VIO output stopped",true);status(t);}
  if((quarantine_||hard_bad_)&&bad_since_>=0&&t-bad_since_>=get("restart_persistence_s")&&healthy_gps(t)){request_restart(t);status(t);}
  if(!aligned_||quarantine_||hard_bad_||frames_.size()<2||t-last_solve_<1.0/get("solve_hz")){return;}
  last_solve_=t;
  ceres::Problem problem;for(auto &f:frames_){problem.AddParameterBlock(f.p.data(),3);problem.AddParameterBlock(f.q.data(),4,new ceres::QuaternionManifold());}
  // Soft boundary retains a previous optimized pose. This is an explicitly
  // approximate bounded pose graph, not an exact probabilistic marginal prior.
  Pose boundary=estimate(frames_.front());problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Prior,6,3,4>(new Prior{boundary,.2,.05}),nullptr,frames_.front().p.data(),frames_.front().q.data());
  for(size_t i=0;i<frames_.size();++i){auto &f=frames_[i];
   if(i){auto &a=frames_[i-1];double dt=std::max(.01,f.dt);problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Relative,6,3,4,3,4>(new Relative{f.delta,get("vio_translation_sigma_per_sqrt_s")*sqrt(dt),get("vio_rotation_sigma_per_sqrt_s")*sqrt(dt)}),new ceres::HuberLoss(get("robust_loss_sigma")),a.p.data(),a.q.data(),f.p.data(),f.q.data());}
   if(f.gps)problem.AddResidualBlock(new ceres::AutoDiffCostFunction<GPSCost,3,3,4>(new GPSCost{f.gps->p,lever_,f.gps->sigma}),new ceres::HuberLoss(get("robust_loss_sigma")),f.p.data(),f.q.data());
  }
  std::vector<Pose> backup;for(auto &f:frames_)backup.push_back(estimate(f));
  ceres::Solver::Options options;options.max_num_iterations=get_parameter("max_iterations").as_int();options.max_solver_time_in_seconds=get("solver_time_s");options.num_threads=1;options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;options.trust_region_strategy_type=ceres::LEVENBERG_MARQUARDT;
  auto begin=std::chrono::steady_clock::now();ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);solve_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  bool valid=summary.IsSolutionUsable();for(auto &f:frames_)valid=valid&&estimate(f).p.allFinite()&&estimate(f).q.coeffs().allFinite();
  if(!valid){for(size_t i=0;i<frames_.size();++i)set(frames_[i],backup[i]);last_reason_="solver failure; last correction retained";return;}
  ++solves_;map_local_=compose(estimate(frames_.back()),inverse(frames_.back().local));
  // Publish a revised window as a separate path, without replacing online history.
  nav_msgs::msg::Path updated;updated.header.frame_id=frame_id_;updated.header.stamp=now();
  for(auto &f:frames_){geometry_msgs::msg::PoseStamped p;p.header=updated.header;p.header.stamp=rclcpp::Time(static_cast<int64_t>(f.t*1e9));p.pose=msg(estimate(f));updated.poses.push_back(p);}window_pub_->publish(updated);
 }
 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr window_pub_;
public:
 Fusion():Node("fusion"){
  auto datum=param<std::vector<double>>("datum",{37,127,0}),lever=param<std::vector<double>>("antenna_in_imu",{-.46,0,.16});
  if(datum.size()!=3||lever.size()!=3){throw std::runtime_error("datum/lever must have 3 values");}
  datum_=V3(datum[0],datum[1],datum[2]);lever_=V3(lever[0],lever[1],lever[2]);ecef_=ecef(datum[0],datum[1],datum[2]);double p=datum[0]*M_PI/180,l=datum[1]*M_PI/180;
  enu_<<-sin(l),cos(l),0,-sin(p)*cos(l),-sin(p)*sin(l),cos(p),cos(p)*cos(l),cos(p)*sin(l),sin(p);
  frame_id_=param<std::string>("frame_id","fusion_map");local_id_=param<std::string>("local_frame_id","fusion_odom");body_id_=param<std::string>("body_frame_id","oak_imu_frame");
  std::vector<std::pair<std::string,double>> defaults={{"window_s",10},{"keyframe_hz",5},{"solve_hz",2},{"solver_time_s",.05},{"vio_translation_sigma_per_sqrt_s",.08},{"vio_rotation_sigma_per_sqrt_s",.02},{"gps_horizontal_sigma_m",.02},{"gps_vertical_sigma_m",.05},{"gps_nonfixed_sigma_m",1.5},{"robust_loss_sigma",3},{"gps_max_speed_mps",5},{"gps_jump_margin_m",.5},{"gps_max_age_s",1},{"gps_reacquire_s",2},{"gps_reacquire_agreement_m",.5},{"quality_max_age_s",.5},{"vio_max_speed_mps",5},{"vio_max_rotation_rate_rps",3},{"vio_jump_margin_m",.15},{"vio_stale_s",2},{"restart_disagreement_m",3},{"restart_persistence_s",3},{"restart_cooldown_s",15},{"alignment_min_span_m",2},{"alignment_max_rmse_m",.5}};
  for(auto &x:defaults){double v=param<double>(x.first,x.second);if(!std::isfinite(v)||v<=0)throw std::runtime_error("positive finite parameter required: "+x.first);}
  if(get("window_s")*get("keyframe_hz")>500)throw std::runtime_error("window too large");
  param<int>("max_iterations",10);param<int>("max_restarts",3);param<int>("path_max_points",2000);param<bool>("auto_restart",true);param<bool>("alignment_require_rtk_fixed",true);param<bool>("use_reported_gps_covariance",true);
  if(get_parameter("max_iterations").as_int()<1||get_parameter("max_restarts").as_int()<0||get_parameter("path_max_points").as_int()<2)throw std::runtime_error("invalid integer limit");
  odom_pub_=create_publisher<Odom>("/fusion/odometry",100);local_pub_=create_publisher<Odom>("/fusion/local_odometry",100);raw_pub_=create_publisher<Odom>("/fusion/raw_vio_aligned",100);
  path_pub_=create_publisher<nav_msgs::msg::Path>("/fusion/path",10);window_pub_=create_publisher<nav_msgs::msg::Path>("/fusion/optimized_window",10);status_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fusion/status",100);
  vio_sub_=create_subscription<Odom>(param<std::string>("vio_topic","/fusion/vio_input"),rclcpp::SensorDataQoS(),[this](Odom::SharedPtr m){vio(*m);});
  gps_sub_=create_subscription<Fix>(param<std::string>("gps_topic","/experiment/gps/fix"),rclcpp::SensorDataQoS(),[this](Fix::SharedPtr m){gps(*m);});
  quality_sub_=create_subscription<Quality>(param<std::string>("quality_topic","/experiment/gps/quality"),100,[this](Quality::SharedPtr m){quality_=*m;quality_t_=stamp(m->header.stamp);});
  epoch_sub_=create_subscription<std_msgs::msg::UInt32>("/fusion/vio_epoch",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::UInt32::SharedPtr m){reset_epoch(m->data);});
  restart_=create_client<std_srvs::srv::Trigger>("/fusion/restart_vio");timer_=create_wall_timer(std::chrono::milliseconds(50),[this](){tick();});
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Fusion>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}

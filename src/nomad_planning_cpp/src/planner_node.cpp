#include "nomad_planning_cpp/policy.hpp"
#include "nomad_planning_cpp/latest_json_writer.hpp"
#include "nomad_planning_cpp/trajectory_writer.hpp"
#include <cstdlib>
#include <locale>
#include "nomad_planning_cpp/result_validation.hpp"
#include "nomad_planning_cpp/lpp_diagnostics.hpp"
#include "nomad_planning_cpp/status_diagnostics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nomad_interfaces/msg/drive_command.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <nomad_interfaces/msg/planning_cost_layers.hpp>
#include <nomad_interfaces/msg/vehicle_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unistd.h>

namespace npc = nomad_planning_cpp;
static constexpr double kPrototypeSteeringLimit = 35.0 * npc::kPi / 180.0;
using Clock = std::chrono::steady_clock;
static double wall_time() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static std::int64_t ns(const builtin_interfaces::msg::Time& s) {return std::int64_t(s.sec)*1000000000LL+s.nanosec;}
static double duration(const builtin_interfaces::msg::Duration& d) {return d.sec+d.nanosec*1e-9;}
static builtin_interfaces::msg::Duration lease(double s) {builtin_interfaces::msg::Duration d;d.sec=static_cast<std::int32_t>(s);d.nanosec=static_cast<std::uint32_t>((s-d.sec)*1e9);return d;}
static std::string json_string(const std::string& s) {std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<c;}o<<'"';return o.str();}
static bool number_field(const std::string& line,const std::string& key,double& value) {std::regex re("\\\""+key+"\\\"\\s*:\\s*([-+]?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][-+]?[0-9]+)?)");std::smatch m;if(!std::regex_search(line,m,re))return false;try{value=std::stod(m[1]);return std::isfinite(value);}catch(...){return false;}}
static std::optional<std::string> string_field(const std::string& line,const std::string& key) {std::regex re("\\\""+key+"\\\"\\s*:\\s*\\\"([^\\\"\\\\]*)\\\"");std::smatch m;if(!std::regex_search(line,m,re))return {};return m[1].str();}
static double yaw_of(const geometry_msgs::msg::Quaternion& q) {const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;if(!std::isfinite(n)||std::abs(n-1)>.05)throw std::invalid_argument("invalid quaternion");return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
static npc::Pose pose_of(const geometry_msgs::msg::Pose& p) {npc::Pose v{p.position.x,p.position.y,yaw_of(p.orientation)};if(!npc::finite_pose(v))throw std::invalid_argument("nonfinite pose");return v;}
static geometry_msgs::msg::Pose ros_pose(const npc::Pose& p) {geometry_msgs::msg::Pose r;r.position.x=p.x;r.position.y=p.y;r.orientation.z=std::sin(p.yaw*.5);r.orientation.w=std::cos(p.yaw*.5);return r;}

class PlannerNode final : public rclcpp::Node {
 using Odom=nav_msgs::msg::Odometry;
 using GridMsg=nav_msgs::msg::OccupancyGrid;
 using LayerMsg=nomad_interfaces::msg::PlanningCostLayers;
 using Status=nomad_interfaces::msg::ModuleStatus;
 using Vehicle=nomad_interfaces::msg::VehicleState;
 using Drive=nomad_interfaces::msg::DriveCommand;
 template<class T> struct Timed {typename T::ConstSharedPtr msg; double received=0;std::string contract_error;};
 struct Inputs {Timed<Odom> odom;Timed<LayerMsg> layers;Timed<GridMsg> local,support;Timed<Status> loc_status,per_status;Timed<Vehicle> vehicle;std::optional<npc::Pose> goal;std::uint64_t revision=0,epoch=0,map_generation=0;};
 struct Result {npc::NavigationOutput nav;npc::Pose pose;std::int64_t stamp=0,gpp_stamp=0,map_stamp=0,local_stamp=0;std::uint64_t revision=0,epoch=0,map_generation=0;double completed=0;bool valid=false;std::string error="NO_INPUT";std::optional<npc::StatusDiagnostic> perception_failure;};
 Inputs input_;Result result_;std::optional<Result> display_reference_,last_valid_result_;
 std::mutex mutex_;std::condition_variable cv_;std::atomic<bool> stopping_{false};
 std::unique_ptr<npc::LatestJsonWriter> gui_output_;
 Odom::ConstSharedPtr last_recorded_odom_;
 std::unique_ptr<npc::TrajectoryWriter> trajectory_output_;
 std::thread worker_,stdio_;npc::NavigatorConfig config_;bool stdio_enabled_,plan_only_,commissioning_,prototype_output_,allow_unmeasured_feedback_;std::atomic<bool> armed_{false};
 struct OutputState {bool enabled=false;std::string inhibit="NO_COMMAND",feedback_source="unavailable";std::int64_t stamp=0;};
 OutputState output_;
 npc::LppDiagnostics lpp_diagnostics_;
 bool preview_camera_heading_=false;
 std::array<double,3> preview_forward_axis_{{1.,0.,0.}};
 std::string frame_,vehicle_frame_;double planning_hz_,odom_timeout_,map_timeout_,status_timeout_,vehicle_timeout_,result_timeout_,future_tolerance_,command_lease_,max_displacement_,max_rotation_;
 std::size_t max_cells_;std::int64_t last_ros_ns_=0;double last_paths_=0;bool last_path_empty_=true,last_local_empty_=true;
 rclcpp::Publisher<Drive>::SharedPtr drive_pub_;
 rclcpp::Publisher<Status>::SharedPtr status_pub_;
 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_pub_,local_pub_;
 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
 rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
 rclcpp::Subscription<LayerMsg>::SharedPtr layers_sub_;
 rclcpp::Subscription<GridMsg>::SharedPtr local_sub_,support_sub_;
 rclcpp::Subscription<Status>::SharedPtr loc_sub_,per_sub_;
 rclcpp::Subscription<Vehicle>::SharedPtr vehicle_sub_;
 rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
 rclcpp::TimerBase::SharedPtr timer_;
 template<class T> T param(const std::string& name,T d){return declare_parameter<T>(name,d);}
 std::string topic(const std::string& key,const std::string& value){return param<std::string>("topics."+key,value);}
 bool need_camera_support()const{return config_.corridor.observation_guard||config_.corridor.observation_turn_weight>0;}
 npc::Pose planning_pose(const geometry_msgs::msg::Pose& message)const{
  auto pose=pose_of(message);
  if(!preview_camera_heading_)return pose;
  const auto& q=message.orientation;
  const double n=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  const double x=q.x/n,y=q.y/n,z=q.z/n,w=q.w/n;
  const auto& a=preview_forward_axis_;
  const double cx=y*a[2]-z*a[1],cy=z*a[0]-x*a[2],cz=x*a[1]-y*a[0];
  const double fx=a[0]+2*w*cx+2*(y*cz-z*cy);
  const double fy=a[1]+2*w*cy+2*(z*cx-x*cz);
  if(!std::isfinite(fx)||!std::isfinite(fy)||std::hypot(fx,fy)<1e-3)throw std::invalid_argument("preview camera heading has no horizontal projection");
  pose.yaw=std::atan2(fy,fx);
  return pose; // Position, input Odometry and TF remain unchanged.
 }
 static bool same_grid_content(const GridMsg& a,const GridMsg& b){
  return a.header.frame_id==b.header.frame_id&&a.info.resolution==b.info.resolution&&a.info.width==b.info.width&&a.info.height==b.info.height&&a.info.origin==b.info.origin&&a.data==b.data;
 }
 static bool same_costs(const std::vector<float>& a,const std::vector<float>& b){
  return a.size()==b.size()&&std::equal(a.begin(),a.end(),b.begin(),[](float x,float y){return x==y||(std::isnan(x)&&std::isnan(y));});
 }
 static bool same_layers_content(const LayerMsg& a,const LayerMsg& b){return same_grid_content(a.grid,b.grid)&&same_costs(a.slope_cost,b.slope_cost)&&same_costs(a.terrain_cost,b.terrain_cost);}
 static bool retained_reference_tail(const npc::Path& previous,const npc::Path& current){
  if(current.empty()||previous.size()<current.size())return false;
  const auto offset=previous.size()-current.size();
  for(std::size_t j=0;j<current.size();++j){const auto& a=previous[offset+j];const auto& b=current[j];if(a.x!=b.x||a.y!=b.y||a.yaw!=b.yaw)return false;}
  return true;
 }
 npc::ResultGridView grid_view(const GridMsg& m)const{return {m.info.resolution,static_cast<int>(m.info.width),static_cast<int>(m.info.height),m.info.origin.position.x,m.info.origin.position.y,yaw_of(m.info.origin.orientation),m.data};}
 std::string grid_contract(const GridMsg& m)const{
  if(m.header.frame_id!=frame_)return "MAP_FRAME";
  if(m.info.width==0||m.info.height==0||std::uint64_t(m.info.width)*m.info.height!=m.data.size()||m.data.size()>max_cells_||m.info.width>INT_MAX||m.info.height>INT_MAX)return "MAP_SHAPE_INVALID";
  if(std::abs(m.info.origin.orientation.x)>1e-5||std::abs(m.info.origin.orientation.y)>1e-5)return "MAP_ORIGIN_NOT_PLANAR";
  try{auto v=grid_view(m);auto why=npc::result_grid_contract(v);if(!why.empty())return why;if(std::abs(v.origin_yaw)>1e-9)return "MAP_ORIGIN_ROTATED";}catch(...){return "MAP_ORIGIN_INVALID";}
  for(auto value:m.data)if(value< -1||value>100)return "MAP_VALUE_INVALID";
  return {};
 }
 template<class T> void update(Timed<T> Inputs::* member,typename T::ConstSharedPtr m){
  std::string contract;
  if constexpr(std::is_same_v<T,LayerMsg>){contract=grid_contract(m->grid);if(contract.empty()&&(m->slope_cost.size()!=m->grid.data.size()||m->terrain_cost.size()!=m->grid.data.size()))contract="Cost layer size mismatch";if(contract.empty())for(const auto* costs:{&m->slope_cost,&m->terrain_cost})for(float v:*costs)if(std::isinf(v)||(std::isfinite(v)&&(v<0||v>1)))contract="Cost layer value invalid";}
  else if constexpr(std::is_same_v<T,GridMsg>)contract=grid_contract(*m);
  std::lock_guard<std::mutex> l(mutex_);auto& previous=input_.*member;bool changed=false;
  if(previous.msg){
   const auto stamp=[](const T& value){if constexpr(std::is_same_v<T,LayerMsg>)return ns(value.grid.header.stamp);else return ns(value.header.stamp);};
   if(stamp(*m)<=stamp(*previous.msg))return; // Never renew age with duplicate/late data.
  }
  if constexpr(std::is_same_v<T,LayerMsg>)changed=!previous.msg||!same_layers_content(*previous.msg,*m);
  else if constexpr(std::is_same_v<T,GridMsg>){if(member==&Inputs::local||need_camera_support())changed=!previous.msg||!same_grid_content(*previous.msg,*m);}
  // Generation wakes re-evaluation; it is not itself evidence of a collision.
  // The command timer checks the remaining route against the latest maps.
  if(changed)++input_.map_generation;
  // Keep the real source/capture stamp even when content is unchanged.
  previous={m,wall_time(),std::move(contract)};cv_.notify_one();
 }
 template<class T> bool fresh(const Timed<T>& v,const builtin_interfaces::msg::Time& stamp,double now_ros,double now_wall,double limit) const {if(!v.msg||v.received<=0||now_wall-v.received>limit)return false;const double source=ns(stamp)*1e-9;return source>0&&source<=now_ros+future_tolerance_&&now_ros-source<=limit;}
 bool status_ready(const Timed<Status>& s,double now_ros,double now_wall) const {return s.msg&&fresh(s,s.msg->header.stamp,now_ros,now_wall,status_timeout_)&&s.msg->state==Status::READY&&duration(s.msg->valid_for)>0&&now_ros-ns(s.msg->header.stamp)*1e-9<=duration(s.msg->valid_for);}
 npc::StatusDiagnostic status_diagnostic(const Timed<Status>& s,double now_ros,double now_wall)const{
  npc::StatusSample sample;sample.present=bool(s.msg);sample.received_wall=s.received;
  sample.evaluated_ros=now_ros;sample.evaluated_wall=now_wall;
  sample.timeout=status_timeout_;sample.future_tolerance=future_tolerance_;
  if(s.msg){sample.state=s.msg->state;sample.stamp_ns=ns(s.msg->header.stamp);sample.valid_for=duration(s.msg->valid_for);sample.detail=s.msg->detail;}
  return npc::diagnose_status(std::move(sample),Status::READY);
 }
 static std::string status_diagnostic_json(const npc::StatusDiagnostic& d){
  const auto& s=d.sample;
  std::size_t detail_size=std::min<std::size_t>(512,s.detail.size());
  while(detail_size>0&&detail_size<s.detail.size()&&(static_cast<unsigned char>(s.detail[detail_size])&0xc0)==0x80)--detail_size;
  const char* state=s.state==Status::WAITING?"WAITING":s.state==Status::READY?"READY":s.state==Status::STOPPED?"STOPPED":s.state==Status::FAULT?"FAULT":"UNKNOWN";
  std::ostringstream o;o<<std::setprecision(17)
   <<"{\"category\":"<<json_string(d.category())<<",\"cause\":"<<json_string(d.cause())
   <<",\"failures\":"<<json_string(d.failure_names())<<",\"present\":"<<(s.present?"true":"false")
   <<",\"state\":"<<s.state<<",\"state_name\":"<<json_string(state)<<",\"stamp_ns\":"<<s.stamp_ns
   <<",\"evaluated_ros\":"<<s.evaluated_ros<<",\"receive_age_s\":"<<(s.present?d.receive_age():-1.)
   <<",\"source_age_s\":"<<(s.present?d.source_age():-1.)<<",\"valid_for_s\":"<<s.valid_for
   <<",\"timeout_s\":"<<s.timeout<<",\"future_tolerance_s\":"<<s.future_tolerance
   <<",\"detail\":"<<json_string(s.detail.substr(0,detail_size))<<",\"detail_truncated\":"<<(detail_size<s.detail.size()?"true":"false")<<'}';
  return o.str();
 }
 std::string input_reason(const Inputs& i,double now_ros,double now_wall) const {
  if(!i.goal)return "NO_GOAL";
  if(!i.odom.msg||!fresh(i.odom,i.odom.msg->header.stamp,now_ros,now_wall,odom_timeout_))return "ODOMETRY_STALE";
  if(i.odom.msg->header.frame_id!=frame_||i.odom.msg->child_frame_id!=vehicle_frame_)return "ODOMETRY_FRAME";
  if(!status_ready(i.loc_status,now_ros,now_wall))return "LOCALIZATION_NOT_READY";
  if(!status_ready(i.per_status,now_ros,now_wall))return "PERCEPTION_NOT_READY";
  if(!i.layers.msg||!fresh(i.layers,i.layers.msg->grid.header.stamp,now_ros,now_wall,map_timeout_))return "LAYERS_STALE";
  if(!i.local.msg||!fresh(i.local,i.local.msg->header.stamp,now_ros,now_wall,map_timeout_))return "LOCAL_COSTMAP_STALE";
  if(i.layers.msg->grid.header.frame_id!=frame_||i.local.msg->header.frame_id!=frame_)return "MAP_FRAME";
  if(!i.layers.contract_error.empty())return i.layers.contract_error;
  if(!i.local.contract_error.empty())return i.local.contract_error;
  const auto& g=i.layers.msg->grid;
  const auto shape_ok=[this](const GridMsg& m){return m.info.width>0&&m.info.height>0&&std::uint64_t(m.info.width)*m.info.height==m.data.size()&&m.data.size()<=max_cells_;};
  if(!shape_ok(g)||!shape_ok(*i.local.msg))return "MAP_SHAPE_INVALID";
  if(i.layers.msg->slope_cost.size()!=g.data.size()||i.layers.msg->terrain_cost.size()!=g.data.size())return "Cost layer size mismatch";
  if(need_camera_support()&&(!i.support.msg||!fresh(i.support,i.support.msg->header.stamp,now_ros,now_wall,map_timeout_)))return "CAMERA_SUPPORT_STALE";
  if(need_camera_support()&&!i.support.contract_error.empty())return i.support.contract_error;
  if(need_camera_support()){const auto& s=*i.support.msg;const auto& a=g.info;const auto& b=s.info;if(s.header.frame_id!=g.header.frame_id||ns(s.header.stamp)!=ns(g.header.stamp)||a.width!=b.width||a.height!=b.height||a.resolution!=b.resolution||a.origin.position.x!=b.origin.position.x||a.origin.position.y!=b.origin.position.y||a.origin.orientation.x!=b.origin.orientation.x||a.origin.orientation.y!=b.origin.orientation.y||a.origin.orientation.z!=b.origin.orientation.z||a.origin.orientation.w!=b.origin.orientation.w)return "CAMERA_SUPPORT_MISMATCH";}
  return {};
 }
 std::string vehicle_reason(const Inputs& i,double now_ros,double now_wall) const {
  if(!i.vehicle.msg||!fresh(i.vehicle,i.vehicle.msg->header.stamp,now_ros,now_wall,vehicle_timeout_))return "VEHICLE_STATE_STALE";
  const auto& v=*i.vehicle.msg;
  if(v.header.frame_id!=vehicle_frame_)return "VEHICLE_FRAME";
  if(v.fault||!v.ready)return "VEHICLE_NOT_READY_OR_FAULT";
  // Valid flags keep their measured-data meaning. The prototype never relabels
  // commanded steering or motor estimates as physical measurements.
  if((v.speed_valid&&!std::isfinite(v.speed))||(v.steering_valid&&!std::isfinite(v.steering_angle)))return "VEHICLE_MEASUREMENT_INVALID";
  if((!v.speed_valid||!v.steering_valid)&&!(prototype_output_&&allow_unmeasured_feedback_))return "VEHICLE_MEASUREMENT_INVALID";
  return {};
 }
 static std::string feedback_source(const Inputs& i){
  if(!i.vehicle.msg)return "unavailable";
  const auto& v=*i.vehicle.msg;
  if(v.speed_valid&&v.steering_valid)return "measured_speed_and_steering";
  if(v.speed_valid)return "measured_speed_unmeasured_steering";
  if(v.steering_valid)return "unmeasured_speed_measured_steering";
  return "unmeasured";
 }
 std::string set_armed(bool requested){
  std::lock_guard<std::mutex> l(mutex_);
  if(!requested){armed_=false;return {};}
  if(plan_only_)return "PLAN_ONLY";
  if(!prototype_output_&&(!config_.vehicle.geometry_confirmed||!commissioning_))return "geometry/calibration not confirmed";
  // One explicit permission lasts for this session. Input/result/vehicle gates
  // still publish STOP while invalid; their recovery must not require rearming.
  armed_=true;return {};
 }
 npc::Grid grid(const GridMsg& m) const {
  const std::uint64_t count=std::uint64_t(m.info.width)*m.info.height;
  if(count==0||count>max_cells_||count!=m.data.size()||m.header.frame_id!=frame_)throw std::invalid_argument("invalid grid dimensions/frame");
  if(std::abs(m.info.origin.orientation.x)>1e-5||std::abs(m.info.origin.orientation.y)>1e-5)throw std::invalid_argument("grid origin must be planar");
  npc::Grid g;g.resolution=m.info.resolution;g.width=static_cast<int>(m.info.width);g.height=static_cast<int>(m.info.height);g.origin_x=m.info.origin.position.x;g.origin_y=m.info.origin.position.y;g.origin_yaw=yaw_of(m.info.origin.orientation);g.frame=m.header.frame_id;g.stamp_ns=ns(m.header.stamp);g.data=m.data;npc::validate_grid(g);return g;
 }
 static bool match(const npc::Grid& a,const npc::Grid& b){return a.frame==b.frame&&a.stamp_ns==b.stamp_ns&&a.width==b.width&&a.height==b.height&&std::abs(a.resolution-b.resolution)<1e-9&&std::abs(a.origin_x-b.origin_x)<1e-7&&std::abs(a.origin_y-b.origin_y)<1e-7&&std::abs(npc::wrap(a.origin_yaw-b.origin_yaw))<1e-7;}
 void json(const std::string& value){if(gui_output_)gui_output_->acknowledge(value);}
 void ack(const std::string& command,bool ok,const std::string& detail){json("{\"type\":\"ack\",\"cmd\":"+json_string(command)+",\"ok\":"+(ok?"true":"false")+",\"detail\":"+json_string(detail)+"}");}
 void goal(const npc::Pose& p,const std::string& f,std::int64_t source_stamp){
  const auto now_ns=get_clock()->now().nanoseconds();
  if(f!=frame_||!npc::finite_pose(p)||source_stamp<=0||now_ns-source_stamp>5000000000LL||source_stamp>now_ns+static_cast<std::int64_t>(future_tolerance_*1e9))throw std::invalid_argument("goal must have fresh stamp and odom frame");
  {std::lock_guard<std::mutex> l(mutex_);input_.goal=p;++input_.revision;result_.valid=false;display_reference_.reset();}
  geometry_msgs::msg::PoseStamped m;m.header.frame_id=frame_;m.header.stamp=rclcpp::Time(source_stamp);m.pose=ros_pose(p);goal_pub_->publish(m);cv_.notify_one();
 }
 void cancel(bool disarm_output=false){{std::lock_guard<std::mutex> l(mutex_);if(disarm_output)armed_=false;input_.goal.reset();++input_.revision;result_.valid=false;display_reference_.reset();}cv_.notify_one();}
 void stdio_loop(){
  std::string buffer;
  while(!stopping_&&rclcpp::ok()){
   pollfd p{STDIN_FILENO,POLLIN,0};const int ready=::poll(&p,1,100);if(ready<0)continue;if(!ready)continue;
   if(p.revents&(POLLERR|POLLNVAL)){cancel(true);break;}
   char chunk[2048];const auto n=::read(STDIN_FILENO,chunk,sizeof(chunk));if(n<=0){cancel(true);json("{\"type\":\"closed\",\"reason\":\"stdin EOF; disarmed\"}");rclcpp::shutdown();break;}
   buffer.append(chunk,static_cast<std::size_t>(n));if(buffer.size()>16384){buffer.clear();ack("invalid",false,"line too long");continue;}
   for(std::size_t pos;(pos=buffer.find('\n'))!=std::string::npos;){auto line=buffer.substr(0,pos);buffer.erase(0,pos+1);auto cmd=string_field(line,"cmd");if(!cmd){ack("invalid",false,"cmd required");continue;}
    try{if(*cmd=="goal"){double x,y,yaw=0;if(!number_field(line,"x",x)||!number_field(line,"y",y))throw std::invalid_argument("finite x/y required");number_field(line,"yaw",yaw);goal({x,y,yaw},string_field(line,"frame").value_or(frame_),get_clock()->now().nanoseconds());ack(*cmd,true,"goal accepted; actuator remains gated");}
    else if(*cmd=="cancel"||*cmd=="stop"){const bool disarm=*cmd=="stop";cancel(disarm);ack(*cmd,true,disarm?"goal cancelled and disarmed":"goal cancelled; output permission unchanged");}
    else if(*cmd=="arm"||*cmd=="disarm"){const auto why=set_armed(*cmd=="arm");ack(*cmd,why.empty(),why.empty()?(*cmd=="arm"?"output permission latched until explicit disarm or session end":"disarmed"):why);}
    else if(*cmd=="status"){if(gui_output_)gui_output_->request_snapshot();}
    else if(*cmd=="quit"){cancel(true);ack(*cmd,true,"node stopping");rclcpp::shutdown();return;}
    else ack(*cmd,false,"supported: goal,cancel,stop,status,arm,disarm,quit");}
    catch(const std::exception& e){if(*cmd=="goal")cancel();ack(*cmd,false,e.what());}
   }
  }
 }
 void run_worker(){
  npc::Navigator navigator(config_);std::uint64_t seen_revision=~std::uint64_t(0),seen_epoch=~std::uint64_t(0);
  std::string last_plan_failure;std::uint64_t logged_reissue=0;
  auto next=Clock::now();
  while(!stopping_&&rclcpp::ok()){
   Inputs i;Result previous;{std::unique_lock<std::mutex> l(mutex_);cv_.wait_until(l,next,[this]{return stopping_.load();});if(stopping_)break;i=input_;previous=result_;}
   next=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/planning_hz_));
   Result r;r.revision=i.revision;r.epoch=i.epoch;r.map_generation=i.map_generation;const double now_ros=get_clock()->now().seconds();
   try{
    if(i.revision!=seen_revision||i.epoch!=seen_epoch){last_plan_failure.clear();navigator.cancel();if(i.goal)navigator.set_goal(*i.goal);seen_revision=i.revision;seen_epoch=i.epoch;}
    const double input_wall=wall_time();auto reason=input_reason(i,now_ros,input_wall);
    if(reason=="PERCEPTION_NOT_READY")r.perception_failure=status_diagnostic(i.per_status,now_ros,input_wall);
    if(!reason.empty())throw std::invalid_argument(reason);
    r.pose=planning_pose(i.odom.msg->pose.pose);r.stamp=ns(i.odom.msg->header.stamp);r.map_stamp=ns(i.layers.msg->grid.header.stamp);r.local_stamp=ns(i.local.msg->header.stamp);
    npc::Layers layers;layers.grid=grid(i.layers.msg->grid);layers.slope_cost.assign(i.layers.msg->slope_cost.begin(),i.layers.msg->slope_cost.end());layers.terrain_cost.assign(i.layers.msg->terrain_cost.begin(),i.layers.msg->terrain_cost.end());
    auto local=grid(*i.local.msg);std::optional<npc::Grid> support;if(need_camera_support()){support=grid(*i.support.msg);if(!match(layers.grid,*support))throw std::invalid_argument("CAMERA_SUPPORT_MISMATCH");}
    // Same latest-map validation as the command timer; feedback is consumed
    // only on the worker and only for this goal/session and a fresh result.
    if(config_.footprint_planning&&previous.nav.motion_valid&&
       result_context_reason(i,previous,now_ros,input_wall).empty()){
      const auto why=result_reason(i,previous,now_ros,input_wall);
      navigator.execution_feedback(why.find("FOOTPRINT_")==0||why=="PATH_KNOWN_OBSTACLE"||why=="PATH_UNOBSERVED"||why=="PATH_OUTSIDE_MAP",now_ros);
    }else navigator.execution_feedback(false,now_ros);
    r.nav=navigator.step(layers,local,r.pose,now_ros,support?&*support:nullptr);
    // 2026-11-25: one planner.log line per waypoint reissue attempt (at most once per new map).
    if(const auto& ev=navigator.reissue_event();ev.seq!=logged_reissue){logged_reissue=ev.seq;
      RCLCPP_INFO(get_logger(),"[GPP_REISSUE] outcome=%s from=(%.2f,%.2f) to=(%.2f,%.2f) progress=%.2f/%.2f m active_score=%.3f best_score=%.3f candidates=%d",
        ev.outcome.c_str(),ev.from.x,ev.from.y,ev.to.x,ev.to.y,ev.progress,ev.length,ev.active_score,ev.best_score,ev.candidates);}
    if(r.nav.arrived||r.nav.motion_valid)last_plan_failure.clear();
    else if(r.nav.reason=="waiting for global path retry interval"){if(!last_plan_failure.empty())r.nav.reason=last_plan_failure+" | retry pending";}
    else last_plan_failure=r.nav.reason;
    r.valid=true;r.error=r.nav.reason;
   }catch(const std::exception& e){last_plan_failure.clear();r.valid=false;r.error=e.what();r.nav.state="WAIT";r.nav.reason=r.error;}
   r.completed=wall_time();
   // A new map need not discard a useful completed search. Validation below and
   // on every command tick uses the latest immutable map snapshots instead.
   {std::lock_guard<std::mutex> l(mutex_);if(input_.revision==r.revision&&input_.epoch==r.epoch){
    if(r.nav.arrived)display_reference_.reset();
    else if(!r.nav.path.empty()){
     // GPP creation time is separate from this worker's fresh pose/result age.
     // A retained reference may be trimmed as the car moves, without a new plan.
     const bool retained=display_reference_&&display_reference_->revision==r.revision&&display_reference_->epoch==r.epoch&&!r.nav.path_changed&&retained_reference_tail(display_reference_->nav.path,r.nav.path);
     r.gpp_stamp=retained?display_reference_->gpp_stamp:r.stamp;
     display_reference_=r;
    }
    if(r.valid)last_valid_result_=r;
    // A worker can reject an old status snapshot just as a new READY arrives.
    // Reuse only the previous completed result, without renewing its age/provenance;
    // the command timer still validates the latest pose, maps, status and revision.
    if(!r.valid&&(r.error=="PERCEPTION_NOT_READY"||r.error=="LOCALIZATION_NOT_READY")&&
       last_valid_result_&&result_context_reason(input_,*last_valid_result_,get_clock()->now().seconds(),wall_time()).empty()){
     result_=*last_valid_result_;
     RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
        "[WORKER_STATUS_RECOVERED] previous result retained at original stamp; latest status/maps/pose still required");
    }else result_=std::move(r);
   }}
  }
 }
 nav_msgs::msg::Path path_msg(const npc::Path& path,std::int64_t stamp)const{nav_msgs::msg::Path m;m.header.frame_id=frame_;m.header.stamp=rclcpp::Time(stamp);m.poses.reserve(path.size());for(const auto& p:path){geometry_msgs::msg::PoseStamped v;v.header=m.header;v.pose=ros_pose(p);m.poses.push_back(v);}return m;}
 std::string result_context_reason(const Inputs& i,const Result& r,double now_ros,double now_wall)const{
  auto why=input_reason(i,now_ros,now_wall);if(!why.empty())return why;
  if(r.epoch!=i.epoch)return "SESSION_CHANGED";
  if(r.revision!=i.revision)return "GOAL_CHANGED_REPLAN";
  if(!r.valid)return r.error.empty()?"NO_PLAN":r.error;
  if(now_wall-r.completed>result_timeout_)return "PLANNER_RESULT_STALE";
  try{const auto p=planning_pose(i.odom.msg->pose.pose);if(npc::distance(p,r.pose)>max_displacement_||std::abs(npc::wrap(p.yaw-r.pose.yaw))>max_rotation_)return "PLANNER_RESULT_POSE_CHANGED";}catch(...){return "INVALID_ODOMETRY";}
  return {};
 }
 std::string global_path_reason(const Inputs& i,const Result& r,double now_ros,double now_wall)const{
  auto why=result_context_reason(i,r,now_ros,now_wall);if(!why.empty())return why;
  if(r.nav.path.empty())return r.nav.arrived?"ARRIVED":(r.nav.reason.empty()?"NO_GLOBAL_PATH":r.nav.reason);
  try{return npc::validate_remaining_result_path(grid_view(i.layers.msg->grid),r.nav.path,planning_pose(i.odom.msg->pose.pose),config_.corridor.allow_unknown_transit,true);}catch(...){return "PATH_MAP_CONTRACT";}
 }
 std::string result_reason(const Inputs& i,const Result& r,double now_ros,double now_wall)const{
  auto why=result_context_reason(i,r,now_ros,now_wall);if(!why.empty())return why;
  if(r.nav.arrived)return {};
  why=global_path_reason(i,r,now_ros,now_wall);if(!why.empty())return why;
  if(!r.nav.motion_valid)return r.nav.reason.empty()?"PLANNER_STOP":r.nav.reason;
  try{const double hl=config_.observed_guard?config_.guard_half_length:0,hw=config_.observed_guard?config_.guard_half_width:0;
   const npc::Pose* initial=config_.allow_initial_footprint_outside&&r.nav.has_start_footprint?&r.nav.start_footprint:nullptr;
   return npc::validate_remaining_result_path(grid_view(*i.local.msg),r.nav.trajectory,planning_pose(i.odom.msg->pose.pose),config_.corridor.allow_unknown_transit,false,hl,hw,initial,config_.guard_center_offset);
  }catch(...){return "TRAJECTORY_MAP_CONTRACT";}
 }
 static bool display_context(const Inputs& i,const Result& r){return i.goal&&r.revision==i.revision&&r.epoch==i.epoch&&!r.nav.path.empty();}
 static std::string stage_state(const std::string& reason,bool has_path){
  if(reason.empty())return "READY";
  if(reason=="ARRIVED")return "ARRIVED";
  if(reason.find("STALE")!=std::string::npos||reason.find("NOT_READY")!=std::string::npos)return "STALE";
  return has_path?"BLOCKED":"WAIT";
 }
 std::string serialize_trajectory_sample(){
  Odom::ConstSharedPtr message;
  {std::lock_guard<std::mutex> lock(mutex_);message=input_.odom.msg;}
  if(!message||message==last_recorded_odom_)return {};
  last_recorded_odom_=message;
  const auto stamp=ns(message->header.stamp);
  if(stamp<=0||message->header.frame_id.empty())return {};
  const auto& p=message->pose.pose.position;const auto& q=message->pose.pose.orientation;
  if(!std::isfinite(p.z))return {};
  try {
   const auto pose=planning_pose(message->pose.pose);
   std::ostringstream o;o.imbue(std::locale::classic());o<<std::setprecision(17);
   o<<"{\"kind\":\"pose\",\"t_ns\":"<<stamp<<",\"sample_ns\":"<<get_clock()->now().nanoseconds()
    <<",\"source\":\"pi4_localization\",\"frame_id\":"<<json_string(message->header.frame_id)
    <<",\"child_frame_id\":"<<json_string(message->child_frame_id)
    <<",\"x\":"<<p.x<<",\"y\":"<<p.y<<",\"z\":"<<p.z<<",\"yaw\":"<<pose.yaw
    <<",\"qx\":"<<q.x<<",\"qy\":"<<q.y<<",\"qz\":"<<q.z<<",\"qw\":"<<q.w<<'}';
   return o.str();
  } catch(const std::invalid_argument&) {return {};}
 }
 std::string serialize_gui_snapshot(){
  const double now_wall=wall_time();
  Inputs i;Result r;std::optional<Result> reference;OutputState output;{std::lock_guard<std::mutex> l(mutex_);i=input_;r=result_;reference=display_reference_;output=output_;}
  const double now_ros=get_clock()->now().seconds();const std::string error=result_reason(i,r,now_ros,now_wall);const bool valid=error.empty();
  const Result& shown=display_context(i,r)?r:(reference?*reference:r);const bool has_path=display_context(i,shown);
  const auto context_error=result_context_reason(i,r,now_ros,now_wall);
  const std::string gpp_error=has_path?global_path_reason(i,shown,now_ros,now_wall):((valid&&r.nav.arrived)?"ARRIVED":(context_error.empty()?(r.nav.reason.empty()?"NO_GLOBAL_PATH":r.nav.reason):context_error));
  const bool path_valid=has_path&&gpp_error.empty(),executable=valid&&r.nav.motion_valid;
  std::ostringstream o;o<<std::setprecision(10)<<"{\"type\":\"planning\",\"t\":"<<now_ros<<",\"frame\":"<<json_string(frame_)<<",\"stamp_ns\":"<<r.stamp<<",\"gpp_stamp_ns\":"<<(has_path?shown.gpp_stamp:0)<<",\"state\":"<<json_string(valid?r.nav.state:"WAIT")<<",\"detail\":"<<json_string(valid?r.nav.reason:error)<<",\"gpp_state\":"<<json_string(stage_state(gpp_error,has_path))<<",\"gpp_reason\":"<<json_string(gpp_error.empty()?"latest map route clear":gpp_error)<<",\"lpp_state\":"<<json_string((valid&&r.nav.arrived)?"ARRIVED":stage_state(error,has_path))<<",\"lpp_reason\":"<<json_string(valid?r.nav.reason:error)<<",\"path_valid\":"<<(path_valid?"true":"false")<<",\"executable\":"<<(executable?"true":"false")<<",\"plan_only\":"<<(plan_only_?"true":"false")<<",\"armed\":"<<(armed_?"true":"false")<<",\"output_mode\":"<<json_string(plan_only_?"plan_only":(prototype_output_?"prototype":"strict_measured"))<<",\"prototype_output\":"<<(prototype_output_?"true":"false")<<",\"feedback_source\":"<<json_string(output.feedback_source)<<",\"geometry_confirmed\":"<<(config_.vehicle.geometry_confirmed?"true":"false")<<",\"commissioning_confirmed\":"<<(commissioning_?"true":"false");
  o<<",\"goal\":";if(i.goal)o<<'['<<i.goal->x<<','<<i.goal->y<<','<<i.goal->yaw<<']';else o<<"null";
  o<<",\"path\":[";if(has_path){for(std::size_t j=0;j<shown.nav.path.size()&&j<4000;++j){if(j)o<<',';const auto&p=shown.nav.path[j];o<<'['<<p.x<<','<<p.y<<','<<p.yaw<<']';}}o<<"],\"local_path\":[";if(executable){for(std::size_t j=0;j<r.nav.trajectory.size()&&j<1000;++j){if(j)o<<',';const auto&p=r.nav.trajectory[j];o<<'['<<p.x<<','<<p.y<<','<<p.yaw<<']';}}o<<"],\"target\":";if(has_path&&shown.nav.has_target)o<<'['<<shown.nav.target.x<<','<<shown.nav.target.y<<','<<shown.nav.target.yaw<<']';else o<<"null";
  o<<",\"candidate_speed\":"<<(valid&&std::isfinite(r.nav.motion.speed)?r.nav.motion.speed:0)<<",\"candidate_steering\":"<<(valid&&std::isfinite(r.nav.motion.steering)?r.nav.motion.steering:0)<<",\"actuator_enabled\":"<<(output.enabled?"true":"false")<<",\"output_inhibit\":"<<json_string(output.inhibit)<<",\"output_stamp_ns\":"<<output.stamp<<'}';return o.str();
 }
 void tick(){
  const auto now=get_clock()->now();Inputs i;Result r;std::optional<Result> reference;bool reset=false;{std::lock_guard<std::mutex> l(mutex_);if(last_ros_ns_>0&&now.nanoseconds()+1000000<last_ros_ns_){++input_.epoch;input_.odom={};input_.layers={};input_.local={};input_.support={};input_.loc_status={};input_.per_status={};input_.vehicle={};result_.valid=false;display_reference_.reset();reset=true;}last_ros_ns_=now.nanoseconds();i=input_;r=result_;reference=display_reference_;}
  if(reset)cv_.notify_one();
  const double w=wall_time();std::string reason=result_reason(i,r,now.seconds(),w);const bool valid=reason.empty();Drive command;command.header.stamp=now;command.header.frame_id=vehicle_frame_;command.valid_for=lease(command_lease_);command.stop_requested=true;
  std::string inhibit;
  if(plan_only_)inhibit="PLAN_ONLY";else if(!prototype_output_&&!config_.vehicle.geometry_confirmed)inhibit="GEOMETRY_UNCONFIRMED";else if(!prototype_output_&&!commissioning_)inhibit="COMMISSIONING_UNCONFIRMED";else if(!armed_)inhibit="DISARMED";else inhibit=vehicle_reason(i,now.seconds(),w);
  if(inhibit.empty()&&!valid)inhibit=reason;
  // An unmeasured speed is never read as a physical braking measurement.
  if(inhibit.empty()&&i.vehicle.msg&&i.vehicle.msg->speed_valid&&std::abs(i.vehicle.msg->speed)>r.nav.observed_speed_cap+1e-3)inhibit="OBSERVED_BRAKING_CAP_EXCEEDED";
  if(valid&&inhibit.empty()&&r.nav.motion_valid&&std::isfinite(r.nav.motion.speed)&&std::isfinite(r.nav.motion.steering)&&r.nav.motion.speed>=0&&r.nav.motion.speed<=config_.rpp.speed&&std::abs(r.nav.motion.steering)<=config_.vehicle.max_steer+1e-8){command.stop_requested=false;command.target_speed=r.nav.motion.speed;command.target_steering_angle=r.nav.motion.steering;}
  if(inhibit.empty()&&command.stop_requested)inhibit=r.nav.arrived?"ARRIVED":"NO_VALID_MOTION";
  // Goal/cancel/disarm can arrive from stdio while this timer computes. Keep
  // permission latched, but never send motion for a replaced/cancelled goal.
  {std::lock_guard<std::mutex> l(mutex_);
   if(!command.stop_requested&&(!armed_||!input_.goal||input_.revision!=i.revision||input_.epoch!=i.epoch)){
    command.stop_requested=true;command.target_speed=0;command.target_steering_angle=0;
    inhibit=!armed_?"DISARMED":"GOAL_CHANGED_REPLAN";
   }
   drive_pub_->publish(command);
   output_={!command.stop_requested,inhibit,feedback_source(i),now.nanoseconds()};
  }
  Status s;s.header.stamp=now;s.header.frame_id=frame_;s.valid_for=lease(.3);s.state=valid?Status::READY:Status::WAITING;s.detail=valid?r.nav.state+": "+r.nav.reason:reason;if(!inhibit.empty())s.detail+=" | output inhibited: "+inhibit;status_pub_->publish(s);
  // These are diagnostic paths, not permission to drive. Retained GPP keeps its
  // original source stamp, even when stale or when the LPP cannot execute it.
  const Result& shown=display_context(i,r)?r:(reference?*reference:r);const bool has_path=display_context(i,shown);
  if(w-last_paths_>=.2){
   global_pub_->publish(path_msg(has_path?shown.nav.path:npc::Path{},has_path?shown.gpp_stamp:now.nanoseconds()));last_path_empty_=!has_path;
   const bool has_local=valid&&r.nav.motion_valid;local_pub_->publish(path_msg(has_local?r.nav.trajectory:npc::Path{},has_local?r.stamp:now.nanoseconds()));last_local_empty_=!has_local;last_paths_=w;
  }else{
   if(!has_path&&!last_path_empty_){global_pub_->publish(path_msg({},now.nanoseconds()));last_path_empty_=true;}
   if((!valid||!r.nav.motion_valid)&&!last_local_empty_){local_pub_->publish(path_msg({},now.nanoseconds()));last_local_empty_=true;}
  }
  // Log candidate failures even without a GUI attached. Use the same validity
  // and reason as GUI telemetry, independently of output inhibit/arming.
  const bool executable=valid&&r.nav.motion_valid;
  const std::string lpp_state=(valid&&r.nav.arrived)?"ARRIVED":stage_state(reason,has_path);
  const std::string lpp_reason=valid?r.nav.reason:reason;
  std::optional<npc::StatusDiagnostic> current_perception;
  std::optional<npc::StatusFailureEvidence> perception_evidence;
  if(lpp_reason=="PERCEPTION_NOT_READY"){
   current_perception=status_diagnostic(i.per_status,now.seconds(),w);
   perception_evidence=npc::status_failure_evidence(*current_perception,r.perception_failure);
  }
  // Extend only the log deduplication key. Published reasons and STOP gates
  // stay unchanged; a new subcause must not hide behind the same generic text.
  const bool report_rpp=!r.nav.rpp_rejection_key.empty()&&lpp_reason==r.nav.reason;
  const auto diagnostic_key=lpp_reason+(report_rpp?" | rpp="+r.nav.rpp_rejection_key:"")+(perception_evidence?" | perception="+perception_evidence->key():"");
  const auto diagnostic=lpp_diagnostics_.update(bool(i.goal),valid&&r.nav.arrived,
      executable,i.revision,i.epoch,lpp_state,diagnostic_key);
  if(diagnostic==npc::LppDiagnostics::Event::pause){
   const auto context_error=result_context_reason(i,r,now.seconds(),w);
   const std::string gpp_error=has_path?global_path_reason(i,shown,now.seconds(),w):
       (context_error.empty()?(r.nav.reason.empty()?"NO_GLOBAL_PATH":r.nav.reason):context_error);
   std::string perception_log;
   if(perception_evidence){
    perception_log=" | perception_status={\"origin\":"+json_string(perception_evidence->from_worker?"worker_result":"current_input")
     +",\"failure\":"+status_diagnostic_json(perception_evidence->failure)
     +",\"current\":"+status_diagnostic_json(*current_perception)+"}";
   }
   RCLCPP_WARN(get_logger(),
       "[LPP_PAUSE] reason=%s | LPP=%s executable=false GPP=%s path_valid=%s | goal_revision=%llu result_stamp_ns=%lld odom_age=%.3f local_map_age=%.3f path_points=%zu trajectory_points=%zu%s",
       json_string(lpp_reason).c_str(),lpp_state.c_str(),stage_state(gpp_error,has_path).c_str(),
       has_path&&gpp_error.empty()?"true":"false",static_cast<unsigned long long>(i.revision),
       static_cast<long long>(r.stamp),i.odom.msg?now.seconds()-ns(i.odom.msg->header.stamp)*1e-9:-1.,
       i.local.msg?now.seconds()-ns(i.local.msg->header.stamp)*1e-9:-1.,shown.nav.path.size(),r.nav.trajectory.size(),perception_log.c_str());
   if(report_rpp){
    RCLCPP_WARN(get_logger(),"[LPP_REJECT] result_stamp_ns=%lld pose=(%.6f,%.6f,%.6f) current_local_map_age=%.3f candidates=%s",
        static_cast<long long>(r.stamp),r.pose.x,r.pose.y,r.pose.yaw,i.local.msg?now.seconds()-ns(i.local.msg->header.stamp)*1e-9:-1.,r.nav.rpp_rejection_detail.c_str());
   }
  }else if(diagnostic==npc::LppDiagnostics::Event::recovered){
   RCLCPP_INFO(get_logger(),"[LPP_CANDIDATE_RECOVERED] executable=true; candidate recovery does not imply actuator output");
  }
 }
 public:
 explicit PlannerNode(bool stdio_enabled):Node("nomad_planning_cpp"),stdio_enabled_(stdio_enabled){
  frame_=param<std::string>("planning_frame","odom");vehicle_frame_=param<std::string>("vehicle_frame","base_link");config_.frame=frame_;
  prototype_output_=param<bool>("prototype_output",false);allow_unmeasured_feedback_=param<bool>("allow_unmeasured_vehicle_feedback",false);
  if(allow_unmeasured_feedback_&&!prototype_output_)throw std::invalid_argument("unmeasured vehicle feedback requires explicit prototype_output=true");
  plan_only_=param<bool>("plan_only",true);commissioning_=param<bool>("commissioning_confirmed",false);config_.vehicle.geometry_confirmed=param<bool>("geometry_confirmed",false);
  preview_camera_heading_=param<bool>("preview_camera_heading",false);
  preview_forward_axis_={{param<double>("preview_forward_axis_x",1.),param<double>("preview_forward_axis_y",0.),param<double>("preview_forward_axis_z",0.)}};
  if(preview_camera_heading_){
   if(!plan_only_&&!prototype_output_)throw std::invalid_argument("preview_camera_heading requires plan_only or explicit prototype_output; it is not calibrated body TF");
   double norm=0;for(double v:preview_forward_axis_){if(!std::isfinite(v))throw std::invalid_argument("nonfinite preview forward axis");norm+=v*v;}
   if(norm<.25||norm>2.25)throw std::invalid_argument("invalid preview forward axis magnitude");
  }
  config_.vehicle.wheelbase=param<double>("wheelbase",.72);config_.vehicle.track=param<double>("track",.60);config_.vehicle.wheel_radius=param<double>("wheel_radius",.15);config_.vehicle.joint_limit=param<double>("steering_joint_limit",.55);config_.vehicle.max_steer=param<double>("max_steer",.35);config_.vehicle.reference_offset=param<double>("reference_offset",-.36);config_.vehicle.body_length=param<double>("body_length",1.0);config_.vehicle.body_width=param<double>("body_width",.65);config_.vehicle.rear_steer=param<bool>("rear_steer",true);
  config_.rpp.speed=param<double>("forward_speed",.5);config_.arrival=param<double>("arrival_tolerance",.35);config_.replan_period=param<double>("replan_period",1.0);config_.search.goal_tolerance=param<double>("goal_tolerance",.29);config_.search.seconds=param<double>("search_seconds",.5);config_.search.max_expansions=param<int>("search_expansions",50000);config_.corridor.seconds=param<double>("corridor_seconds",.25);config_.corridor.max_expansions=param<int>("corridor_expansions",20000);config_.corridor.arrival=param<double>("corridor.arrival_tolerance",1.0);
  config_.corridor.allow_unknown_transit=param<bool>("corridor.allow_unknown_transit",false);config_.corridor.waypoint_radius=param<double>("corridor.waypoint_radius",1.5);
  config_.allow_initial_footprint_outside=param<bool>("observed_guard.allow_initial_footprint_outside",false);
  planning_hz_=param<double>("planning_hz",5);const double command_hz=param<double>("command_hz",20);odom_timeout_=param<double>("odometry_timeout",.5);map_timeout_=param<double>("map_timeout",2.5);status_timeout_=param<double>("status_timeout",1.5);vehicle_timeout_=param<double>("vehicle_timeout",.3);result_timeout_=param<double>("result_timeout",1);future_tolerance_=param<double>("clock_future_tolerance",.2);command_lease_=param<double>("command_lease",.15);max_displacement_=param<double>("max_result_displacement",.4);max_rotation_=param<double>("max_result_rotation",.4);const int max_cells=param<int>("max_map_cells",1000000);max_cells_=static_cast<std::size_t>(std::max(0,max_cells));
  config_.costs.multi=param<bool>("cost.multi",true);config_.costs.obstacle_weight=param<double>("cost.obstacle_weight",16);config_.costs.radius=param<double>("cost.proximity_radius",4);config_.costs.slope_weight=param<double>("cost.slope_weight",5);config_.costs.terrain_weight=param<double>("cost.terrain_weight",.5);config_.costs.missing_cost=param<double>("cost.missing_cost",1);config_.costs.unknown_surcharge=param<double>("cost.unknown_surcharge",9);config_.costs.unknown_penalty=param<double>("cost.unknown_penalty",9);config_.costs.cluster_weight=param<double>("cost.cluster_weight",32);config_.costs.cluster_radius=param<double>("cost.cluster_radius",4);config_.costs.cluster_influence=param<double>("cost.cluster_influence",.75);config_.costs.cluster_saturation=param<double>("cost.cluster_saturation",.4);
  config_.rpp.lookahead=param<double>("rpp.lookahead",.8);config_.rpp.adaptive_lookahead=param<bool>("rpp.adaptive_lookahead",true);config_.rpp.lateral_accel=param<double>("rpp.lateral_accel",.35);config_.observed_guard=param<bool>("observed_guard.enabled",true);config_.guard_half_length=param<double>("observed_guard.half_length",.20);config_.guard_half_width=param<double>("observed_guard.half_width",.10);config_.guard_center_offset=param<double>("observed_guard.center_offset",-.15);config_.guard_deceleration=param<double>("observed_guard.deceleration",.4);config_.guard_latency=param<double>("observed_guard.latency",.3);config_.guard_margin=param<double>("observed_guard.margin",.1);config_.corridor.observation_guard=param<bool>("corridor.observation_guard",true);config_.corridor.observation_turn_weight=param<double>("corridor.observation_turn_weight",80);
  config_.costs.clearance_weight=param<double>("cost.clearance_weight",0.0);
  config_.costs.clearance_radius=param<double>("cost.clearance_radius",.45);
  // 2026-11-25: relative clearance cost (corner hugging fix); 0 = previous additive-only behaviour.
  config_.costs.clearance_factor=param<double>("cost.clearance_factor",2.0);
  config_.corridor.corner_aware=param<bool>("corridor.corner_aware",false);
  config_.corridor.preferred_clearance=param<double>("corridor.preferred_clearance",.45);
  // 2026-11-11: hard GPP clearance (cells within this distance of a blocked cell become impassable). Default 0 = previous behaviour.
  config_.corridor.min_clearance=param<double>("corridor.min_clearance",0.0);
  config_.corridor.transit_clearance_weight=param<double>("corridor.transit_clearance_weight",4.0);
  // 2026-11-25: obstacle-aware waypoint direction score (corner_aware only); false = previous bearing score.
  config_.corridor.goal_cost_scoring=param<bool>("corridor.goal_cost_scoring",true);
  config_.corridor.goal_cost_clearance_weight=param<double>("corridor.goal_cost_clearance_weight",1.0);
  // 2026-11-25 goal approach / feasibility (field review 19:03-19:13 records).
  config_.corridor.final_goal_tolerance=param<double>("corridor.final_goal_tolerance",0.30);
  config_.corridor.goal_approach_radius=param<double>("corridor.goal_approach_radius",2.0);
  config_.corridor.gpp_steer_ratio=param<double>("corridor.gpp_steer_ratio",0.85);
  config_.corridor.exit_check_length=param<double>("corridor.exit_check_length",0.6);
  config_.corridor.start_clearance_exempt=param<double>("corridor.start_clearance_exempt",0.2);
  // 2026-11-25 waypoint reissue; parameters are optional and default to the previous behaviour.
  config_.corridor.reissue_enabled=param<bool>("corridor.reissue_enabled",false);
  config_.corridor.reissue_travel_ratio=param<double>("corridor.reissue_travel_ratio",.4);
  config_.corridor.reissue_min_travel=param<double>("corridor.reissue_min_travel",.2);
  config_.corridor.reissue_end_margin=param<double>("corridor.reissue_end_margin",.6);
  config_.corridor.reissue_margin=param<double>("corridor.reissue_margin",.15);
  config_.corridor.reissue_min_interval=param<double>("corridor.reissue_min_interval",1.0);
  config_.corridor.reissue_search_seconds=param<double>("corridor.reissue_search_seconds",.5);
  config_.footprint_planning=param<bool>("footprint_planning",false);
  config_.rpp.unknown_speed=param<double>("rpp.unknown_speed",0.0);
  if(!std::isfinite(config_.rpp.unknown_speed)||config_.rpp.unknown_speed<0||config_.rpp.unknown_speed>config_.rpp.speed)throw std::invalid_argument("rpp.unknown_speed must be between zero and forward_speed");
  if(prototype_output_&&(config_.rpp.speed>.25+1e-9||config_.vehicle.max_steer>kPrototypeSteeringLimit+1e-9))throw std::invalid_argument("prototype output is limited to nominal 0.25 m/s and 35 degree steering commands; physical calibration remains unverified");
  config_.rpp.lookahead_candidates=param<std::vector<double>>("rpp.lookahead_candidates",std::vector<double>{});
  (void)npc::rpp_lookaheads(config_.rpp);
  npc::validate_vehicle(config_.vehicle);
  if(!(planning_hz_>0&&planning_hz_<=20&&command_hz>=5&&command_hz<=50&&config_.search.seconds>0&&config_.search.seconds<=2&&config_.corridor.seconds>0&&config_.corridor.seconds<=1&&max_cells_>0&&max_cells_<=4000000&&command_lease_>0&&command_lease_<=.3&&odom_timeout_>0&&map_timeout_>0&&status_timeout_>0&&vehicle_timeout_>0&&result_timeout_>0))throw std::invalid_argument("unsafe planning bounds");
  const auto map_qos=rclcpp::QoS(1).reliable().transient_local();const auto status_qos=rclcpp::QoS(1).reliable();const auto sensor_qos=rclcpp::SensorDataQoS().keep_last(1);
  drive_pub_=create_publisher<Drive>(topic("drive_command","/nomad/planning/drive_command"),rclcpp::QoS(1).reliable());status_pub_=create_publisher<Status>("/nomad/planning/status",status_qos);global_pub_=create_publisher<nav_msgs::msg::Path>("/nomad/global_path",map_qos);local_pub_=create_publisher<nav_msgs::msg::Path>("/nomad/local_path",rclcpp::QoS(1).reliable());goal_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/nomad/goal",map_qos);
  goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(topic("goal","/goal_pose"),rclcpp::QoS(1).reliable(),[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){try{goal(pose_of(m->pose),m->header.frame_id,ns(m->header.stamp));}catch(const std::exception&e){cancel();RCLCPP_WARN(get_logger(),"Rejected goal; current motion cancelled: %s",e.what());}});
  odom_sub_=create_subscription<Odom>(topic("odometry","/nomad/localization/odometry"),sensor_qos,[this](Odom::ConstSharedPtr m){update(&Inputs::odom,m);});
  layers_sub_=create_subscription<LayerMsg>(topic("layers","/nomad/planning_cost_layers"),map_qos,[this](LayerMsg::ConstSharedPtr m){update(&Inputs::layers,m);});
  local_sub_=create_subscription<GridMsg>(topic("local_costmap","/nomad/navigation_local_costmap"),map_qos,[this](GridMsg::ConstSharedPtr m){update(&Inputs::local,m);});
  const auto camera_support_topic=topic("camera_support","/nomad/camera_observation_support");
  if(need_camera_support())support_sub_=create_subscription<GridMsg>(camera_support_topic,map_qos,[this](GridMsg::ConstSharedPtr m){update(&Inputs::support,m);});
  loc_sub_=create_subscription<Status>(topic("localization_status","/nomad/localization/status"),status_qos,[this](Status::ConstSharedPtr m){update(&Inputs::loc_status,m);});
  per_sub_=create_subscription<Status>(topic("perception_status","/nomad/perception/status"),status_qos,[this](Status::ConstSharedPtr m){update(&Inputs::per_status,m);});
  vehicle_sub_=create_subscription<Vehicle>(topic("vehicle_state","/nomad/control/vehicle_state"),status_qos,[this](Vehicle::ConstSharedPtr m){update(&Inputs::vehicle,m);});
  arm_srv_=create_service<std_srvs::srv::SetBool>("/nomad/planning/arm",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){const auto why=set_armed(req->data);res->success=why.empty();res->message=res->success?(req->data?"explicitly armed":"disarmed"):why;});
  cancel_srv_=create_service<std_srvs::srv::Trigger>("/nomad/planning/cancel",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){cancel();res->success=true;res->message="goal cancelled; output permission unchanged";});
  global_pub_->publish(path_msg({},get_clock()->now().nanoseconds()));
  timer_=create_wall_timer(std::chrono::duration<double>(1/command_hz),std::bind(&PlannerNode::tick,this));
  if(stdio_enabled_)gui_output_=std::make_unique<npc::LatestJsonWriter>(STDOUT_FILENO,[this]{return serialize_gui_snapshot();},[this]{cancel(true);rclcpp::shutdown();});
  const char* log_dir=std::getenv("NOMAD_PLANNING_LOG_DIR");
  const auto trajectory_file=param<std::string>("trajectory_log_file",log_dir?std::string(log_dir)+"/telemetry.jsonl":"");
  worker_=std::thread(&PlannerNode::run_worker,this);if(stdio_enabled_)stdio_=std::thread(&PlannerNode::stdio_loop,this);
  RCLCPP_INFO(get_logger(),"Hybrid A* + RPP on %s; plan_only=%s; geometry=%s; commissioning=%s; no automatic arming",frame_.c_str(),plan_only_?"true":"false",config_.vehicle.geometry_confirmed?"confirmed":"UNCONFIRMED",commissioning_?"confirmed":"UNCONFIRMED");
  RCLCPP_INFO(get_logger(),"LPP footprint: front=%.2f rear=%.2f left=%.2f right=%.2f m from planning reference; center_offset=%.2f; margin=%.2f",config_.guard_half_length+config_.guard_center_offset,config_.guard_half_length-config_.guard_center_offset,config_.guard_half_width,config_.guard_half_width,config_.guard_center_offset,config_.guard_margin);
  RCLCPP_INFO(get_logger(),"LPP unknown-cell speed cap: %.3f m/s (0=legacy cost regulation); other motion and braking limits still apply",config_.rpp.unknown_speed);
  if(prototype_output_)RCLCPP_WARN(get_logger(),"PROTOTYPE OUTPUT: unconfirmed geometry/calibration; feedback policy=%s; nominal max %.2f m/s / %.9f rad (%.1f degree steering command); physical calibration unverified; explicit arming required",allow_unmeasured_feedback_?"unmeasured allowed":"measured required",config_.rpp.speed,config_.vehicle.max_steer,config_.vehicle.max_steer*180.0/npc::kPi);
  if(preview_camera_heading_)RCLCPP_INFO(get_logger(),"Preview heading uses rotated camera forward axis only inside planning; source Odometry/TF are unchanged");
  if(!trajectory_file.empty()){
   try {
    trajectory_output_=std::make_unique<npc::TrajectoryWriter>(trajectory_file,[this]{return serialize_trajectory_sample();},
     [this](const std::string& error){RCLCPP_ERROR(get_logger(),"[TRAJECTORY_LOG_ERROR] %s; coordinate recording stopped",error.c_str());});
    RCLCPP_INFO(get_logger(),"[TRAJECTORY_LOG] automatic coordinates at up to 10Hz: %s",trajectory_file.c_str());
   }catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"[TRAJECTORY_LOG_ERROR] %s; coordinates unavailable",e.what());}
  }
 }
 ~PlannerNode()override{stopping_=true;armed_=false;cv_.notify_all();if(gui_output_)gui_output_->stop(std::chrono::milliseconds(100));if(worker_.joinable())worker_.join();if(stdio_.joinable())stdio_.join();if(trajectory_output_)trajectory_output_->stop();}
};
int main(int argc,char**argv){bool stdio=false;std::vector<char*> args;for(int i=0;i<argc;++i){if(std::string(argv[i])=="--stdio")stdio=true;else args.push_back(argv[i]);}int n=static_cast<int>(args.size());rclcpp::init(n,args.data());try{auto node=std::make_shared<PlannerNode>(stdio);rclcpp::spin(node);rclcpp::shutdown();return 0;}catch(const std::exception&e){std::cerr<<"planning startup failed: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}}

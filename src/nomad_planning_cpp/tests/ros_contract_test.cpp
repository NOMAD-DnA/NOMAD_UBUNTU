// Synthetic ROS contract checks only. Must use an isolated test domain.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nomad_interfaces/msg/drive_command.hpp>
#include <nomad_interfaces/msg/planning_cost_layers.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <nomad_interfaces/msg/vehicle_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
using namespace std::chrono_literals;
class Contract final:public rclcpp::Node{
 using Odom=nav_msgs::msg::Odometry;using Grid=nav_msgs::msg::OccupancyGrid;using Layers=nomad_interfaces::msg::PlanningCostLayers;using Status=nomad_interfaces::msg::ModuleStatus;using Drive=nomad_interfaces::msg::DriveCommand;using Vehicle=nomad_interfaces::msg::VehicleState;
 rclcpp::Publisher<Odom>::SharedPtr odom_;rclcpp::Publisher<Layers>::SharedPtr layers_;rclcpp::Publisher<Grid>::SharedPtr local_,support_;rclcpp::Publisher<Status>::SharedPtr loc_,per_;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_;
 rclcpp::Subscription<Drive>::SharedPtr drive_;rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_,local_path_;rclcpp::Subscription<Status>::SharedPtr state_;
 rclcpp::Publisher<Vehicle>::SharedPtr vehicle_;
 public:bool camera_guard_enabled=true,map_content_case=false,arm_intent_case=false;int drives=0,moving=0,paths=0,empty=0,local_paths=0,local_empty=0,statuses=0,map_change_waits=0;std::string last_status;
 Contract():Node("nomad_planning_contract_test"){
  camera_guard_enabled=declare_parameter<bool>("camera_guard_enabled",true);
  map_content_case=declare_parameter<bool>("map_content_case",false);
  arm_intent_case=declare_parameter<bool>("arm_intent_case",false);
  // This opt-in contract uses synthetic VehicleState only, on an isolated domain.
  // Run the planner in prototype output mode with preview_camera_heading=false.
  if(arm_intent_case)vehicle_=create_publisher<Vehicle>("/nomad/control/vehicle_state",rclcpp::QoS(1).reliable());
  auto q=rclcpp::QoS(1).reliable().transient_local();odom_=create_publisher<Odom>("/nomad/localization/odometry",rclcpp::QoS(2).best_effort());layers_=create_publisher<Layers>("/nomad/planning_cost_layers",q);local_=create_publisher<Grid>("/nomad/navigation_local_costmap",q);support_=create_publisher<Grid>("/nomad/camera_observation_support",q);loc_=create_publisher<Status>("/nomad/localization/status",5);per_=create_publisher<Status>("/nomad/perception/status",5);goal_=create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",1);
  drive_=create_subscription<Drive>("/nomad/planning/drive_command",1,[this](Drive::ConstSharedPtr m){++drives;if(!m->stop_requested||m->target_speed!=0||m->target_steering_angle!=0)++moving;});
  path_=create_subscription<nav_msgs::msg::Path>("/nomad/global_path",q,[this](nav_msgs::msg::Path::ConstSharedPtr m){if(m->poses.empty())++empty;else ++paths;});
  local_path_=create_subscription<nav_msgs::msg::Path>("/nomad/local_path",rclcpp::QoS(1).reliable(),[this](nav_msgs::msg::Path::ConstSharedPtr m){if(m->poses.empty())++local_empty;else ++local_paths;});
  state_=create_subscription<Status>("/nomad/planning/status",5,[this](Status::ConstSharedPtr m){last_status=m->detail;++statuses;if(last_status.find("MAP_CONTENT_CHANGED_REPLAN")!=std::string::npos||last_status.find("MAP_UPDATED_REPLAN")!=std::string::npos)++map_change_waits;});
 }
 void send_goal(double x=2,double y=0){geometry_msgs::msg::PoseStamped g;g.header.stamp=now();g.header.frame_id="odom";g.pose.position.x=x;g.pose.position.y=y;g.pose.orientation.w=1;goal_->publish(g);}
 bool camera_subscription_contract()const{return camera_guard_enabled?support_->get_subscription_count()>0:support_->get_subscription_count()==0;}
 void tick(const std::string& mode){auto t=now();Odom o;o.header.stamp=t;o.header.frame_id=mode=="wrong_frame"?"map":"odom";o.child_frame_id="base_link";o.pose.pose.orientation.w=1;if(mode=="future_odom")o.header.stamp=t+rclcpp::Duration::from_seconds(10);if(mode=="stale_odom")o.header.stamp=t-rclcpp::Duration::from_seconds(10);odom_->publish(o);
  if(vehicle_){Vehicle v;v.header.stamp=t;v.header.frame_id="base_link";v.speed_valid=true;v.steering_valid=true;v.ready=mode!="vehicle_wait";v.speed=mode=="fast_vehicle"?2.0:0.0;vehicle_->publish(v);}
  Status s;s.header.stamp=t;s.header.frame_id="odom";s.valid_for.nanosec=600000000;s.state=mode=="bad_status"?Status::WAITING:Status::READY;loc_->publish(s);per_->publish(s);
  Grid g;g.header.stamp=t;g.header.frame_id="odom";g.info.resolution=.2;g.info.width=40;g.info.height=40;g.info.origin.position.x=-4;g.info.origin.position.y=-4;g.info.origin.orientation.w=1;g.data.assign(1600,0);
  if(mode=="stale_map")g.header.stamp=t-rclcpp::Duration::from_seconds(10);
  // The focused content case deliberately sends new capture stamps at 20 Hz.
  // Every synthetic capture has a fresh stamp, including fault transitions.
  // Whole-second quantization delayed some faults past the settle period,
  // because the node correctly discards duplicate/nonmonotonic timestamps.
  if(mode=="obstacle_update")g.data.assign(1600,100);
  if(mode=="unrelated_update")g.data[0]=100;
  Layers l;l.grid=g;l.slope_cost.assign(1600,0);l.terrain_cost.assign(1600,0);if(mode=="identical_captures"||mode=="obstacle_update"||mode=="unrelated_update"){l.slope_cost[0]=std::numeric_limits<float>::quiet_NaN();l.terrain_cost[0]=std::numeric_limits<float>::quiet_NaN();}if(mode=="bad_layers")l.slope_cost.resize(1);layers_->publish(l);if(mode=="local_blocked")g.data.assign(1600,100);local_->publish(g);g.data.assign(1600,100);if(mode=="bad_support")g.header.stamp.nanosec=1;support_->publish(g);
 }
 void pump(const std::string& mode,double seconds){
  const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
  while(std::chrono::steady_clock::now()<end){tick(mode);rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(50ms);}
 }
 bool arm_request(bool enabled){
  auto client=create_client<std_srvs::srv::SetBool>("/nomad/planning/arm");
  if(!client->wait_for_service(2s))return false;
  auto request=std::make_shared<std_srvs::srv::SetBool::Request>();request->data=enabled;
  auto response=client->async_send_request(request);
  for(int j=0;j<40&&response.wait_for(0s)!=std::future_status::ready;++j)pump("healthy",.05);
  if(response.wait_for(0s)!=std::future_status::ready)return false;
  auto result=response.get();
  std::cout<<"arm_request "<<enabled<<" success="<<result->success<<" detail="<<result->message<<std::endl;
  return result->success;
 }
 bool cancel_request(){
  auto client=create_client<std_srvs::srv::Trigger>("/nomad/planning/cancel");
  if(!client->wait_for_service(2s))return false;
  auto response=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  for(int j=0;j<40&&response.wait_for(0s)!=std::future_status::ready;++j)pump("healthy",.05);
  return response.wait_for(0s)==std::future_status::ready&&response.get()->success;
 }
 bool intent_phase(const std::string& label,const std::string& mode,bool expect_moving,
                   const std::string& expected_reason="",bool expect_disarmed=false,double settle=.5){
  pump(mode,settle);
  const int old_moving=moving,old_drives=drives;
  last_status.clear();pump(mode,.6);
  bool ok=drives>old_drives&&!last_status.empty()&&(expect_moving?moving>old_moving:moving==old_moving);
  if(!expected_reason.empty())ok=ok&&last_status.find(expected_reason)!=std::string::npos;
  ok=ok&&((last_status.find("DISARMED")!=std::string::npos)==expect_disarmed);
  std::cout<<"arm_intent "<<label<<" "<<(ok?"PASS":"FAIL")
           <<" commands="<<drives-old_drives<<" moving="<<moving-old_moving
           <<" status="<<last_status<<std::endl;
  return ok;
 }
 bool arm_intent_contract(){
  // No VESC driver/controller is launched by this program. Movement commands
  // are observed as messages only; the main() isolation check is mandatory.
  bool ok=intent_phase("default disarmed","healthy",false,"NO_GOAL",true);
  // Arm is operator intent: it can be latched before a goal/path is available.
  ok=arm_request(true)&&ok;
  ok=intent_phase("arm before goal","healthy",false,"NO_GOAL")&&ok;
  send_goal();
  ok=intent_phase("first goal starts without second arm","healthy",true,"",false,1.8)&&ok;
  for(const auto& mode:{"stale_odom","bad_status","vehicle_wait","local_blocked","obstacle_update","fast_vehicle"}){
   const std::string reason=std::string(mode)=="stale_odom"?"ODOMETRY_STALE":
     std::string(mode)=="bad_status"?"LOCALIZATION_NOT_READY":
     std::string(mode)=="vehicle_wait"?"VEHICLE_NOT_READY_OR_FAULT":
     std::string(mode)=="fast_vehicle"?"OBSERVED_BRAKING_CAP_EXCEEDED":"";
   ok=intent_phase(std::string("temporary STOP: ")+mode,mode,false,reason)&&ok;
   ok=intent_phase(std::string("automatic recovery: ")+mode,"healthy",true,"",false,1.8)&&ok;
  }
  // A new goal and a goal cancellation stop the old motion but preserve arm.
  send_goal(2,.2);
  ok=intent_phase("replacement goal keeps intent","healthy",true,"",false,1.8)&&ok;
  ok=cancel_request()&&ok;
  ok=intent_phase("cancel stops but keeps intent","healthy",false,"NO_GOAL")&&ok;
  send_goal();
  ok=intent_phase("goal after cancel resumes","healthy",true,"",false,1.8)&&ok;
  // Explicit output-stop is the action that clears the session intent.
  ok=arm_request(false)&&ok;
  ok=intent_phase("explicit disarm stops","healthy",false,"DISARMED",true)&&ok;
  send_goal(2,.1);
  ok=intent_phase("new goal cannot undo explicit disarm","healthy",false,"DISARMED",true,1.8)&&ok;
  ok=arm_request(true)&&ok;
  ok=intent_phase("explicit rearm resumes","healthy",true)&&ok;
  // Leave even this isolated synthetic session with no permitted movement.
  ok=arm_request(false)&&ok;
  ok=intent_phase("final disarm","healthy",false,"DISARMED",true)&&ok;
  std::cout<<"arm_intent_contract "<<(ok?"PASS":"FAIL")<<std::endl;
  return ok;
 }
 bool content_case(){
  bool ok=phase("identical_captures",1.2,true,false);
  const int before_empty=empty,before_wait=map_change_waits;
  ok=phase("identical_captures",1.2,true,false)&&ok;
  ok=ok&&empty==before_empty&&map_change_waits==before_wait;
  const int before_unrelated_empty=empty;
  ok=phase("unrelated_update",1.2,true,false)&&ok;
  ok=ok&&empty==before_unrelated_empty&&map_change_waits==before_wait;
  // Global reference remains useful diagnostics while the separate local map
  // rejects the motion. No local executable trajectory may survive settling.
  for(int j=0;j<6;++j){tick("local_blocked");rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(50ms);}
  const int local_before=local_paths;
  ok=phase("local_blocked",.8,true,false)&&ok;
  ok=ok&&local_paths==local_before;
  const int blocked_before=paths;
  for(int j=0;j<6;++j){tick("obstacle_update");rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(50ms);}
  phase("obstacle_update",.8,false,false);
  ok=ok&&paths>blocked_before&&moving==0;
  const int before_cancel_empty=empty;
  ok=cancel_test()&&ok;
  ok=ok&&empty>before_cancel_empty;
  std::cout<<"map_content_case "<<(ok?"PASS":"FAIL")<<" unrelated_empty="<<before_unrelated_empty<<"->"<<before_cancel_empty<<" change_waits="<<map_change_waits<<" retained_blocked_paths="<<paths-blocked_before<<" local_paths_while_blocked="<<local_paths-local_before<<" moving="<<moving<<std::endl;
  return ok;
 }
 bool phase(const std::string& mode,double seconds,bool expect_path,bool expect_reject){if(expect_reject){for(int settle=0;settle<6;++settle){tick(mode);rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(50ms);}}const int oldp=paths,olde=empty,oldm=moving,oldlocal=local_paths;last_status.clear();const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);while(std::chrono::steady_clock::now()<end){tick(mode);rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(50ms);}std::string expected;if(mode=="bad_support")expected="CAMERA_SUPPORT_MISMATCH";else if(mode=="bad_status")expected="LOCALIZATION_NOT_READY";else if(mode=="wrong_frame")expected="ODOMETRY_FRAME";else if(mode=="future_odom")expected="ODOMETRY_STALE";else if(mode=="bad_layers")expected="Cost layer size mismatch";else if(mode=="stale_map")expected="LAYERS_STALE";bool ok=moving==oldm&&(!expect_path||paths>oldp)&&(!expect_reject||(local_paths==oldlocal&&last_status.find(expected)!=std::string::npos&&last_status.find("PLAN_ONLY")!=std::string::npos));if(mode=="healthy"&&expect_path)ok=ok&&local_paths>oldlocal&&last_status.find("DRIVING:")==0;std::cout<<mode<<" "<<(ok?"PASS":"FAIL")<<" paths="<<paths-oldp<<" cleared="<<empty-olde<<" moving="<<moving-oldm<<" status="<<last_status<<std::endl;return ok;}
 bool cancel_test(){auto c=create_client<std_srvs::srv::Trigger>("/nomad/planning/cancel");if(!c->wait_for_service(2s))return false;auto f=c->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());for(int j=0;j<50&&f.wait_for(0s)!=std::future_status::ready;++j){rclcpp::spin_some(shared_from_this());std::this_thread::sleep_for(20ms);}if(f.wait_for(0s)!=std::future_status::ready)return false;const bool ack=f.get()->success;phase("healthy",.5,false,false);return ack&&last_status.find("NO_GOAL")!=std::string::npos;}
};
int main(int argc,char**argv){
 const char*d=std::getenv("ROS_DOMAIN_ID"),*local=std::getenv("ROS_LOCALHOST_ONLY");
 if(!d||std::atoi(d)<100||std::atoi(d)>230||!local||std::string(local)!="1"){
  std::cerr<<"Refusing: isolated ROS_DOMAIN_ID 100..230 and ROS_LOCALHOST_ONLY=1 required\n";return 2;
 }
 rclcpp::init(argc,argv);auto n=std::make_shared<Contract>();
 for(int j=0;j<20;++j){n->tick("healthy");rclcpp::spin_some(n);std::this_thread::sleep_for(100ms);}
 const bool camera_contract=n->camera_subscription_contract();
 std::cout<<"camera_subscription "<<(camera_contract?"PASS":"FAIL")<<" enabled="<<n->camera_guard_enabled<<std::endl;
 if(n->arm_intent_case){
  const bool result=n->arm_intent_contract()&&camera_contract;
  rclcpp::shutdown();return result?0:1;
 }
 n->send_goal();
 if(n->map_content_case){const bool result=n->content_case()&&camera_contract;rclcpp::shutdown();return result?0:1;}
 bool ok=n->phase("healthy",4,true,false)&&camera_contract;
 ok=n->phase("bad_support",1,!n->camera_guard_enabled,n->camera_guard_enabled)&&ok;
 ok=n->phase("bad_status",.5,false,true)&&ok;
 ok=n->phase("wrong_frame",.5,false,true)&&ok;
 ok=n->phase("bad_layers",1,false,true)&&ok;
 // Older map stamps are intentionally ignored; let the last accepted map
 // expire before checking LAYERS_STALE rather than checking the prior error.
 n->pump("stale_map",3.0);
 ok=n->phase("stale_map",.5,false,true)&&ok;
 ok=n->phase("healthy",2,true,false)&&ok;
 ok=n->phase("future_odom",.5,false,true)&&ok;
 // The timestamp high-water mark is monotonic. A +10s sample must age out
 // before normal timestamps can be accepted again. Do not contaminate the
 // layer/map checks above, and verify actual local motion after recovery.
 n->pump("healthy",10.5);
 ok=n->phase("healthy",2,true,false)&&ok;
 ok=n->cancel_test()&&ok;
 ok=ok&&n->drives>50&&n->moving==0;
 std::cout<<"contract "<<(ok?"PASS":"FAIL")<<" commands="<<n->drives<<" moving="<<n->moving<<std::endl;
 rclcpp::shutdown();return ok?0:1;
}

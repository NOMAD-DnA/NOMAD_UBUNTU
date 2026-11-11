// Fake ROS endpoints + a separately built production controller. Never opens hardware.
#include <rclcpp/rclcpp.hpp>
#include <nomad_interfaces/msg/drive_command.hpp>
#include <nomad_interfaces/msg/vehicle_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <future>
#include <memory>
#include <stdexcept>
#include <utility>

namespace {
using Scalar=std_msgs::msg::Float64;
using Drive=nomad_interfaces::msg::DriveCommand;
using Vehicle=nomad_interfaces::msg::VehicleState;
using Telemetry=vesc_msgs::msg::VescStateStamped;
using Trigger=std_srvs::srv::Trigger;
int checks=0, failures=0;
void expect(bool ok,const char* message){++checks;std::cout<<(ok?"PASS ":"FAIL ")<<message<<std::endl;if(!ok)++failures;}
double wall(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class Rig {
public:
 explicit Rig(const char* controller){
  prefix_="/module_pause_contract_"+std::to_string(getpid());
  std::vector<std::string> args={controller,"--ros-args","-r","__ns:="+prefix_};
  auto remap=[&](const std::string& from,const std::string& to){args.insert(args.end(),{"-r",from+":="+prefix_+to});};
  remap("/nomad/planning/drive_command","/planning");remap("/nomad/control/vehicle_state","/vehicle_state");
  auto param=[&](const std::string& name,const std::string& value){args.insert(args.end(),{"-p",name+":="+value});};
  for(const auto& kv:std::vector<std::pair<std::string,std::string>>{
   {"mode","commission"},{"calibration_verified","false"},{"hardware_timeout_verified","true"},{"commissioning_setup_acknowledged","true"},
   {"commissioning.stationary_hold_s","0.2"},{"commissioning.max_duration_s","1.0"},{"commissioning.motor_max_erpm","1850.0"},
   {"commissioning.overspeed_erpm","2500.0"},{"commissioning.stall_watchdog_enabled","false"},
   {"planning.headless","true"},{"planning.allow_estimated_calibration","true"},{"planning.resume_after_stop","true"},
   {"planning.measured_speed_control","true"},{"planning.start_hold_s","0.0"},{"planning.restart_cooldown_s","0.0"},
   {"planning.minimum_running_erpm","700.0"},{"planning.feedback_limit_erpm","1200.0"},
   {"planning.erpm_per_mps","4000.0"},{"planning.servo_per_rad","0.5"},
   {"planning.max_mps","0.25"},{"planning.max_rad","0.3"}})param(kv.first,kv.second);
  for(const auto& kv:std::vector<std::pair<std::string,std::string>>{
   {"telemetry_topic","/telemetry"},{"motor_speed_topic","/speed"},{"motor_current_topic","/current"},{"servo_topic","/servo"},
   {"motor_duty_topic","/duty"},{"motor_brake_topic","/brake"},{"motor_position_topic","/position"},{"stability.imu_topic","/imu"}})param(kv.first,prefix_+kv.second);
  child_=fork();if(child_==0){std::vector<char*> argv;for(auto& a:args)argv.push_back(a.data());argv.push_back(nullptr);execv(controller,argv.data());_exit(127);}
  if(child_<0)throw std::runtime_error("fork failed");
  rclcpp::init(0,nullptr);
  executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  node_=std::make_shared<rclcpp::Node>("fake_endpoints",prefix_);
  telemetry_=node_->create_publisher<Telemetry>(prefix_+"/telemetry",rclcpp::QoS(1).reliable());
  planning_=node_->create_publisher<Drive>(prefix_+"/planning",rclcpp::QoS(1).reliable());
  speed_=node_->create_subscription<Scalar>(prefix_+"/speed",10,[this](Scalar::ConstSharedPtr m){if(m->data>0){++positive_;last_positive_=m->data;}});
  current_=node_->create_subscription<Scalar>(prefix_+"/current",10,[this](Scalar::ConstSharedPtr m){if(m->data==0)++releases_;});
  servo_=node_->create_subscription<Scalar>(prefix_+"/servo",10,[](Scalar::ConstSharedPtr){});
  state_=node_->create_subscription<Vehicle>(prefix_+"/vehicle_state",1,[this](Vehicle::ConstSharedPtr m){ready_=m->ready;fault_=m->fault;});
  stop_client_=node_->create_client<Trigger>(prefix_+"/nomad_vesc_trial/stop");reset_client_=node_->create_client<Trigger>(prefix_+"/nomad_vesc_trial/reset");
  executor_->add_node(node_);
 }
 ~Rig(){if(child_>0){kill(child_,SIGINT);int status=0;for(int i=0;i<100;++i){if(waitpid(child_,&status,WNOHANG)==child_)return;std::this_thread::sleep_for(std::chrono::milliseconds(10));}kill(child_,SIGKILL);waitpid(child_,&status,0);}}
 void pump(double seconds){const double end=wall()+seconds;while(wall()<end){
  if(wall()-last_publish_>.025){if(send_telemetry_){Telemetry t;t.header.stamp=node_->now();t.state.speed=0;t.state.voltage_input=12;t.state.current_motor=0;t.state.current_input=0;t.state.fault_code=fault_code_;telemetry_->publish(t);}
   if(send_){Drive d;d.header.stamp=node_->now()-rclcpp::Duration::from_seconds(stamp_age_);d.header.frame_id=frame_;d.target_speed=.05;d.target_steering_angle=.01;d.valid_for.nanosec=150000000;d.stop_requested=stop_;planning_->publish(d);}last_publish_=wall();}
  executor_->spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
 }}
 bool call(const rclcpp::Client<Trigger>::SharedPtr& client){if(!client->service_is_ready()){pump(.2);if(!client->service_is_ready())return false;}auto f=client->async_send_request(std::make_shared<Trigger::Request>());double end=wall()+1.;while(wall()<end&&f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)pump(.01);return f.wait_for(std::chrono::milliseconds(0))==std::future_status::ready&&f.get()->success;}
 void resume(const char* name){send_=true;stop_=false;frame_="base_link";stamp_age_=0;int before=positive_;pump(.35);expect(positive_>before&&last_positive_>=700-1e-6&&ready_&&!fault_,name);}
 void no_motion(const char* name,bool expect_release=true){pump(.22);int before=positive_;int release_before=releases_;pump(.16);expect(positive_==before&&(!expect_release||releases_>release_before)&&ready_&&!fault_,name);}
 void run(){
  const double discovery_deadline=wall()+4.;do{pump(.1);}while(!ready_&&wall()<discovery_deadline);pump(.3);expect(ready_&&!fault_,"hardware ready without positive planner command");expect(positive_==0,"idle creates no motor motion");
  send_=true;stop_=true;no_motion("fresh idle STOP keeps ready without taking output ownership",false);
  stamp_age_=1.;no_motion("stale idle STOP keeps ready without taking output ownership",false);
  resume("positive 200 ERPM request starts at running floor 700");
  stop_=true;stamp_age_=1.;no_motion("stale STOP during motion releases without reset latch");
  resume("new valid command resumes after stale STOP without reset");
  send_=false;no_motion("expired 0.15-second planner lease releases and stays ready");
  resume("fresh command resumes after lease expiry without reset");
  send_=false;pump(.25);send_=true;stop_=true;no_motion("STOP arriving after lease expiry remains a soft module pause");
  resume("fresh command resumes after post-expiry STOP without reset");
  frame_="invalid";no_motion("new invalid-frame command releases without reset latch");
  resume("fresh command resumes after invalid frame without reset");
  send_=false;pump(.25);send_telemetry_=false;int gap_before=positive_;pump(.45);
  expect(positive_==gap_before&&!fault_&&!ready_,"paused telemetry gap releases without reset fault");
  send_telemetry_=true;pump(.15);resume("paused telemetry restored resumes from fresh command without reset");
  send_telemetry_=false;pump(.45);gap_before=positive_;pump(.15);
  expect(positive_==gap_before&&!fault_&&!ready_,"running telemetry gap releases without reset fault");
  send_telemetry_=true;pump(.15);resume("running telemetry restored resumes without reset");
  send_=false;pump(.25);current_.reset();pump(.25);gap_before=positive_;send_=true;pump(.25);
  expect(positive_==gap_before&&!fault_&&!ready_,"missing driver endpoint remains output-zero wait");
  current_=node_->create_subscription<Scalar>(prefix_+"/current",10,[this](Scalar::ConstSharedPtr m){if(m->data==0)++releases_;});
  pump(.3);resume("driver endpoint restored resumes without reset");
  send_=false;expect(call(stop_client_),"explicit operator STOP service succeeds");pump(.15);int before=positive_;send_=true;pump(.3);expect(positive_==before&&!ready_,"fresh planner commands cannot clear explicit operator STOP");
  send_=false;pump(.3);expect(call(reset_client_),"explicit operator STOP can be reset with fake stationary telemetry");resume("fresh command resumes after explicit reset");
  fault_code_=1;pump(.15);before=positive_;pump(.2);expect(positive_==before&&fault_&&!ready_,"VESC hardware fault releases and remains latched despite fresh commands");
  fault_code_=0;pump(.3);expect(!ready_&&positive_==before,"clearing telemetry fault alone does not clear hardware latch");
 }
private:
 pid_t child_=-1;std::string prefix_,frame_="base_link";rclcpp::Node::SharedPtr node_;std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
 rclcpp::Publisher<Telemetry>::SharedPtr telemetry_;rclcpp::Publisher<Drive>::SharedPtr planning_;
 rclcpp::Subscription<Scalar>::SharedPtr speed_,current_,servo_;rclcpp::Subscription<Vehicle>::SharedPtr state_;
 rclcpp::Client<Trigger>::SharedPtr stop_client_,reset_client_;bool ready_=false,fault_=false,send_=false,stop_=false,send_telemetry_=true;int fault_code_=0,positive_=0,releases_=0;double last_publish_=0,stamp_age_=0,last_positive_=0;
};
}
int main(int argc,char** argv){
 const char* domain=std::getenv("ROS_DOMAIN_ID"),*local=std::getenv("ROS_LOCALHOST_ONLY"),*rmw=std::getenv("RMW_IMPLEMENTATION");
 if(argc!=2||!domain||std::string(domain)!="229"||!local||std::string(local)!="1"||!rmw||std::string(rmw)!="rmw_fastrtps_cpp"){
  std::cerr<<"REFUSED: controller executable plus domain 229, localhost 1 and FastDDS required before init/fork\n";return 2;}
 for(const char* key:{"FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE","ROS_DISCOVERY_SERVER","ROS_STATIC_PEERS"})unsetenv(key);
 setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);
 // Fork before initializing this process's DDS threads.
 try{Rig rig(argv[1]);rig.run();}catch(const std::exception& e){std::cerr<<"EXCEPTION "<<e.what()<<'\n';++failures;}
 if(rclcpp::ok())rclcpp::shutdown();std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}

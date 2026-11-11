// Isolated fake VESC/DriveCommand endpoints only; no driver or serial access.
#define NOMAD_CONTROLLER_NO_MAIN
#include "../src/controller_node.cpp"
#include <cstdlib>
#include <functional>
#include <vector>

namespace {
int checks=0, failures=0;
void expect(bool value,const char* name){++checks;if(!value){++failures;std::cerr<<"FAIL: "<<name<<'\n';}}
class FloorRig {
public:
 FloorRig(){
  prefix_="/rpm_floor_contract_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  fake_=std::make_shared<rclcpp::Node>("fake_vesc",prefix_);
  telemetry_=fake_->create_publisher<nomad::VescState>(prefix_+"/telemetry",1);
  planner_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
  speed_=fake_->create_subscription<nomad::Scalar>(prefix_+"/speed",10,[this](nomad::Scalar::ConstSharedPtr m){outputs_.push_back(m->data);});
  current_=fake_->create_subscription<nomad::Scalar>(prefix_+"/current",10,[this](nomad::Scalar::ConstSharedPtr m){++releases_;expect(m->data==0,"release uses zero current");});
  servo_=fake_->create_subscription<nomad::Scalar>(prefix_+"/servo",10,[](nomad::Scalar::ConstSharedPtr){});
  state_=fake_->create_subscription<nomad_interfaces::msg::VehicleState>(prefix_+"/vehicle_state",1,[this](nomad_interfaces::msg::VehicleState::ConstSharedPtr m){ready_=m->ready;});
  status_sub_=fake_->create_subscription<std_msgs::msg::String>(prefix_+"/nomad_vesc_trial/status_json",1,[this](std_msgs::msg::String::ConstSharedPtr m){status_=nlohmann::json::parse(m->data);});
  rclcpp::NodeOptions o;
  o.arguments({"--ros-args","-r","__ns:="+prefix_,"-r","/nomad/planning/drive_command:="+prefix_+"/planning","-r","/nomad/control/vehicle_state:="+prefix_+"/vehicle_state"});
  o.parameter_overrides({
   {"mode","commission"},{"calibration_verified",false},{"hardware_timeout_verified",true},{"commissioning_setup_acknowledged",true},
   {"commissioning.stationary_hold_s",.2},{"commissioning.max_duration_s",1.0},{"commissioning.motor_max_erpm",1000.0},
   {"commissioning.overspeed_erpm",2500.0},{"commissioning.stall_watchdog_enabled",false},
   {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
   {"planning.measured_speed_control",true},{"planning.start_hold_s",0.0},{"planning.restart_cooldown_s",0.0},
   {"planning.minimum_running_erpm",550.0},{"planning.stop_request_erpm",100.0},{"planning.feedback_limit_erpm",1200.0},{"planning.startup_boost_enabled",false},
   {"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
   {"telemetry_topic",prefix_+"/telemetry"},{"motor_speed_topic",prefix_+"/speed"},{"motor_current_topic",prefix_+"/current"},
   {"servo_topic",prefix_+"/servo"},{"motor_duty_topic",prefix_+"/duty"},{"motor_brake_topic",prefix_+"/brake"},
   {"motor_position_topic",prefix_+"/position"},{"stability.imu_topic",prefix_+"/imu"}});
  controller_=std::make_shared<nomad::ControllerNode>(o);executor_.add_node(fake_);executor_.add_node(controller_);
 }
 ~FloorRig(){controller_->prepareShutdown();}
 void pump(double duration){
  const double end=now()+duration;
  while(now()<end){
   if(now()-last_sent_>.025){
    nomad::VescState t;t.header.stamp=fake_->now();t.state.speed=measured_;t.state.voltage_input=12;t.state.current_motor=0;t.state.current_input=0;t.state.fault_code=0;telemetry_->publish(t);
    if(send_){nomad_interfaces::msg::DriveCommand c;c.header.stamp=fake_->now();c.header.frame_id="base_link";c.target_speed=request_/4000.;c.target_steering_angle=.01;c.valid_for.nanosec=300000000;c.stop_requested=stop_;planner_->publish(c);}
    last_sent_=now();
   }
   executor_.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
 }
 void run(){
  pump(.8);expect(ready_,"fake hardware is ready independently of a positive planner request");expect(outputs_.empty(),"idle does not synthesize minimum motion");
  send_=true;request_=100;pump(.22);
  expect(outputs_.empty(),"initial request at the inclusive 100 ERPM boundary does not synthesize motion");
  request_=101;pump(.22);expect(!outputs_.empty(),"101 request just above the deadzone produces fake speed output");
  if(!outputs_.empty())expect(std::abs(outputs_.front()-550.)<1e-6,"first 101 request is transmitted as 550 without a low-ERPM ramp");
  const auto normal_begin=outputs_.size();request_=200;pump(.22);
  expect(all_since(normal_begin,550.,550.),"normal 200 request remains at 550");
  request_=100;pump(.12);const auto boundary_end=outputs_.size();const int boundary_releases=releases_;pump(.12);
  expect(outputs_.size()==boundary_end&&releases_>boundary_releases,"returning to inclusive 100 boundary releases instead of retaining 550");
  request_=200;pump(.22);expect(outputs_.size()>boundary_end&&std::abs(outputs_.back()-550.)<1e-6,"fresh 200 request resumes at 550 after boundary stop");
  const auto high_begin=outputs_.size();request_=700;pump(.35);expect(outputs_.size()>high_begin&&std::abs(outputs_.back()-700.)<1e-6,"700 request is preserved above the running minimum");
  request_=0;pump(.12);const auto zero_end=outputs_.size();const int zero_releases=releases_;pump(.12);
  expect(outputs_.size()==zero_end&&releases_>zero_releases,"zero target releases and never becomes minimum motion");
  request_=200;pump(.2);expect(!outputs_.empty()&&std::abs(outputs_.back()-550.)<1e-6,"fresh positive target resumes at 550 after zero");
  stop_=true;pump(.12);const auto stop_end=outputs_.size();const auto stop_releases=releases_;pump(.12);
  expect(outputs_.size()==stop_end&&releases_>stop_releases,"STOP immediately releases instead of retaining minimum motion");
  stop_=false;pump(.22);expect(outputs_.size()>stop_end&&std::abs(outputs_.back()-550.)<1e-6&&!status_.value("planning_reset_required",true),"fresh command resumes at 550 without reset after planner pause");
  measured_=1400;pump(.3);expect(!outputs_.empty()&&outputs_.back()<550&&outputs_.back()>=0,"measured soft overspeed can reduce output below the 550 floor");
  expect(!status_.value("planning_reset_required",true)&&status_.value("autonomous_active",false),"soft overspeed remains resumable rather than latching stop");
  measured_=0;pump(.35);expect(!outputs_.empty()&&std::abs(outputs_.back()-550.)<1e-6,"speed recovery restores normal 550 output");
  send_=false;pump(.45);const auto expiry_end=outputs_.size();const auto expiry_releases=releases_;pump(.12);
  expect(outputs_.size()==expiry_end&&releases_>expiry_releases&&!status_.value("autonomous_active",true),"expired planner lease releases instead of retaining minimum motion");
 }
private:
 static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 bool all_since(std::size_t begin,double lower,double upper){return outputs_.size()>begin&&std::all_of(outputs_.begin()+begin,outputs_.end(),[=](double v){return v>=lower-1e-6&&v<=upper+1e-6;});}
 std::string prefix_;rclcpp::Node::SharedPtr fake_;std::shared_ptr<nomad::ControllerNode> controller_;
 rclcpp::Publisher<nomad::VescState>::SharedPtr telemetry_;rclcpp::Publisher<nomad_interfaces::msg::DriveCommand>::SharedPtr planner_;
 rclcpp::Subscription<nomad::Scalar>::SharedPtr speed_,current_,servo_;rclcpp::Subscription<nomad_interfaces::msg::VehicleState>::SharedPtr state_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;rclcpp::executors::SingleThreadedExecutor executor_;
 nlohmann::json status_=nlohmann::json::object();std::vector<double> outputs_;bool ready_=false,send_=false,stop_=false;double request_=0,measured_=0,last_sent_=0;int releases_=0;
};
}
int main(int argc,char** argv){
 const char* domain=std::getenv("ROS_DOMAIN_ID");const char* local=std::getenv("ROS_LOCALHOST_ONLY");const char* rmw=std::getenv("RMW_IMPLEMENTATION");
 if(!domain||std::string(domain)!="232"||!local||std::string(local)!="1"||!rmw||std::string(rmw)!="rmw_fastrtps_cpp"){
  std::cerr<<"Refusing test: requires domain 232, localhost only, FastDDS before ROS initialization\n";return 2;}
 for(const char* name:{"FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE","ROS_DISCOVERY_SERVER","ROS_STATIC_PEERS"})unsetenv(name);
 setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);
 rclcpp::init(argc,argv);
 try{{FloorRig rig;rig.run();}}catch(const std::exception& e){++failures;std::cerr<<"EXCEPTION: "<<e.what()<<'\n';}
 rclcpp::shutdown();std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}

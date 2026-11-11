// 실제 노드 구현을 가짜 VESC와 같은 executor에서 시험합니다. 시험 토픽은 고유 namespace에 격리합니다.
#include "nomad_control_cpp/controller_node.hpp"
#include "json.hpp"
#ifdef NOMAD_ENABLE_PLANNING
#include <nomad_interfaces/msg/drive_command.hpp>
#include <nomad_interfaces/msg/vehicle_state.hpp>
#endif
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <functional>
#include <limits>
#define NOMAD_GUI_NO_MAIN
#include "../src/gui_server.cpp"

using namespace std;

using nomad::steadySeconds;

namespace {
int checks = 0, failures = 0;
void expect(bool ok, const char* name) {
    ++checks;
    if (!ok) { ++failures; cerr << "FAIL: " << name << '\n'; }
}

class RosRig {
public:
    explicit RosRig(bool bench, bool commission = false, bool setup_acknowledged = true,
                    bool timeout_verified = true, bool stall_watchdog = false,
                    const vector<rclcpp::Parameter>& extra_parameters = {}) {
        const auto token = chrono::steady_clock::now().time_since_epoch().count();
        prefix_ = "/nomad_cpp_test_" + to_string(token);
        fake_ = make_shared<rclcpp::Node>("fake_vesc", prefix_);
        pub_ = fake_->create_publisher<nomad::VescState>(prefix_ + "/sensors/core", 1);
        for (const auto& suffix : {"speed", "current"}) {
            subs_.push_back(fake_->create_subscription<nomad::Scalar>(prefix_ + "/commands/motor/" + suffix, 10,
                [this, name = string(suffix)](nomad::Scalar::ConstSharedPtr msg) {
                    if (name == "speed") {
                        ++speed_messages_; if (msg->data != 0.0) ++moving_messages_;
                        observed_motor_erpm_.push_back(msg->data);
                    }
                    else { ++release_messages_; expect(msg->data == 0.0, "release is zero current"); }
                }));
        }
        subs_.push_back(fake_->create_subscription<nomad::Scalar>(prefix_ + "/commands/servo/position", 10,
            [this](nomad::Scalar::ConstSharedPtr) { ++servo_messages_; }));
        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__ns:=" + prefix_,
            "-r","/nomad/planning/drive_command:="+prefix_+"/planning",
            "-r","/nomad/control/vehicle_state:="+prefix_+"/vehicle_state"});
        options.parameter_overrides({
            {"planning.start_hold_s",0.0},{"planning.restart_cooldown_s",0.0},
            {"mode", commission ? "commission" : (bench ? "bench" : "monitor")},
            {"calibration_verified", !commission}, {"hardware_timeout_verified", timeout_verified},
            {"commissioning_setup_acknowledged", setup_acknowledged},
            {"commissioning.stationary_hold_s", 0.2}, {"commissioning.max_duration_s", 1.0},
            {"commissioning.motor_max_erpm", 1000.0}, {"commissioning.overspeed_erpm", 1500.0},
            {"commissioning.stall_watchdog_enabled", stall_watchdog}, {"commissioning.stall_timeout_s", 0.5},
            {"target_speed_mps", 0.1}, {"target_steering_rad", 0.0},
            {"trial_duration_s", 1.0}, {"stationary_hold_s", 0.2}, {"steering_settle_s", 0.1},
            {"calibration.speed_gain", 6000.0}, {"calibration.speed_offset", 0.0},
            {"calibration.steering_gain", -0.6}, {"calibration.steering_offset", 0.52},
            {"calibration.command_erpm_sign", 1}, {"calibration.feedback_erpm_sign", -1},
            {"calibration.speed_min_mps", 0.0}, {"calibration.speed_max_mps", 0.4},
            {"calibration.steering_min_rad", -0.2}, {"calibration.steering_max_rad", 0.2},
            {"calibration.servo_min", 0.4}, {"calibration.servo_max", 0.64},
            {"calibration.motor_erpm_min", -3000.0}, {"calibration.motor_erpm_max", 3000.0},
            {"telemetry_topic", prefix_ + "/sensors/core"},
            {"motor_speed_topic", prefix_ + "/commands/motor/speed"},
            {"motor_current_topic", prefix_ + "/commands/motor/current"},
            {"servo_topic", prefix_ + "/commands/servo/position"},
            {"motor_duty_topic", prefix_ + "/commands/motor/duty_cycle"},
            {"motor_brake_topic", prefix_ + "/commands/motor/brake"},
            {"motor_position_topic", prefix_ + "/commands/motor/position"}});
        auto overrides = options.parameter_overrides();
        overrides.emplace_back("stability.imu_topic", prefix_+"/imu_body");
        overrides.insert(overrides.end(), extra_parameters.begin(), extra_parameters.end());
        options.parameter_overrides(overrides);
        controller_ = nomad::ControllerNode::create(options);
        status_sub_ = fake_->create_subscription<std_msgs::msg::String>(prefix_ + "/nomad_vesc_trial/status_json", 1,
            [this](std_msgs::msg::String::ConstSharedPtr message) { status_ = nomad::Json::parse(message->data); });
        executor_.add_node(fake_);
        executor_.add_node(controller_);
    }

    ~RosRig() { controller_->prepareShutdown(); }

    void pump(double seconds) {
        const auto end = steadySeconds() + seconds;
        while (steadySeconds() < end) {
            if (send_telemetry_ && steadySeconds() - last_sent_ > 0.025) {
                nomad::VescState msg;
                msg.header.stamp = fake_->now();
                msg.state.speed = telemetry_erpm_;
                msg.state.current_motor = 0.0;
                msg.state.current_input = 0.0;
                msg.state.voltage_input = 12.0;
                msg.state.fault_code = telemetry_fault_;
                pub_->publish(msg);
                if (imu_pub_ && send_imu_) {
                    sensor_msgs::msg::Imu imu;
                    imu.header.stamp=fake_->now(); imu.header.frame_id=imu_frame_;
                    imu.orientation.w=cos(imu_roll_/2); imu.orientation.x=sin(imu_roll_/2);
                    imu.orientation_covariance[0]=imu.orientation_covariance[4]=imu.orientation_covariance[8]=0.001;
                    imu.linear_acceleration.z=9.80665;
                    imu_pub_->publish(imu);
                }
                last_sent_ = steadySeconds();
#ifdef NOMAD_ENABLE_PLANNING
                if(planning_pub_&&send_planning_){
                    nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();cmd.header.frame_id="base_link";
                    cmd.target_speed=planner_speed_;cmd.target_steering_angle=.01;cmd.valid_for.nanosec=300000000;
                    planning_pub_->publish(cmd);
                }
#endif
            }
            executor_.spin_some();
            this_thread::sleep_for(chrono::milliseconds(3));
        }
    }

    bool call(const string& action) {
        auto client = fake_->create_client<nomad::Trigger>(prefix_ + "/nomad_vesc_trial/" + action);
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.02);
        if (!client->service_is_ready()) throw runtime_error("test service discovery timed out");
        auto future = client->async_send_request(make_shared<nomad::Trigger::Request>());
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline)
            pump(0.01);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready)
            throw runtime_error("test service response timed out");
        return future.get()->success;
    }

    bool manual(int action, const string& session, uint64_t sequence = 1, double age = 0.0) {
        auto client = fake_->create_client<nomad::Manual>(prefix_ + "/nomad_vesc_trial/manual_control");
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.01);
        auto request = make_shared<nomad::Manual::Request>();
        request->action = static_cast<uint8_t>(action); request->session = session; request->sequence = sequence;
        request->stamp = rclcpp::Time(static_cast<int64_t>((fake_->now().seconds() - age) * 1e9));
        request->speed_mps = 0.1; request->steering_rad = 0.0; request->duration_s = 1.0;
        auto future = client->async_send_request(request);
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline) pump(0.005);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready) throw runtime_error("manual service timeout");
        return future.get()->accepted;
    }

    bool commission(int action, int axis, const string& session, uint64_t sequence = 1,
                    double erpm = 300.0, double servo = 0.50, double duration = 1.0, double age = 0.0) {
        auto client = fake_->create_client<nomad::Commissioning>(prefix_ + "/nomad_vesc_trial/commissioning_control");
        const double deadline = steadySeconds() + 3.0;
        while (!client->service_is_ready() && steadySeconds() < deadline) pump(0.01);
        auto request = make_shared<nomad::Commissioning::Request>();
        request->action = static_cast<uint8_t>(action); request->session = session; request->sequence = sequence;
        if (action != nomad::Commissioning::Request::STOP) {
            request->axis = static_cast<uint8_t>(axis);
            request->stamp = rclcpp::Time(static_cast<int64_t>((fake_->now().seconds() - age) * 1e9));
            request->motor_erpm = erpm; request->servo_position = servo; request->duration_s = duration;
        }
        auto future = client->async_send_request(request);
        while (future.wait_for(chrono::milliseconds(0)) != future_status::ready && steadySeconds() < deadline) pump(0.005);
        if (future.wait_for(chrono::milliseconds(0)) != future_status::ready)
            throw runtime_error("commissioning service timeout");
        return future.get()->accepted;
    }

    void runCommissionGate() {
        pump(0.7);
        expect(!commission(1, 1, string(32, '1')), "raw setup/timeout gate refuses start");
        pump(0.1);
        expect(speed_messages_ == 0 && servo_messages_ == 0 && release_messages_ == 0, "raw gate refusal has no output");
    }

#ifdef NOMAD_ENABLE_PLANNING
    void runMinimumRunningFloor() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.75);
        const auto outputs_at_least=[this](size_t first,double floor) {
            if(first==observed_motor_erpm_.size())return false;
            for(auto i=first;i<observed_motor_erpm_.size();++i)
                if(observed_motor_erpm_[i]+1e-6<floor)return false;
            return true;
        };

        auto first=observed_motor_erpm_.size();
        planner_speed_=1.0/4000.0;send_planning_=true;pump(.20);
        expect(abs(status_.value("planning_requested_erpm",0.0)-1.0)<1e-6&&
               outputs_at_least(first,700.0),
               "positive 1 ERPM planner request publishes at least the configured 700 ERPM floor");
        first=observed_motor_erpm_.size();pump(.95);
        expect(outputs_at_least(first,700.0)&&status_.value("autonomous_active",false),
               "700 ERPM floor remains active beyond the former 0.8 second startup duration");

        first=observed_motor_erpm_.size();planner_speed_=206.0/4000.0;pump(.15);
        expect(abs(status_.value("planning_requested_erpm",0.0)-206.0)<1e-6&&
               outputs_at_least(first,700.0),
               "206 ERPM planner request continues publishing at least 700 ERPM");
        first=observed_motor_erpm_.size();planner_speed_=930.0/4000.0;pump(.40);
        expect(outputs_at_least(first,700.0)&&observed_motor_erpm_.back()+1e-6>=930.0&&
               abs(status_.value("target_motor_erpm",0.0)-930.0)<1e-6,
               "930 ERPM planner request ramps from the floor to its larger requested output");

        const int zero_releases=release_messages_;planner_speed_=0.0;pump(.06);
        expect(release_messages_>zero_releases,"zero planner request promptly publishes zero-current release");
        const int zero_moves=moving_messages_;pump(.10);
        expect(moving_messages_==zero_moves&&status_.value("commanded_motor_erpm",-1.0)==0.0&&
               status_.value("target_motor_erpm",-1.0)==0.0,
               "zero planner request stays released without a low-speed deceleration ramp");
        first=observed_motor_erpm_.size();planner_speed_=206.0/4000.0;pump(.15);
        expect(outputs_at_least(first,700.0),"fresh positive planning command resumes the running floor after zero");

        send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.header.frame_id="base_link";
        stop.valid_for.nanosec=300000000;stop.stop_requested=true;
        const int stop_releases=release_messages_;planning_pub_->publish(stop);pump(.06);
        expect(release_messages_>stop_releases,"LPP STOP promptly publishes zero-current release");
        const int stopped=moving_messages_;pump(.10);
        expect(moving_messages_==stopped&&status_.value("planning_soft_stopped",false)&&
               !status_.value("planning_reset_required",true),
               "LPP STOP remains released and keeps its existing resumable pause behavior");
        first=observed_motor_erpm_.size();planner_speed_=1.0/4000.0;send_planning_=true;pump(.16);
        expect(outputs_at_least(first,700.0)&&status_.value("autonomous_active",false)&&
               !status_.value("planning_reset_required",true),
               "fresh planning command resumes at the floor without a startup cooldown");
    }

    // 실제 드라이버 없이 가짜 telemetry/DriveCommand만으로 새 속도 피드백 분기를 검사합니다.
    void runMeasuredSpeedFeedback() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.75); // 발견·초기 정지 확인은 이동 명령 없이 끝냅니다.
        planner_speed_=.0515;send_planning_=true;pump(.18); // 206 ERPM, 기존 550 입력 gate 미만
        expect(moving_messages_>0,"measured-speed mode starts low valid request without fixed 750ms dwell");
        expect(abs(status_.value("planning_requested_erpm",0.0)-206.0)<1e-6&&
               status_.value("target_motor_erpm",0.0)>0&&status_.value("target_motor_erpm",1000.0)<=206.000001,
               "measured-speed mode preserves 206 ERPM request without blocking or boosting it to 550");

        telemetry_erpm_=300.0;send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.header.frame_id="base_link";
        stop.valid_for.nanosec=300000000;stop.stop_requested=true;planning_pub_->publish(stop);pump(.08);
        const int paused=moving_messages_;
        expect(!status_.value("planning_reset_required",true),"fresh LPP pause stays resumable with rolling feedback");
        send_planning_=true;pump(.14);
        expect(moving_messages_>paused&&status_.value("autonomous_active",false)&&
               !status_.value("planning_reset_required",true),
               "fresh LPP command resumes while rolling without 500ms stationary or one-second cooldown wait");

        planner_speed_=.232625;telemetry_erpm_=300.0;pump(.25); // requested 930.5 ERPM
        telemetry_erpm_=1438.0;pump(.36); // 과거 1200/0.2s latch 조건, hard 2500 미만
        const double reduced=status_.value("target_motor_erpm",-1.0);
        expect(!status_.value("planning_reset_required",true)&&status_.value("state","")!="FAULT"&&
               status_.value("autonomous_active",false),
               "sustained modest overspeed reduces output instead of latching reset or fault");
        expect(reduced>=0.0&&reduced<930.5&&status_.value("commanded_motor_erpm",1000.0)<930.5,
               "modest measured overspeed actually reduces fake VESC speed output below requested 930.5 ERPM");
        telemetry_erpm_=300.0;pump(.35);
        expect(status_.value("target_motor_erpm",0.0)>reduced&&
               status_.value("target_motor_erpm",1000.0)<=930.500001&&
               !status_.value("planning_reset_required",true)&&status_.value("autonomous_active",false),
               "speed recovery automatically restores bounded demand without operator reset");
    }

    void runMeasuredSpeedSafety(const string& failure) {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.75);
        planner_speed_=.0515;send_planning_=true;pump(.18);
        expect(moving_messages_>0,"measured-speed safety fixture begins with fake output only");
        if(failure=="operator STOP") {
            expect(call("stop"),"measured-speed mode accepts explicit operator STOP");pump(.13);
        }
        else if(failure=="invalid command") {
            send_planning_=false;
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();cmd.header.frame_id="base_link";
            cmd.valid_for.nanosec=300000000;cmd.target_speed=numeric_limits<double>::quiet_NaN();
            planning_pub_->publish(cmd);pump(.13);
        } else if(failure=="command expiry") {send_planning_=false;pump(.45);}
        else if(failure=="telemetry timeout") {send_telemetry_=false;pump(.45);}
        else if(failure=="VESC fault") {
            telemetry_fault_=1;
            // observe()가 먼저 FAULT를 만들고 다음 제어 tick이 headless latch를 갱신합니다.
            // 수신 status는 10Hz이므로 두 번의 발행까지 진행한 뒤 두 상태를 함께 검사합니다.
            pump(.25);
        }
        else if(failure=="hard overspeed") {telemetry_erpm_=2501.0;pump(.25);} // same two-status window as VESC fault
        else throw runtime_error("unknown measured-speed safety case");
        const int stopped=moving_messages_;
        expect(status_.value("planning_reset_required",false)&&!status_.value("autonomous_active",true),
               ("measured-speed mode preserves latched stop for "+failure).c_str());
        send_planning_=true;send_telemetry_=true;telemetry_erpm_=0;telemetry_fault_=0;pump(.25);
        expect(moving_messages_==stopped&&status_.value("planning_reset_required",false),
               ("fresh commands cannot undo "+failure+" without reset").c_str());
    }

    void runStableHeadlessStart() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        planner_speed_=.1;pump(.8);
        expect(moving_messages_==0,"autonomy never boosts unsupported low speed to minimum ERPM");
        expect(status_.value("planning_minimum_start_erpm",0.0)==550,"autonomy exposes startup threshold");
        planner_speed_=.2;pump(.1);expect(moving_messages_==0,"short valid burst cannot start motor");
        send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.header.frame_id="base_link";
        stop.valid_for.nanosec=300000000;stop.stop_requested=true;planning_pub_->publish(stop);pump(.1);
        send_planning_=true;pump(.1);expect(moving_messages_==0,"pause resets stability dwell");
        pump(.5);expect(moving_messages_>0,"continuous qualifying motion starts without GUI");
        planner_speed_=.1;pump(.15);
        expect(status_.value("target_motor_erpm",-1.0)==0,"low requested speed inhibits output instead of boosting");
        planner_speed_=.2;const int before=moving_messages_;pump(.1);
        expect(moving_messages_==before,"zero-to-motion restart observes cooldown and dwell");
        pump(.5);expect(moving_messages_>before,"motion resumes after continuous stable stream");
        telemetry_erpm_=1400;pump(.5); // 0.2 s rule + telemetry + 10 Hz status publication
        expect(status_.value("planning_reset_required",false),"autonomy sustained measured overspeed latches stop");
        expect(!status_.at("planning_requested_erpm").is_number()||status_.at("planning_requested_erpm").get<double>()!=1400,
               "requested and measured ERPM remain distinct");
    }
    void runHeadlessResume() {
        bool vehicle_ready=false;
        auto vehicle=fake_->create_subscription<nomad_interfaces::msg::VehicleState>(prefix_+"/vehicle_state",1,
            [&vehicle_ready](nomad_interfaces::msg::VehicleState::ConstSharedPtr msg){
                vehicle_ready=msg->ready;
                expect(!msg->steering_valid&&!msg->speed_valid,"initial mapping never pretends measured feedback");
            });
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        send_planning_=false;pump(.6);
        const auto stop=[&]{
            nomad_interfaces::msg::DriveCommand cmd;cmd.header.stamp=fake_->now();
            cmd.header.frame_id="base_link";cmd.valid_for.nanosec=300000000;cmd.stop_requested=true;
            planning_pub_->publish(cmd);pump(.13);
        };
        const int initial_release=release_messages_;
        stop();
        expect(moving_messages_==0&&servo_messages_==0&&release_messages_==initial_release,
               "initial planner STOP has no motor/servo output");
        expect(!status_.value("planning_reset_required",true),"initial planner STOP keeps headless ready");
        expect(vehicle_ready,"initial planner STOP keeps VehicleState ready");
        send_planning_=true;pump(.4);
        expect(moving_messages_>0,"fresh command starts after initial planner STOP");
        send_planning_=false;telemetry_erpm_=100;stop();const int stopped=moving_messages_;
        expect(status_.value("planning_soft_stopped",false)&&!status_.value("planning_reset_required",true),
               "normal planner STOP is a resumable pause");
        expect(vehicle_ready,"soft pause retains VehicleState ready");
        send_planning_=true;pump(.2);
        expect(moving_messages_==stopped,"pause cannot resume before stationary hold");
        telemetry_erpm_=0;pump(.45);
        expect(moving_messages_>stopped,"fresh command resumes after stationary hold");
        send_planning_=false;pump(.45);const int expired=moving_messages_;
        pump(.1);
        expect(moving_messages_==expired&&!status_.value("planning_reset_required",true),
               "planner lease expiry releases output without a reset latch");
        expect(vehicle_ready,"module expiry retains VehicleState ready");
        send_planning_=true;pump(.4);
        expect(moving_messages_>expired&&!status_.value("planning_reset_required",true),
               "fresh planner stream resumes after lease expiry without reset");
        send_planning_=false;pump(.45);stop();const int stopped_again=moving_messages_;
        send_planning_=true;pump(.4);
        expect(moving_messages_>stopped_again&&!status_.value("planning_reset_required",true),
               "STOP after expired command remains an automatically resumable module pause");
        expect(call("stop"),"operator STOP accepted with pause resume enabled");const int operator_stop=moving_messages_;
        send_planning_=false;stop();send_planning_=true;pump(.2);
        expect(moving_messages_==operator_stop&&status_.value("planning_reset_required",false),
               "planner pause cannot undo operator STOP");
        expect(!status_.value("planning_calibration_verified",true),"pause resume never fakes physical calibration");
    }
    void runHeadless() {
        pump(.6);expect(moving_messages_==0,"headless waits for planner");
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        pump(.8);expect(moving_messages_>0,"headless starts without GUI");
        expect(!commission(1,3,string(32,'c'),1,0,.5,0),"manual cannot steal headless output");
        const int first=moving_messages_;pump(2.2);
        expect(moving_messages_>first&&status_.value("autonomous_active",false),"headless continues beyond GUI heartbeat");
        send_planning_=false;pump(.45);const int stopped=moving_messages_;
        send_planning_=true;pump(.3);
        expect(moving_messages_==stopped&&status_.value("planning_reset_required",false),"headless expiry latches stop");
        expect(call("reset"),"headless reset re-arms without GUI");pump(.4);
        expect(moving_messages_>stopped,"fresh planning resumes after reset");
        expect(call("stop"),"global STOP works in headless");const int stopped2=moving_messages_;pump(.4);
        expect(moving_messages_==stopped2,"continuing planner cannot undo global STOP");
        expect(call("reset"),"reset before planner STOP");pump(.4);
        send_planning_=false;
        nomad_interfaces::msg::DriveCommand stop;stop.header.stamp=fake_->now();stop.stop_requested=true;
        planning_pub_->publish(stop);pump(.15);const int stopped3=moving_messages_;
        send_planning_=true;pump(.3);
        expect(moving_messages_==stopped3&&status_.value("planning_reset_required",false),"LPP stop_requested latches headless stop");
        expect(call("reset"),"reset before session STOP");pump(.4);
        expect(commission(3,0,string(32,'d')),"session STOP accepted without GUI owner");
        const int stopped4=moving_messages_;pump(.3);
        expect(moving_messages_==stopped4,"session STOP halts headless output");
        expect(!status_.value("planning_calibration_verified",true),"estimated execution does not fake calibration verification");
    }
    void runPlanning() {
        planning_pub_=fake_->create_publisher<nomad_interfaces::msg::DriveCommand>(prefix_+"/planning",rclcpp::QoS(1).reliable());
        auto vehicle=fake_->create_subscription<nomad_interfaces::msg::VehicleState>(prefix_+"/vehicle_state",1,
            [](nomad_interfaces::msg::VehicleState::ConstSharedPtr msg){expect(!msg->steering_valid,"servo command never presented as measured angle");});
        pump(.8);
        const string id(32,'a'),other(32,'b');
        expect(moving_messages_==0,"planner reception alone cannot actuate");
        expect(commission(1,4,id,1,0,.5,0),"explicit LPP session starts");
        pump(.15);expect(moving_messages_>0,"LPP converted ERPM reaches fake VESC");
        expect(!commission(1,3,other,1,0,.5,0),"manual cannot steal autonomous owner");
        expect(commission(2,4,id,2,0,.5,0),"operator LPP heartbeat accepted");
        send_planning_=false;pump(.45);
        expect(status_.value("state","")!="RUNNING"&&!status_.value("autonomous_active",true),"expired LPP stops despite GUI heartbeat");
        send_planning_=true;const int before=moving_messages_;pump(.2);
        expect(moving_messages_==before,"planner reconnect never restarts output");
        expect(call("reset"),"stationary reset after LPP expiry");
        expect(commission(1,3,other,1,0,.5,0),"manual available again after STOP/reset");
        expect(commission(3,0,other),"manual STOP after autonomous session");
    }
#endif
    void runManualBelowRunningFloor() {
        const string active(32,'f');
        pump(.70);
        expect(commission(1,3,active,1,0.0,.5,0.0),"independent manual session starts at zero motor target");
        const auto first=observed_motor_erpm_.size();
        expect(commission(2,3,active,2,200.0,.5,0.0),"manual session accepts requested 200 ERPM");
        pump(.23);
        bool bounded=observed_motor_erpm_.size()>first;
        for(auto i=first;i<observed_motor_erpm_.size();++i)
            bounded=bounded&&observed_motor_erpm_[i]>=0.0&&observed_motor_erpm_[i]<=200.000001;
        expect(bounded&&abs(observed_motor_erpm_.back()-200.0)<1e-6&&
               abs(status_.value("target_motor_erpm",0.0)-200.0)<1e-6,
               "manual 200 ERPM output preserves its ramp and is never raised to the autonomy floor");
        expect(commission(3,0,active),"manual floor-isolation fixture stops normally");
    }

    void runCommission() {
        const string a(32, '1'), b(32, '2'), c(32, '3'), d(32, '4');
        pump(0.7);
        expect(status_.value("control_units", "") == "raw" && status_.value("commissioning", false), "raw status labels device units");
        expect(status_.at("speed_mps").is_null() && !status_.value("calibration_verified", true), "raw status never invents calibrated speed");
        expect(!call("start_test") && !manual(1, a), "commission rejects both calibrated start paths");
        expect(commission(3, 0, a), "raw STOP with no axis/goals accepted");
        expect(!commission(1, 1, a), "raw delayed START after STOP rejected");
        expect(!commission(1, 1, b, 1, 300, 0.5, 1, 0.5), "raw stale START rejected");
        expect(commission(1, 1, b), "raw motor starts through ROS");
        for (uint64_t seq = 2; seq <= 5; ++seq) {
            pump(0.08);
            expect(commission(2, 1, b, seq), "raw motor heartbeat accepted");
        }
        expect(moving_messages_ > 0 && servo_messages_ == 0, "raw motor trial publishes no servo commands");
        expect(!commission(2, 1, b, 5), "raw duplicate sequence rejected");
        expect(!commission(2, 2, b, 6, 0, 0.51), "raw axis change rejected");
        pump(0.1);
        expect(call("reset"), "raw axis fault reset");
        const int moves = moving_messages_;
        expect(commission(1, 2, c, 1, 0, 0.51), "raw steering starts through ROS");
        pump(0.15);
        expect(commission(2, 2, c, 2, 0, 0.49), "raw steering heartbeat accepted");
        pump(0.1);
        expect(servo_messages_ > 0 && moving_messages_ == moves, "raw steering never drives motor");
        expect(commission(3, 0, c), "raw STOP accepts omitted targets while active");
        expect(call("reset"), "raw stop reset");
        expect(!commission(1, 2, c, 1, 0, 0.50), "closed raw session cannot restart after reset");
        expect(!commission(1, 2, d, 1, 0, 0.52), "raw unsafe servo target refused");
    }

    void runCommissionGuiBridge() {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller", prefix_ + "/nomad_vesc_trial"}});
        auto bridge = make_shared<nomad::GuiBridge>(options);
        executor_.add_node(bridge); pump(0.8);
        auto invoke = [&](const nomad::Json& data) {
            auto future = async(launch::async, [&] { return bridge->command("commissioning", data); });
            while (future.wait_for(chrono::milliseconds(0)) != future_status::ready) pump(0.005);
            return future.get().value("accepted", false);
        };
        const string id(32, '5');
        auto request = nomad::Json{{"action", 1}, {"axis", 2}, {"session", id}, {"sequence", 1},
            {"motor_erpm", 0.0}, {"servo_position", 0.50}, {"duration_s", 1.0}, {"server_time_s", nomad::unixSeconds()}};
        expect(invoke(request), "GUI bridge raw START reaches commissioning service");
        pump(0.15);
        expect(servo_messages_ > 0 && moving_messages_ == 0, "GUI raw steering only drives servo");
        expect(invoke(nomad::Json{{"action", 3}, {"session", id}, {"sequence", 2}}), "GUI raw STOP needs no axis or targets");
        pump(0.3);
        expect(bridge->status().value("state", "") != "RUNNING", "GUI bridge observes raw stop");
        expect(call("reset"), "GUI continuous preparation resets stopped state");
        const string manual_id(32, '8');
        request["axis"] = 3; request["session"] = manual_id; request["duration_s"] = 0.0;
        request["server_time_s"] = nomad::unixSeconds();
        expect(invoke(request), "GUI bridge accepts continuous manual axis and explicit zero duration");
        request["action"] = 2; request["sequence"] = 2; request["motor_erpm"] = 200.0;
        request["servo_position"] = 0.51; request["server_time_s"] = nomad::unixSeconds();
        expect(invoke(request), "GUI bridge forwards both continuous slider targets");
        pump(0.15);
        expect(moving_messages_ > 0 && servo_messages_ > 0, "GUI continuous update drives both fake outputs");
        expect(invoke(nomad::Json{{"action", 3}, {"session", manual_id}, {"sequence", 3}}),
               "GUI continuous STOP accepts targetless payload");
        executor_.remove_node(bridge);
    }

    void runContinuousCommission() {
        const string rejected(32, '9'), active(32, 'a'), next(32, 'b');
        pump(0.7);
        expect(status_.value("continuous_manual_supported", false) &&
               !status_.value("continuous_manual_active", true) &&
               status_.at("commanded_servo_position").is_number(), "raw IDLE advertises continuous contract and candidate command");
        expect(!commission(1, 3, rejected, 1, 1, 0.5, 0), "continuous ROS START rejects nonzero motor");
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "continuous ROS START at zero motor accepted");
        for (uint64_t sequence = 2; sequence <= 44; ++sequence) {
            pump(0.08);
            expect(commission(2, 3, active, sequence, 300, sequence % 2 ? 0.49 : 0.51, 0),
                   "continuous ROS heartbeat with two slider goals accepted");
        }
        expect(status_.value("state", "") == "RUNNING" && status_.value("continuous_manual_active", false) &&
               status_.value("remaining_s", -1.0) == 0.0,
               "continuous ROS control exceeds trial deadline while heartbeat remains fresh");
        expect(moving_messages_ > 0 && servo_messages_ > 0, "continuous ROS emits both command channels");
        expect(!commission(2, 3, active, 44, 0, 0.5, 0), "continuous duplicate sequence rejected");
        expect(!commission(2, 3, active, 45, 0, 0.5, 0, 0.5), "continuous stale UPDATE rejected");
        pump(0.4);
        expect(!commission(2, 3, active, 46, 300, 0.51, 0), "continuous heartbeat timeout cannot be revived");
        const int moves = moving_messages_, servos = servo_messages_;
        pump(0.15);
        expect(moving_messages_ == moves && servo_messages_ == servos &&
               status_.value("state", "") == "FAULT" && !status_.value("continuous_manual_active", true),
               "continuous timeout latches and stops both command channels");
        expect(call("reset"), "continuous fault allows explicit stationary reset");
        expect(!commission(1, 3, active, 47, 0, 0.5, 0), "closed continuous session stays closed after reset");
        expect(commission(3, 0, next), "continuous preemptive STOP accepted");
        expect(!commission(1, 3, next, 1, 0, 0.5, 0), "delayed continuous START cannot pass earlier STOP");
    }

    void runRawMotorOptimization() {
        const string active(32, '1'), blocked(32, '2'), resumed(32, '3');
        pump(0.7);
        expect(status_.value("motor_control_mode", "") == "vesc_speed_pid" &&
               status_.value("speed_pid_min_erpm", -1.0) == 50.0 &&
               !status_.value("speed_pid_min_erpm_verified", true),
               "PID minimum is exposed as unverified metadata until external readback confirmation");
        expect(status_.value("steering_input_limit_rad", 0.0) == 0.5 &&
               status_.value("control_tick_hz", 0) == 50 &&
               status_.value("motor_deceleration_erpm_s", 0.0) == 2400.0,
               "optimized command rates and candidate input range are advertised");
        expect(status_.value("observed_smooth_start_erpm", -1.0) == 0.0,
               "unconfigured operator observation is explicitly unset");
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "optimized raw session starts explicitly");
        expect(commission(2, 3, active, 2, 600, 0.5, 0), "positive raw speed reaches internal VESC PID channel");
        pump(0.15);
        expect(moving_messages_ > 0, "positive command produces speed messages");
        expect(commission(2, 3, active, 3, 0, 0.5, 0), "zero target accepted without ending steering session");
        pump(0.08);
        const int moves = moving_messages_;
        pump(0.12);
        expect(moving_messages_ == moves && status_.value("commanded_motor_erpm", -1.0) == 0.0 &&
               status_.value("target_motor_erpm", -1.0) == 0.0,
               "zero target immediately stops speed messages rather than ramping through low RPM PID");
        expect(commission(3, 0, active), "optimized session STOP accepted");
        expect(call("reset"), "optimized zero session may reset when stationary");
        auto competing = fake_->create_publisher<nomad::Scalar>(prefix_ + "/commands/motor/speed", 1);
        const double deadline = steadySeconds() + 1.0;
        while (controller_->count_publishers(prefix_ + "/commands/motor/speed") < 2 && steadySeconds() < deadline)
            pump(0.005);
        expect(!commission(1, 3, blocked, 1, 0, 0.5, 0), "START refreshes graph and rejects a competing speed publisher");
        competing.reset(); pump(0.25);
        expect(commission(1, 3, resumed, 1, 0, 0.5, 0), "new session starts after competing publisher leaves");
        competing = fake_->create_publisher<nomad::Scalar>(prefix_ + "/commands/motor/speed", 1);
        pump(0.22);
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("competing") != string::npos,
               "cached graph is refreshed within bounded time and faults an active competing publisher");
    }

    // Wi-Fi 유예는 발행 시각 기준이며 로컬 telemetry/timer 감시와 짧은 축 시험은 그대로입니다.
    void runWifiManualProfile() {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller", prefix_ + "/nomad_vesc_trial"}});
        auto bridge = make_shared<nomad::GuiBridge>(options);
        executor_.add_node(bridge); pump(0.8);
        expect(status_.value("wifi_manual_profile", false) && status_.value("manual_timeout_s", 0.0) == 2.0 &&
               status_.value("command_max_age_s", 0.0) == 1.5 && status_.value("browser_freshness_s", 0.0) == 1.5 &&
               status_.value("browser_request_timeout_ms", 0) == 1500,
               "Wi-Fi commissioning advertises bounded command and browser timing policy");
        auto invoke = [&](const nomad::Json& data) {
            auto future = async(launch::async, [&] { return bridge->command("commissioning", data); });
            while (future.wait_for(chrono::milliseconds(0)) != future_status::ready) pump(0.005);
            return future.get().value("accepted", false);
        };
        const string active(32, 'a');
        auto request = nomad::Json{{"action", 1}, {"axis", 3}, {"session", active}, {"sequence", 1},
            {"motor_erpm", 0.0}, {"servo_position", 0.5}, {"duration_s", 0.0}, {"server_time_s", nomad::unixSeconds()}};
        expect(invoke(request), "Wi-Fi MANUAL starts explicitly at zero through GUI bridge");
        request["action"] = 2; request["sequence"] = 2; request["server_time_s"] = nomad::unixSeconds() - 1.6;
        expect(!invoke(request), "Wi-Fi GUI bridge rejects command older than 1.5 seconds");
        expect(!commission(2, 3, active, 2, 0, 0.5, 0, 1.6), "Wi-Fi ROS service independently rejects over-age command");
        pump(1.35);
        request["sequence"] = 3; request["server_time_s"] = nomad::unixSeconds() - 1.2;
        expect(invoke(request), "Wi-Fi GUI and ROS accept 1.2-second-old MANUAL update with fresh local telemetry");
        pump(0.45);
        expect(status_.value("state", "") == "RUNNING", "Wi-Fi issued-command lease tolerates bounded delivery delay");
        pump(0.65);
        expect(status_.value("state", "") == "FAULT" && status_.value("telemetry_fresh", false) &&
               status_.value("reason", "").find("GUI command timeout") != string::npos,
               "delayed update expires from original issue time instead of adding two seconds after receipt");
        expect(!commission(2, 3, active, 4, 0, 0.5, 0), "expired Wi-Fi session cannot be revived by fresh update");
        expect(call("reset") && !commission(1, 3, active, 5, 0, 0.5, 0), "Wi-Fi reset cannot reopen expired session ID");

        for (const int axis : {1, 2}) {
            const string id(32, axis == 1 ? 'b' : 'c');
            request["action"] = 1; request["axis"] = axis; request["session"] = id;
            request["sequence"] = 1; request["duration_s"] = 1.0;
            request["server_time_s"] = nomad::unixSeconds() - 1.2;
            expect(!invoke(request), "Wi-Fi profile does not extend short-axis GUI command age");
            expect(!commission(1, axis, id, 1, 0, 0.5, 1.0, 1.2), "Wi-Fi profile keeps short-axis ROS age strict");
            request["server_time_s"] = nomad::unixSeconds();
            expect(invoke(request), "short-axis trial accepts a fresh command under Wi-Fi profile");
            pump(0.5);
            expect(status_.value("state", "") == "FAULT" &&
                   status_.value("reason", "").find("GUI command timeout") != string::npos,
                   "short-axis trial retains 350ms heartbeat lease in Wi-Fi profile");
            expect(call("reset"), "short-axis Wi-Fi fixture resets only explicitly");
        }
        const string telemetry(32, 'd'), timer(32, 'e');
        expect(commission(1, 3, telemetry, 1, 0, 0.5, 0), "Wi-Fi telemetry guard fixture starts");
        send_telemetry_ = false; pump(0.45);
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("telemetry") != string::npos,
               "Wi-Fi grace does not extend the local 300ms telemetry guard");
        send_telemetry_ = true; pump(0.4);
        expect(call("reset") && commission(1, 3, timer, 1, 0, 0.5, 0), "Wi-Fi timer guard fixture starts explicitly");
        this_thread::sleep_for(chrono::milliseconds(250));
        pump(0.15);
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("control timer stalled") != string::npos,
               "Wi-Fi grace does not extend the local 200ms timer guard");
        expect(moving_messages_ == 0, "Wi-Fi timing integration tests never request positive motor output");
        executor_.remove_node(bridge);
    }

    void runRawStallWatchdog() {
        const string active(32, '4');
        pump(0.7);
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "stall watchdog fixture starts at zero");
        for (uint64_t sequence = 2; sequence < 11; ++sequence) {
            commission(2, 3, active, sequence, 200, 0.5, 0);
            pump(0.1);
        }
        expect(status_.value("state", "") == "FAULT" &&
               status_.value("reason", "").find("stalled") != string::npos,
               "ROS controller detects commanded speed without rotation while heartbeat is fresh");
        const int moves = moving_messages_, servos = servo_messages_;
        pump(0.15);
        expect(moving_messages_ == moves && servo_messages_ == servos && release_messages_ > 0,
               "ROS stall stop emits only motor release and no further servo commands");
    }

    void runExpandedSteeringContract() {
        const string active(32, '5');
        pump(0.7);
        expect(status_.value("steering_input_limit_rad", 0.0) == 0.55 &&
               status_.value("servo_min", 0.0) == 0.3625 && status_.value("servo_max", 0.0) == 0.6375,
               "configured next candidate angle and servo envelope are advertised together");
        expect(status_.value("observed_smooth_start_erpm", 0.0) == 550.0 &&
               status_.value("observed_smooth_start_note", "") == "공중 시험 · 사용자 관찰 · 실제 주행 최저속도 미보정",
               "operator startup observation carries its uncalibrated context");
        expect(commission(1, 3, active, 1, 0, 0.5, 0), "expanded candidate session still starts at zero motor");
        expect(commission(2, 3, active, 2, 200, 0.6375, 0), "target below observed startup threshold is accepted unchanged");
        pump(0.15);
        expect(status_.value("target_motor_erpm", -1.0) == 200.0 &&
               status_.value("commanded_motor_erpm", -1.0) > 0.0 &&
               status_.value("commanded_motor_erpm", -1.0) <= 200.0,
               "observed 550 ERPM metadata never raises the 200 ERPM control target or output");
        uint64_t sequence = 2;
        for (const double target : {0.6375, 0.3625}) {
            bool accepted = true;
            for (int step = 0; step < 16; ++step) {
                accepted = commission(2, 3, active, ++sequence, 0, target, 0) && accepted;
                pump(0.05);
            }
            expect(accepted && abs(status_.value("commanded_servo_position", 0.0) - target) < 1e-9 &&
                   status_.value("commanded_motor_erpm", -1.0) == 0.0,
                   "both configured candidate endpoints are reachable with motor released");
        }
        expect(!commission(2, 3, active, ++sequence, 0, 0.6376, 0),
               "ROS service refuses a target just outside the new candidate envelope");
        pump(0.15);
        expect(status_.value("state", "") == "FAULT" && status_.value("commanded_motor_erpm", -1.0) == 0.0,
               "out-of-range expanded steering request latches fault and motor release");
    }

    void runManual() {
        const string a(32, 'a'), b(32, 'b'), c(32, 'c');
        pump(0.7);
        expect(manual(3, a), "STOP before START accepted");
        expect(!manual(1, a), "delayed START after STOP refused");
        expect(!manual(1, b, 1, 0.5), "stale command refused");
        expect(manual(1, b), "manual ROS start succeeds");
        expect(manual(2, b, 2), "manual ROS update succeeds");
        expect(!manual(2, b, 2), "duplicate sequence refused");
        expect(!manual(2, c, 3), "wrong session refused");
        pump(0.5);
        expect(!manual(2, b, 3), "expired GUI lease cannot revive");
        const int moves = moving_messages_;
        pump(0.1);
        expect(moves == moving_messages_, "GUI timeout stops outputs while telemetry remains fresh");
        expect(call("reset"), "manual fault reset");
        expect(!manual(1, b), "closed session refused after reset");
        expect(manual(1, c), "new session accepted after reset");
        expect(manual(3, c), "manual release stop");
        expect(call("reset"), "manual stop reset");
        expect(!manual(1, c), "stopped session cannot start again");
    }

    void runGuiBridge() {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller", prefix_ + "/nomad_vesc_trial"}});
        auto bridge = make_shared<nomad::GuiBridge>(options);
        executor_.add_node(bridge);
        pump(0.8);
        expect(bridge->status().value("connected", false), "GUI bridge receives controller status");
        const string id(32, 'e');
        auto request = nomad::Json{{"action", 1}, {"session", id}, {"sequence", 1},
            {"speed_mps", 0.1}, {"steering_rad", 0.0}, {"duration_s", 1.0}, {"server_time_s", nomad::unixSeconds()}};
        auto invoke = [&](const string& route, const nomad::Json& data) {
            auto future = async(launch::async, [&] { return bridge->command(route, data); });
            while (future.wait_for(chrono::milliseconds(0)) != future_status::ready) pump(0.005);
            return future.get().value("accepted", false);
        };
        expect(invoke("manual", request), "GUI bridge starts via real ROS service");
        pump(0.15);
        request["action"] = 2; request["sequence"] = 2; request["server_time_s"] = nomad::unixSeconds();
        expect(invoke("manual", request), "GUI bridge forwards heartbeat");
        request["server_time_s"] = nomad::unixSeconds() - 0.5;
        expect(!invoke("manual", request), "GUI bridge rejects delayed HTTP command");
        expect(invoke("stop", nomad::Json::object()), "GUI global stop reaches controller");
        pump(0.3);
        expect(bridge->status().value("state", "") != "RUNNING", "GUI bridge observes stopped state");
        expect(invoke("reset", nomad::Json::object()), "GUI reset reaches controller");
        executor_.remove_node(bridge);
    }

    void runMonitor() {
        pump(1.0);
        expect(fake_->count_publishers(prefix_ + "/commands/motor/speed") == 0, "monitor creates no motor publisher");
        expect(fake_->count_publishers(prefix_ + "/commands/servo/position") == 0, "monitor creates no servo publisher");
        expect(!call("start_test"), "monitor start refused");
        expect(!manual(1, string(32, 'd')), "monitor manual start refused");
        expect(!commission(1, 1, string(32, '6')), "monitor raw start refused");
        expect(call("stop"), "monitor stop service callable");
        pump(0.1);
        expect(speed_messages_ == 0 && servo_messages_ == 0 && release_messages_ == 0, "monitor emits no actuator data");
    }

    void runBench() {
        pump(1.0);
        expect(!commission(1, 1, string(32, '7')), "calibrated bench rejects raw start service");
        expect(speed_messages_ == 0 && servo_messages_ == 0 && release_messages_ == 0, "bench boot never auto-starts");
        expect(call("start_test"), "explicit bench start succeeds");
        expect(!call("start_test"), "duplicate start rejected");
        pump(1.2);
        expect(moving_messages_ > 0 && servo_messages_ > 0, "fake driver receives trial output");
        const int moves_at_stop = moving_messages_;
        const int releases_at_stop = release_messages_;
        pump(0.2);
        expect(moving_messages_ == moves_at_stop && release_messages_ > releases_at_stop, "deadline releases and stays stopped");
        expect(!call("start_test"), "deadline requires explicit reset");
        expect(call("reset") && call("start_test"), "reset then start");
        pump(0.3);
        expect(call("stop"), "stop service succeeds");
        pump(0.1);
        const int stopped_moves = moving_messages_;
        pump(0.2);
        expect(moving_messages_ == stopped_moves, "service stop stays stopped");
        expect(call("reset") && call("start_test"), "start telemetry timeout trial");
        pump(0.2);
        send_telemetry_ = false;
        pump(0.5);
        const int timeout_moves = moving_messages_;
        pump(0.2);
        expect(moving_messages_ == timeout_moves, "telemetry loss stops outputs");
        expect(!call("reset"), "stale telemetry cannot reset");
    }

    void runSteeringRecords() {
        const auto directory=filesystem::temp_directory_path()/prefix_.substr(1);
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"controller",prefix_+"/nomad_vesc_trial"},{"steering_records_directory",directory.string()}});
        auto bridge=make_shared<nomad::GuiBridge>(options);executor_.add_node(bridge);pump(.8);
        auto setRange=[&](double span){
            const nomad::Json data={{"span",span},{"server_time_s",nomad::unixSeconds()}};
            auto future=async(launch::async,[&]{return bridge->command("steering-range",data);});
            while(future.wait_for(chrono::milliseconds(0))!=future_status::ready)pump(.005);
            return future.get().value("accepted",false);
        };
        const auto servo_before=servo_messages_;
        expect(setRange(.3),"GUI range service accepts stationary steering-only limit");pump(.15);
        expect(servo_messages_==servo_before&&moving_messages_==0,"range change publishes neither steering nor motor movement");
        expect(bridge->status().at("steering_test_servo_max")==.8&&bridge->status().at("servo_max")!=.8,"test range is separate from drive range");
        expect(!setRange(.6),"GUI range request above protocol bounds rejected");
        nomad::Json row={{"operation","save"},{"id",string(32,'a')},{"servo_command",.5},
            {"left_deg",0.0},{"right_deg",0.0},{"note","integration test only"},{"measured_confirmed",true}};
        expect(bridge->command("steering-records",row).value("accepted",false),"stationary measurement stored through GUI bridge");
        auto invoke=[&](const nomad::Json& data){
            auto future=async(launch::async,[&]{return bridge->command("commissioning",data);});
            while(future.wait_for(chrono::milliseconds(0))!=future_status::ready)pump(.005);
            return future.get().value("accepted",false);
        };
        nomad::Json step={{"action",1},{"axis",2},{"session",string(32,'b')},{"sequence",1},
            {"motor_erpm",0},{"servo_position",.505},{"duration_s",bridge->status().at("max_duration_s")},{"server_time_s",nomad::unixSeconds()},
            {"record_steering_command",true}};
        expect(invoke(step),"calibration uses steering-only trial and automatic recording");pump(.12);
        expect(!setRange(.4),"range service rejects changes during steering motion");
        step["action"]=2;step["sequence"]=2;step["server_time_s"]=nomad::unixSeconds();
        expect(invoke(step),"recorded steering heartbeat accepted");
        auto automatic=bridge->command("steering-records",{{"operation","list"}});
        expect(automatic.at("records").size()==2,"unchanged heartbeat does not duplicate command record");
        for(const auto& record:automatic.at("records"))if(record.value("kind",string{})=="command")
            expect(record.at("left_deg").is_null()&&!record.at("measured_confirmed").get<bool>()&&record.at("servo_command")==.505,
                   "automatic record contains accepted servo target but no fabricated angle");
        expect(!bridge->command("steering-records",row).value("accepted",true),"cannot save while steering trial running");
        expect(call("stop"),"calibration explicit stop");pump(.25);
        row["id"]=string(32,'c');row["servo_command"]=.6;
        expect(!bridge->command("steering-records",row).value("accepted",true),"stale servo pairing refused");
        row["servo_command"]=bridge->status().at("commanded_servo_position");row["left_deg"]=1.2;row["right_deg"]=1.1;
        expect(bridge->command("steering-records",row).value("accepted",false),"settled command pairs with user measurement");
        auto note=row;note["id"]=string(32,'e');note["kind"]="observation";
        note.erase("left_deg");note.erase("right_deg");note.erase("measured_confirmed");
        auto noted=bridge->command("steering-records",note);
        expect(noted.value("accepted",false)&&noted.at("record").at("left_rad").is_null(),"memo needs no measured angle");
        auto listed=bridge->command("steering-records",{{"operation","list"}});
        expect(listed.at("records").size()==4&&!listed.value("applied",true),"command, observation and measurements remain distinct without applying calibration");
        send_telemetry_=false;pump(.4);row["id"]=string(32,'d');
        expect(!bridge->command("steering-records",row).value("accepted",true),"stale telemetry cannot save measurement");
        expect(moving_messages_==0,"calibration emitted no nonzero motor speed command");
        executor_.remove_node(bridge);
        if(directory.parent_path()==filesystem::temp_directory_path())filesystem::remove_all(directory);
    }

    void runStability() {
        pump(.8);
        expect(!call("start_test"),"stability rejects missing IMU before drive");
        expect(!manual(1,"77777777777777777777777777777777"),"GUI start also rejects missing IMU");
        expect(manual(3,"77777777777777777777777777777777",2),"GUI STOP remains available without IMU");
        expect(moving_messages_==0,"missing IMU produces no drive");
        imu_pub_=fake_->create_publisher<sensor_msgs::msg::Imu>(prefix_+"/imu_body",1);
        send_imu_=true; imu_frame_="wrong_frame"; pump(.3);
        expect(!call("start_test"),"wrong IMU frame rejected");
        imu_frame_="base_link";pump(.3);
        expect(call("start_test"),"qualified simulated IMU starts bench");
        pump(.2); send_imu_=false; pump(.3);
        expect(status_.value("state","")=="STOPPED_OUTPUT","stale IMU stops owned bench output");
        const auto moves=moving_messages_; send_imu_=true; pump(.3);
        expect(moving_messages_==moves,"IMU recovery cannot restart motion");
        expect(call("reset"),"explicit reset with stationary telemetry");
        expect(call("start_test"),"explicit restart after IMU recovery");
        imu_roll_=.35; pump(.15);
        expect(status_.value("state","")=="STOPPED_OUTPUT","excessive tilt stops bench output");
        expect(status_.value("ground_drive_supported",true)==false,"never claims ground support");
    }

private:
#ifdef NOMAD_ENABLE_PLANNING
    rclcpp::Publisher<nomad_interfaces::msg::DriveCommand>::SharedPtr planning_pub_;
    bool send_planning_=true;
    double planner_speed_=.2;
#endif
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    bool send_imu_=false;
    string imu_frame_="base_link";
    double imu_roll_=0;
    string prefix_;
    rclcpp::Node::SharedPtr fake_;
    shared_ptr<nomad::ControllerNode> controller_;
    rclcpp::Publisher<nomad::VescState>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    nomad::Json status_ = nomad::Json::object();
    vector<rclcpp::Subscription<nomad::Scalar>::SharedPtr> subs_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    bool send_telemetry_ = true;
    double last_sent_ = 0.0;
    double telemetry_erpm_ = 0.0;
    int telemetry_fault_ = 0;
    int speed_messages_ = 0, moving_messages_ = 0, servo_messages_ = 0, release_messages_ = 0;
    vector<double> observed_motor_erpm_;
};

#ifdef NOMAD_ENABLE_PLANNING
void testMinimumRunningFloor() {
    const vector<rclcpp::Parameter> parameters={
        {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
        {"planning.measured_speed_control",true},{"planning.minimum_running_erpm",700.0},
        {"planning.start_hold_s",.75},{"planning.restart_cooldown_s",1.0},
        {"planning.minimum_start_erpm",550.0},{"planning.feedback_limit_erpm",1200.0},
        {"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
        {"commissioning.overspeed_erpm",2500.0},{"commissioning.stationary_hold_s",.5}};
    {RosRig floor(true,true,true,true,false,parameters);floor.runMinimumRunningFloor();}
    {RosRig manual(true,true);manual.runManualBelowRunningFloor();}
}

void testMeasuredSpeedControl() {
    const vector<rclcpp::Parameter> parameters={
        {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
        {"planning.measured_speed_control",true},{"planning.start_hold_s",.75},{"planning.restart_cooldown_s",1.0},
        {"planning.minimum_start_erpm",550.0},{"planning.feedback_limit_erpm",1200.0},
        {"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3},
        {"commissioning.overspeed_erpm",2500.0},{"commissioning.stationary_hold_s",.5}};
    auto feedback_parameters=parameters;
    feedback_parameters.emplace_back("planning.minimum_running_erpm",0.0);
    {RosRig feedback(true,true,true,true,false,feedback_parameters);feedback.runMeasuredSpeedFeedback();}
    for(const string failure:{"operator STOP","invalid command","command expiry","telemetry timeout","VESC fault","hard overspeed"}) {
        RosRig guarded(true,true,true,true,false,parameters);guarded.runMeasuredSpeedSafety(failure);
    }
}
#endif

void testRawMetadataConfiguration() {
    const double nan = numeric_limits<double>::quiet_NaN();
    const double infinity = numeric_limits<double>::infinity();
    const vector<rclcpp::Parameter> invalid = {
        {"steering_input_limit_rad", 0.0}, {"steering_input_limit_rad", -0.01},
        {"steering_input_limit_rad", 0.5501}, {"steering_input_limit_rad", nan},
        {"steering_input_limit_rad", infinity}, {"observed_smooth_start_erpm", -1.0},
        {"observed_smooth_start_erpm", nan}, {"observed_smooth_start_erpm", infinity}};
    for (const auto& parameter : invalid) {
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"mode", "commission"}, parameter});
        bool rejected = false;
        try { auto node = nomad::ControllerNode::create(options); }
        catch (const invalid_argument&) { rejected = true; }
        expect(rejected, "invalid raw angle or observation metadata is rejected before output creation");
    }
}
} // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
#ifdef NOMAD_ENABLE_PLANNING
        if(argc>1&&string(argv[1])=="--running-floor-only"){
            testMinimumRunningFloor();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--measured-speed-only"){
            testMeasuredSpeedControl();
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
        if(argc>1&&string(argv[1])=="--headless-resume-only"){
            {
            RosRig resume(true,true,true,true,false,{
                {"planning.headless",true},{"planning.allow_estimated_calibration",true},
                {"planning.resume_after_stop",true},{"planning.erpm_per_mps",1000.0},
                {"planning.servo_per_rad",.5},{"planning.max_mps",.25},{"planning.max_rad",.3}});
            resume.runHeadlessResume();
            }
            cout<<checks<<" checks, "<<failures<<" failures\n";
            rclcpp::shutdown();return failures?1:0;
        }
#endif
        testRawMetadataConfiguration();
        for (const auto& params : vector<vector<rclcpp::Parameter>>{
                {{"operation_environment","ground"}},{{"mode","commission"},{"stability.enabled",true}}}) {
            rclcpp::NodeOptions options;options.parameter_overrides(params);
            bool refused=false;
            try { auto node=nomad::ControllerNode::create(options); }
            catch(const invalid_argument&) { refused=true; }
            expect(refused,"unvalidated ground/raw stability modes refused");
        }
#ifdef NOMAD_ENABLE_PLANNING
        for (const string mode : {"bench","monitor"}) {
            rclcpp::NodeOptions options;
            options.parameter_overrides({{"mode",mode},{"planning.headless",true},
                {"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true}});
            bool refused=false;
            try { auto node=nomad::ControllerNode::create(options); }
            catch(const invalid_argument& error) {
                refused=string(error.what()).find("planning.headless")!=string::npos;
            }
            expect(refused,"headless planning outside commission mode refused at startup");
        }
#endif
        { RosRig stability(true,false,true,true,false,{
            {"stability.enabled",true},{"stability.geometry_verified",true},{"stability.frame_verified",true},
            {"stability.track_m",.25},{"stability.wheelbase_m",.335},{"stability.cg_height_m",.15},
            {"stability.cg_from_rear_m",.17}});stability.runStability(); }
        { RosRig monitor(false); monitor.runMonitor(); }
#ifdef NOMAD_ENABLE_PLANNING
        testMeasuredSpeedControl();
        { RosRig stable(true,true,true,true,false,{
            {"planning.headless",true},{"planning.allow_estimated_calibration",true},{"planning.resume_after_stop",true},
            {"planning.start_hold_s",.2},{"planning.restart_cooldown_s",.3},{"planning.minimum_start_erpm",550.0},
            {"planning.feedback_limit_erpm",1200.0},{"planning.erpm_per_mps",4000.0},{"planning.servo_per_rad",.5},
            {"planning.max_mps",.25},{"planning.max_rad",.3}});stable.runStableHeadlessStart(); }
        { RosRig headless(true,true,true,true,false,{
            {"planning.headless",true},{"planning.allow_estimated_calibration",true},
            {"planning.erpm_per_mps",1000.0},{"planning.servo_per_rad",.5},
            {"planning.max_mps",.25},{"planning.max_rad",.3}});headless.runHeadless(); }
        { RosRig planning(true,true,true,true,false,{
            {"planning.calibration_verified",true},{"planning.erpm_per_mps",1000.0},
            {"planning.feedback_erpm_sign",1},{"planning.servo_per_rad",-.5},
            {"planning.max_mps",.6},{"planning.max_rad",.1}});planning.runPlanning(); }
        { RosRig table(true,true,true,true,false,{
            {"planning.calibration_verified",true},{"planning.erpm_per_mps",1000.0},
            {"planning.feedback_erpm_sign",1},{"planning.max_mps",.6},{"planning.max_rad",.1},
            {"planning.steering_rads",vector<double>{-.1,0.0,.1}},
            {"planning.steering_servos",vector<double>{.55,.5,.45}}});table.runPlanning(); }
#endif
        { RosRig bench(true); bench.runBench(); }
        { RosRig manual(true); manual.runManual(); }
        { RosRig gui(true); gui.runGuiBridge(); }
        { RosRig raw(true, true); raw.runCommission(); }
        { RosRig raw_gui(true, true); raw_gui.runCommissionGuiBridge(); }
        { RosRig calibration(true,true);calibration.runSteeringRecords(); }
        { RosRig continuous(true, true); continuous.runContinuousCommission(); }
        { RosRig wifi(true, true, true, true, false, {{"commissioning.manual_timeout_s", 2.0}});
          wifi.runWifiManualProfile(); }
        { RosRig optimized(true, true); optimized.runRawMotorOptimization(); }
        { RosRig expanded(true, true, true, true, false, {
              {"steering_input_limit_rad", 0.55}, {"observed_smooth_start_erpm", 550.0},
              {"commissioning.servo_min", 0.3625}, {"commissioning.servo_max", 0.6375}});
          expanded.runExpandedSteeringContract(); }
        { RosRig stalled(true, true, true, true, true); stalled.runRawStallWatchdog(); }
        { RosRig no_setup(true, true, false, true); no_setup.runCommissionGate(); }
        { RosRig no_timeout(true, true, true, false); no_timeout.runCommissionGate(); }
    } catch (const exception& error) {
        ++failures;
        cerr << "ROS integration test error: " << error.what() << '\n';
    }
    rclcpp::shutdown();
    cout << checks << " ROS checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}

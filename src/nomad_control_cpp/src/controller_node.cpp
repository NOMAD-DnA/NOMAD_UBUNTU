#include "nomad_control_cpp/controller_node.hpp"
#include "nomad_control_cpp/trial_controller.hpp"
#include "nomad_control_cpp/raw_trial.hpp"
#include "nomad_control_cpp/stability_guard.hpp"
#include "nomad_control_cpp/planning_command.hpp"
#include "nomad_control_cpp/planning_start_gate.hpp"
#include "nomad_control_cpp/planning_speed_feedback.hpp"
#include "nomad_control_cpp/control_limits.hpp"
#ifdef NOMAD_ENABLE_PLANNING
#include <nomad_interfaces/msg/drive_command.hpp>
#include <nomad_interfaces/msg/vehicle_state.hpp>
#endif
#include <sensor_msgs/msg/imu.hpp>
#include "calibration.hpp"
#include "nomad_control_cpp/srv/manual_control.hpp"
#include "nomad_control_cpp/srv/commissioning_control.hpp"
#include "nomad_control_cpp/srv/set_steering_range.hpp"
#include "json.hpp"
#include <cctype>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std;
// Exception: inside rclcpp::Node subclasses, Node::make_shared/make_unique (static members added by
// RCLCPP_SMART_PTR_DEFINITIONS) hide the free functions, so those calls stay std::-qualified.

namespace nomad {
namespace {
// 하나의 출력 소유자가 수동/LPP 입력을 중재합니다. monitor는 장치 publisher를 만들지 않습니다.
class ControllerNodeImpl final : public ControllerNode {
public:
    explicit ControllerNodeImpl(const rclcpp::NodeOptions& options)
        : ControllerNode(options) {
        bool sim_time = false;
        get_parameter("use_sim_time", sim_time);
        if (sim_time) throw invalid_argument("real VESC trial requires use_sim_time=false");
        time_guard_ = add_on_set_parameters_callback([](const vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& p : params) {
                if (p.get_name() == "use_sim_time" && p.as_bool()) {
                    result.successful = false;
                    result.reason = "VESC trial cannot switch to simulation time";
                }
            }
            return result;
        });

        const string mode = parameter<string>("mode", "monitor");
        if (mode != "monitor" && mode != "bench" && mode != "commission")
            throw invalid_argument("mode must be monitor, bench or commission");
        commissioning_ = mode == "commission";
        options_.actuation_enabled = mode != "monitor";
        options_.calibration_verified = parameter<bool>("calibration_verified", false);
        options_.hardware_timeout_verified = parameter<bool>("hardware_timeout_verified", false);
        options_.target_speed_mps = parameter<double>("target_speed_mps", 0.0);
        options_.target_steering_rad = parameter<double>("target_steering_rad", 0.0);
        options_.duration_s = parameter<double>("trial_duration_s", 1.0);
        options_.telemetry_timeout_s = parameter<double>("telemetry_timeout_s", 0.30);
        options_.max_tick_gap_s = parameter<double>("max_tick_gap_s", 0.20);
        options_.stationary_hold_s = parameter<double>("stationary_hold_s", 0.50);
        options_.stationary_erpm = parameter<double>("stationary_erpm", 50.0);
        options_.stationary_speed_mps = parameter<double>("stationary_speed_mps", 0.03);
        options_.current_limit_a = parameter<double>("current_limit_a", 30.0);
        options_.acceleration_mps2 = parameter<double>("acceleration_mps2", 0.20);
        options_.steering_rate_radps = parameter<double>("steering_rate_radps", 0.30);
        options_.steering_settle_s = parameter<double>("steering_settle_s", 0.30);
        options_.manual_timeout_s = parameter<double>("manual_timeout_s", 0.35);
        // 현재 정지는 0A coast이므로 이 노드는 지상/경사 주행 모드를 지원하지 않습니다.
        const auto environment = parameter<string>("operation_environment", "raised_bench");
        if (environment != "raised_bench")
            throw invalid_argument("ground driving unavailable: downhill braking/hold and stability not validated");
        loadStability();
        if (commissioning_) {
            if (options_.calibration_verified)
                throw invalid_argument("raw commissioning must keep calibration_verified=false");
            raw_options_ = loadRawOptions();
            raw_controller_ = std::make_unique<RawTrialController>(raw_options_);
        } else {
            calibration_ = loadCalibration();
            controller_ = std::make_unique<TrialController>(calibration_, options_);
        }

        const auto telemetry_topic = parameter<string>("telemetry_topic", "/sensors/core");
        const auto speed_topic = parameter<string>("motor_speed_topic", "/commands/motor/speed");
        const auto current_topic = parameter<string>("motor_current_topic", "/commands/motor/current");
        const auto servo_topic = parameter<string>("servo_topic", "/commands/servo/position");
        const vector<string> other_topics = {
            parameter<string>("motor_duty_topic", "/commands/motor/duty_cycle"),
            parameter<string>("motor_brake_topic", "/commands/motor/brake"),
            parameter<string>("motor_position_topic", "/commands/motor/position")};
        set<string> distinct;
        for (const auto& topic : {telemetry_topic, speed_topic, current_topic, servo_topic})
            if (!distinct.insert(get_node_topics_interface()->resolve_topic_name(topic)).second)
                throw invalid_argument("topics must be distinct");
        for (const auto& topic : other_topics) {
            const auto resolved = get_node_topics_interface()->resolve_topic_name(topic);
            if (!distinct.insert(resolved).second) throw invalid_argument("topics must be distinct");
            other_command_topics_.push_back(resolved);
        }

        if (options_.actuation_enabled) {
            const auto command_qos = rclcpp::QoS(1).reliable().durability_volatile();
            speed_pub_ = create_publisher<Scalar>(speed_topic, command_qos);
            current_pub_ = create_publisher<Scalar>(current_topic, command_qos);
            servo_pub_ = create_publisher<Scalar>(servo_topic, command_qos);
        }
        telemetry_sub_ = create_subscription<VescState>(telemetry_topic,
            rclcpp::QoS(1).best_effort().durability_volatile(),
            [this](VescState::ConstSharedPtr msg) { onTelemetry(*msg); });
        status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 1);

        status_json_pub_ = create_publisher<std_msgs::msg::String>("~/status_json", 1);
        manual_service_ = create_service<Manual>("~/manual_control",
            [this](const shared_ptr<Manual::Request> request, shared_ptr<Manual::Response> response) {
                onManual(*request, *response);
            });
        commissioning_service_ = create_service<Commissioning>("~/commissioning_control",
            [this](const shared_ptr<Commissioning::Request> request,
                   shared_ptr<Commissioning::Response> response) {
                onCommissioning(*request, *response);
            });
        steering_range_service_=create_service<SteeringRange>("~/steering_test_range",
            [this](const shared_ptr<SteeringRange::Request> request,shared_ptr<SteeringRange::Response> response){
                const double sent=rclcpp::Time(request->stamp).seconds(),age=now().seconds()-sent;
                response->accepted=commissioning_&&isfinite(age)&&age>=-limits::kFutureStampToleranceS&&age<=limits::kWifiManualCommandMaxAgeS&&
                    raw_controller_->setSteeringTestSpan(request->span,steadySeconds(),now().seconds());
                response->reason=response->accepted?"steering-only range applied; no actuator command sent":
                    "range refused: stop first, require fresh stationary telemetry, valid span and current command inside new range";
                if(commissioning_){response->servo_min=raw_controller_->steeringTestMin();response->servo_max=raw_controller_->steeringTestMax();}
            });
        start_service_ = create_service<Trigger>("~/start_test",
            [this](const shared_ptr<Trigger::Request>, shared_ptr<Trigger::Response> response) {
                if (commissioning_) {
                    response->success = false;
                    response->message = "commission mode requires explicit raw commissioning_control session";
                    return;
                }
                if (!stabilityReady()) { response->success=false; response->message=stability_decision_.reason; return; }
                response->success = controller_->start(steadySeconds(), now().seconds(), exclusiveDriverReady());
                response->message = controller_->reason();
            });
        stop_service_ = create_service<Trigger>("~/stop",
            [this](const shared_ptr<Trigger::Request>, shared_ptr<Trigger::Response> response) {
                stopAndRelease("operator stop");
                response->success = true;
                response->message = options_.actuation_enabled
                    ? "Trial stopped; owned output released with 0A, physical stop must be observed"
                    : "Monitor mode: no actuator output was created or sent";
            });
        reset_service_ = create_service<Trigger>("~/reset",
            [this](const shared_ptr<Trigger::Request>, shared_ptr<Trigger::Response> response) {
                response->success = activeReset(steadySeconds(), now().seconds());
                response->message = response->success ? activeReason()
                    : "Reset refused: need non-running state and fresh stationary telemetry";
            });
#ifdef NOMAD_ENABLE_PLANNING
        setupPlanning();
#endif
        timer_ = create_wall_timer(chrono::milliseconds(20), [this]() { controlTick(); });
        if (commissioning_)
            RCLCPP_INFO(get_logger(), "mode=commission; raw units; trial_max_duration=%.2fs; continuous MANUAL supported; servo_center=%.3f is an unmeasured candidate",
                        raw_options_.max_duration_s, raw_options_.servo_center);
        else
            RCLCPP_INFO(get_logger(), "mode=%s; start is manual; duration=%.2fs (hard maximum 5s)",
                        mode.c_str(), options_.duration_s);
        RCLCPP_WARN(get_logger(), "0A release is coast, not active braking. SIGKILL/power loss require verified VESC timeout.");
    }

    // 기본 ROS signal shutdown보다 먼저 호출해 정상 Ctrl+C/SIGTERM 시 0A 전송 기회를 확보합니다.
    void prepareShutdown() noexcept override {
        if (!controller_ && !raw_controller_) return;
        activeStop("node shutting down");
        for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
            try { releaseMotor(); } catch (...) { break; }
            if (activeOwnsOutput()) this_thread::sleep_for(chrono::milliseconds(20));
        }
    }

private:
#ifdef NOMAD_ENABLE_PLANNING
    // 팀의 nomad_interfaces를 사용합니다. 별도의 동명 메시지를 임의 생성하지 않습니다.
    void setupPlanning() {
        planning_headless_=parameter<bool>("planning.headless",false);
        // Headless planning drives the raw commission controller only; other modes have no raw controller.
        if(planning_headless_&&!commissioning_)
            throw invalid_argument("planning.headless requires mode=commission");
        planning_resume_after_stop_=parameter<bool>("planning.resume_after_stop",false);
        planning_start_hold_s_=parameter<double>("planning.start_hold_s",0.75);
        planning_restart_cooldown_s_=parameter<double>("planning.restart_cooldown_s",1.0);
        planning_min_start_erpm_=parameter<double>("planning.minimum_start_erpm",0.0);
        planning_feedback_limit_erpm_=parameter<double>("planning.feedback_limit_erpm",0.0);
        planning_measured_speed_control_=parameter<bool>("planning.measured_speed_control",false);
        planning_min_running_erpm_=parameter<double>("planning.minimum_running_erpm",
            planning_headless_&&planning_measured_speed_control_?700.0:0.0);
        if(!isfinite(planning_min_running_erpm_)||planning_min_running_erpm_<0||
           planning_min_running_erpm_>raw_options_.motor_max_erpm||
           (planning_min_running_erpm_>0&&(!planning_headless_||!planning_measured_speed_control_||
             planning_min_running_erpm_>planning_feedback_limit_erpm_)))
            throw invalid_argument("planning running floor requires measured headless mode and bounded ERPM");
        if(!isfinite(planning_feedback_limit_erpm_)||planning_feedback_limit_erpm_<0||planning_feedback_limit_erpm_>2500)
            throw invalid_argument("invalid autonomous feedback ERPM limit");
        planning_start_gate_=std::make_unique<PlanningStartGate>(planning_start_hold_s_,planning_restart_cooldown_s_,planning_min_start_erpm_);
        if(planning_measured_speed_control_){
            if(!planning_headless_||planning_feedback_limit_erpm_<=0)
                throw invalid_argument("measured speed control requires headless planning and a positive feedback limit");
            planning_speed_feedback_=std::make_unique<PlanningSpeedFeedback>(
                planning_feedback_limit_erpm_,raw_options_.stationary_erpm,1.0,raw_options_.motor_ramp_erpm_s);
        }
        if(planning_resume_after_stop_&&!planning_headless_)
            throw invalid_argument("planning pause resume requires headless mode");
        if(raw_controller_)raw_controller_->setPlanningRecovery(planning_headless_ && planning_resume_after_stop_);
        planning_allow_estimated_=parameter<bool>("planning.allow_estimated_calibration",false);
        if(planning_allow_estimated_&&!planning_headless_)
            throw invalid_argument("estimated calibration opt-in requires headless planning mode");
        planning_calibration_.verified=parameter<bool>("planning.calibration_verified",false);
        planning_calibration_.erpm_per_mps=parameter<double>("planning.erpm_per_mps",0.0);
        planning_calibration_.servo_per_rad=parameter<double>("planning.servo_per_rad",0.0);
        planning_calibration_.servo_zero=parameter<double>("planning.servo_zero",0.5);
        planning_calibration_.max_mps=parameter<double>("planning.max_mps",0.0);
        planning_calibration_.max_rad=parameter<double>("planning.max_rad",0.0);
        planning_calibration_.steering_rads=parameter<vector<double>>("planning.steering_rads",vector<double>{});
        planning_calibration_.steering_servos=parameter<vector<double>>("planning.steering_servos",vector<double>{});
        planning_feedback_sign_=parameter<int>("planning.feedback_erpm_sign",0);
        if(planning_calibration_.verified&&planning_feedback_sign_!=1&&planning_feedback_sign_!=-1)
            throw invalid_argument("planning.feedback_erpm_sign must be measured as +1 or -1");
        auto execution_calibration=planning_calibration_;
        // 초기값 사용 동의는 출력 변환에만 적용합니다. 실측 피드백 유효 상태는 바꾸지 않습니다.
        if(planning_allow_estimated_)execution_calibration.verified=true;
        planning_guard_=std::make_unique<PlanningCommandGuard>(execution_calibration,raw_options_.motor_max_erpm);
        planning_sub_=create_subscription<nomad_interfaces::msg::DriveCommand>(
            "/nomad/planning/drive_command",rclcpp::QoS(1).reliable().durability_volatile(),
            [this](nomad_interfaces::msg::DriveCommand::ConstSharedPtr msg){
                PlanningCommand c;
                c.stamp=static_cast<double>(msg->header.stamp.sec)+msg->header.stamp.nanosec*1e-9;
                c.valid_for=static_cast<double>(msg->valid_for.sec)+msg->valid_for.nanosec*1e-9;
                c.frame=msg->header.frame_id;c.speed_mps=msg->target_speed;
                c.steering_rad=msg->target_steering_angle;c.stop=msg->stop_requested;
                if(msg->header.stamp.nanosec>=1000000000u||msg->valid_for.nanosec>=1000000000u)c.frame="invalid";
                const auto ros=now().seconds(),steady=steadySeconds();
                // STOP도 먼저 guard에 전달해 이전 이동 명령의 stamp 재사용을 막습니다.
                const bool received=planning_guard_->receive(c,ros,steady);
                if(planning_headless_&&c.stop){
                    // An out-of-order STOP rejected by the stamp fence cannot erase a newer live command.
                    if(planning_guard_->target(ros,steady))return;
                    onPlanningLost("LPP pause; waiting for fresh command","LPP STOP; reset required",steady,ros);
                    return;
                }
                if(received){
                    planning_speed_=c.speed_mps;planning_angle_=c.steering_rad;
                    if(auto target=planning_guard_->target(ros,steady))
                        planning_start_gate_->update(steady,target->erpm,min(limits::kPlanningReceiveLeaseS,c.valid_for-max(0.0,ros-c.stamp)));
                    else planning_start_gate_->reset(steady,true);
                    if(planning_soft_stopped_)planning_resume_pending_=true;
                }
                if(planning_headless_&&planning_resume_after_stop_&&!planning_guard_->target(ros,steady)){
                    pausePlanning("planning input unavailable; waiting for fresh command",steady,ros);return;
                }
                // 수동 제어 중 LPP STOP은 수동 세션을 빼앗지 않습니다. 전역 STOP은 모든 모드를 해제합니다.
                if(autonomous_active_&&!planning_guard_->target(now().seconds(),steadySeconds())) {
                    stopAndRelease("planning stopped or invalid");
                }
            });
        vehicle_state_pub_=create_publisher<nomad_interfaces::msg::VehicleState>(
            "/nomad/control/vehicle_state",rclcpp::QoS(1).reliable().durability_volatile());
    }
    // LPP input went away (STOP, expiry, invalid): headless auto-resume profiles pause and wait for a
    // fresh command; every other profile latches a stop that needs RESET.
    void onPlanningLost(const string& pause_reason,const string& stop_reason,double steady,double ros) {
        if(planning_headless_&&planning_resume_after_stop_)pausePlanning(pause_reason,steady,ros);
        else stopAndRelease(stop_reason);
    }
    bool planningReady(double steady,double ros) const {
        return commissioning_&&planning_guard_&&planning_sub_&&
            count_publishers(planning_sub_->get_topic_name())==1&&planning_guard_->target(ros,steady).has_value();
    }
    void onAutonomous(const Commissioning::Request& request,Commissioning::Response& response) {
        if(planning_headless_&&request.action!=Commissioning::Request::STOP){
            response.accepted=false;response.reason="headless planner owns output";return;
        }
        // GUI는 목표를 쓰지 않고 승인 세션만 유지합니다. 목표는 매 tick의 최신 LPP 명령입니다.
        handleSession(request,response,[&]{
            const auto steady=steadySeconds(),ros=now().seconds();
            if(!planningReady(steady,ros))return false;
            const bool ok=raw_controller_->start(RawAxis::Manual,0,raw_controller_->commandedServoPosition(),
                0,steady,ros,exclusiveDriverReady());
            if(ok){autonomous_active_=true;autonomous_heartbeat_=steady-max(0.0,ros-rclcpp::Time(request.stamp).seconds());commissioning_axis_=4;}
            return ok;
        },[&]{
            const double steady=steadySeconds();
            if(!autonomous_active_)return false;
            if(steady-autonomous_heartbeat_>limits::kOperatorHeartbeatS){activeStop("operator heartbeat expired");return false;}
            autonomous_heartbeat_=steady-max(0.0,now().seconds()-rclcpp::Time(request.stamp).seconds());return true;
        },limits::kWifiManualCommandMaxAgeS);
        if(!response.accepted&&!autonomous_active_)response.reason="autonomy requires one fresh planner and verified m/s/rad calibration; stop manual control first";
    }
    void resetPlanningSpeedFeedback() {
        if(planning_speed_feedback_)planning_speed_feedback_->reset();
        planning_feedback_tick_.reset();
        planning_feedback_target_=0;
        planning_running_target_=0;
        planning_feedback_reducing_=false;
    }
    double planningExecutionTarget(double requested,double dt) {
        // Positive autonomous requests use the configured running floor; zero releases.
        // Existing measured-overspeed reduction may temporarily lower this demand.
        planning_running_target_=requested>0?max(requested,planning_min_running_erpm_):requested;
        return planning_speed_feedback_->update(planning_running_target_,last_raw_erpm_,dt);
    }
    string planningStartReason(double steady) const {
        if(!planning_measured_speed_control_)return planning_start_gate_->reason(steady);
        if(headless_blocked_)return activeReason();
        if(planning_soft_stopped_)return planning_wait_reason_;
        if(planning_feedback_reducing_)return "measured overspeed; reducing speed command";
        return autonomous_active_?"measured speed tracking":"waiting for fresh command and measured speed";
    }
    void logPlanningWait(const string& reason,double steady,double ros) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
            "[PLANNING_WAIT] reason=%s state=%s telemetry=%s receive_age=%.3f source_age=%.3f "
            "telemetry_publishers=%zu speed_subscribers=%zu current_subscribers=%zu servo_subscribers=%zu "
            "command_publishers=%zu,%zu,%zu other_publishers=%zu fault_code=%d motor_current=%.2f voltage=%.2f reset_required=%d",
            reason.c_str(),trialStateName(activeState()),raw_controller_->telemetryIssue().c_str(),
            raw_controller_->telemetryReceiveAge(steady),raw_controller_->telemetryStampAge(ros),
            graph_telemetry_publishers_,graph_speed_subscribers_,graph_current_subscribers_,graph_servo_subscribers_,
            graph_speed_publishers_,graph_current_publishers_,graph_servo_publishers_,graph_other_publishers_,
            static_cast<int>(last_fault_),last_current_,last_voltage_,static_cast<int>(headless_blocked_));
    }
    // A missing module/endpoint/telemetry sample releases output and waits. Actual
    // physical faults, clock/timer failures and explicit operator STOP stay latched.
    void pausePlanning(const string& reason,double steady,double ros) {
        if(headless_blocked_ || activeState()==TrialState::Fault){
            if(!headless_blocked_)activeStop(activeReason());
            autonomous_active_=false;planning_soft_stopped_=false;planning_resume_pending_=false;
            releaseMotor();return;
        }
        if(!raw_controller_->pausePlanning(reason,steady,ros)){
            const auto fault=activeReason();stopAndRelease(fault);
            logPlanningWait(fault,steady,ros);return;
        }
        autonomous_active_=false;planning_resume_pending_=false;planning_wait_reason_=reason;
        planning_guard_->clear();planning_start_gate_->reset(steady,true);
        resetPlanningSpeedFeedback();planning_overspeed_since_.reset();
        if(activeState()==TrialState::Stopped){planning_soft_stopped_=true;logRawTransition();}
        closeManualSession();releaseMotor();
        logPlanningWait(reason,steady,ros);
    }
    void tickPlanning(double steady,double ros) {
        if(planning_headless_ && planning_resume_after_stop_ && !headless_blocked_){
            const bool driver_ready=exclusiveDriverReady(false);
            if(activeState()==TrialState::Fault || graph_competing_publishers_){
                const auto fault=activeState()==TrialState::Fault?activeReason():"competing command/telemetry publisher";
                stopAndRelease(fault);logPlanningWait(fault,steady,ros);return;
            }
            if(autonomous_active_ && (raw_controller_->planningWaitActive() || !activeHealthy(steady,ros) || !driver_ready)){
                pausePlanning(!driver_ready?"driver endpoints unavailable; waiting":"telemetry unavailable; waiting",steady,ros);return;
            }
        }
        if(planning_headless_&&planning_resume_after_stop_&&planning_soft_stopped_&&!headless_blocked_){
            const bool driver_ready=exclusiveDriverReady(false);
            if(activeState()!=TrialState::Stopped){
                stopAndRelease("unexpected raw state during planning wait; reset required");
            }else if(!activeHealthy(steady,ros)||!driver_ready){
                planning_wait_reason_=!driver_ready?"driver endpoints unavailable; waiting":"telemetry unavailable; waiting";
                releaseMotor();logPlanningWait(planning_wait_reason_,steady,ros);
            }else if(planning_resume_pending_&&!planning_guard_->target(ros,steady)){
                pausePlanning("planning input expired; waiting for fresh command",steady,ros);
            }else if(planning_resume_pending_&&planningReady(steady,ros)&&stabilityReady()){
                if(planning_measured_speed_control_){
                    const auto next=planning_guard_->target(ros,steady);
                    if(next&&raw_controller_->resumePlanning(next->erpm,next->servo,steady,ros,driver_ready)){
                        planning_soft_stopped_=false;planning_resume_pending_=false;
                        autonomous_active_=true;commissioning_axis_=4;
                        resetPlanningSpeedFeedback();logRawTransition();
                    }
                }else if(planning_start_gate_->ready(steady)&&activeStationary(steady,ros)&&
                         raw_controller_->reset(steady,ros)){
                    planning_soft_stopped_=false;planning_resume_pending_=false;logRawTransition();
                }
            }
        }
        if(planning_headless_&&!headless_blocked_&&!autonomous_active_&&
           activeState()==TrialState::Idle&&planningReady(steady,ros)&&
           (planning_measured_speed_control_||planning_start_gate_->ready(steady))&&stabilityReady()){
            if(raw_controller_->start(RawAxis::Manual,0,raw_controller_->commandedServoPosition(),0,
                                      steady,ros,exclusiveDriverReady())){
                autonomous_active_=true;commissioning_axis_=4;resetPlanningSpeedFeedback();
            }
        }
        if(!autonomous_active_)return;
        const auto target=planning_guard_->target(ros,steady);
        // Legacy mode keeps its original limit. The measured-speed profile reduces
        // the speed request and recovers automatically instead of latching a stop.
        if(!planning_measured_speed_control_&&planning_feedback_limit_erpm_>0&&
           activeHealthy(steady,ros)&&abs(last_raw_erpm_)>planning_feedback_limit_erpm_){
            if(!planning_overspeed_since_)planning_overspeed_since_=steady;
            if(steady-*planning_overspeed_since_>=.2){
                stopAndRelease("autonomous measured ERPM limit exceeded; reset required");return;
            }
        }else planning_overspeed_since_.reset();
        double execution_erpm=target?target->erpm:0;
        if(planning_measured_speed_control_&&target&&activeHealthy(steady,ros)){
            const double dt=planning_feedback_tick_?steady-*planning_feedback_tick_:.02;
            try {
                // Shape once per accepted autonomous control tick.
                execution_erpm=dt==0?planning_feedback_target_:planningExecutionTarget(target->erpm,dt);
                planning_feedback_tick_=steady;
                planning_feedback_target_=execution_erpm;
                planning_feedback_reducing_=execution_erpm+1e-6<planning_running_target_;
            } catch(const invalid_argument&) {
                stopAndRelease("invalid measured-speed feedback or clock; reset required");return;
            }
        }else if(planning_headless_&&target&&!planning_measured_speed_control_){
            if((target->erpm==0||!planning_start_gate_->ready(steady))&&raw_controller_->targetMotorErpm()>0)
                planning_start_gate_->reset(steady,true);
            if(target->erpm>0&&!planning_start_gate_->ready(steady))execution_erpm=0;
        }
        if(!planningReady(steady,ros)||!target){
            onPlanningLost("planning input expired/unavailable; waiting for fresh command","autonomy expired or invalid",steady,ros);
            return;
        }
        if((!planning_headless_&&steady-autonomous_heartbeat_>limits::kOperatorHeartbeatS)||
           !raw_controller_->update(RawAxis::Manual,execution_erpm,target->servo,steady,ros)){
            stopAndRelease("autonomy hardware/timer fault or operator disconnected");
        }
    }

    void publishVehicleState(bool fresh,bool driver_ready) {
        if(!vehicle_state_pub_)return;
        nomad_interfaces::msg::VehicleState msg;msg.header.stamp=now();msg.header.frame_id="base_link";
        const auto gain=planning_calibration_.erpm_per_mps;
        msg.speed_valid=fresh&&planning_calibration_.verified&&planning_guard_->calibrated()&&(planning_feedback_sign_==1||planning_feedback_sign_==-1);
        msg.speed=msg.speed_valid?last_raw_erpm_*planning_feedback_sign_/gain:0;
        msg.steering_angle=0;msg.steering_valid=false; // 각도 센서 없음: 명령을 측정으로 보고하지 않습니다.
        msg.fault=activeState()==TrialState::Fault;
        // 준비 여부를 LPP 명령 수신에 종속시키면 양측이 서로 기다리는 교착이 생깁니다.
        msg.ready=commissioning_&&fresh&&driver_ready&&planning_guard_->calibrated()&&
            options_.hardware_timeout_verified&&raw_options_.setup_acknowledged&&
            (!stability_enabled_||stability_decision_.allowed)&&!msg.fault&&
            (!planning_headless_||!headless_blocked_);
        msg.detail=string(autonomous_active_?"LPP":"MANUAL/IDLE")+"; steering measurement unavailable; "+activeReason();
        vehicle_state_pub_->publish(msg);
    }
#endif
    // IMU는 차체 x=전방/y=좌측/z=위 축으로 보정된 입력만 허용합니다. 센서 프레임을 임의로 재명명하지 않습니다.
    void loadStability() {
        stability_enabled_=parameter<bool>("stability.enabled",false);
        if (stability_enabled_ && commissioning_)
            throw invalid_argument("stability enforcement needs calibrated m/s and actual steering mapping; raw mode unsupported");
        StabilityConfig c;
        c.geometry_verified=parameter<bool>("stability.geometry_verified",false);
        c.frame_verified=parameter<bool>("stability.frame_verified",false);
        c.track_m=parameter<double>("stability.track_m",0.0);
        c.wheelbase_m=parameter<double>("stability.wheelbase_m",0.0);
        c.cg_height_m=parameter<double>("stability.cg_height_m",0.0);
        c.cg_from_rear_m=parameter<double>("stability.cg_from_rear_m",0.0);
        c.cg_lateral_m=parameter<double>("stability.cg_lateral_m",0.0);
        c.margin=parameter<double>("stability.margin",c.margin);
        c.max_tilt_rad=parameter<double>("stability.max_tilt_rad",c.max_tilt_rad);
        c.max_rate_radps=parameter<double>("stability.max_rate_radps",c.max_rate_radps);
        c.timeout_s=parameter<double>("stability.timeout_s",c.timeout_s);
        c.max_speed_mps=parameter<double>("stability.max_speed_mps",c.max_speed_mps);
        c.max_accel_mps2=parameter<double>("stability.max_accel_mps2",c.max_accel_mps2);
        stability_guard_=std::make_unique<StabilityGuard>(c);
        imu_frame_=parameter<string>("stability.imu_frame","base_link");
        const auto topic=parameter<string>("stability.imu_topic","/nomad/imu_body");
        imu_sub_=create_subscription<sensor_msgs::msg::Imu>(topic,rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                const auto& q=m->orientation;
                const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
                // covariance=-1/전체 0은 자세 또는 정확도 미제공입니다. 잘못된 자세·프레임은 거부합니다.
                if (m->header.frame_id!=imu_frame_ || !isfinite(norm) || abs(norm-1)>0.01 ||
                    m->header.stamp.nanosec>=1000000000u ||
                    m->angular_velocity_covariance[0]<0 || m->linear_acceleration_covariance[0]<0) {
                    stability_guard_->invalidate(); return;
                }
                for (auto i : {0,4,8}) {
                    if (!isfinite(m->orientation_covariance[i]) || m->orientation_covariance[i]<=0 ||
                        m->orientation_covariance[i]>0.01) { stability_guard_->invalidate(); return; }
                }
                const double n=sqrt(norm),x=q.x/n,y=q.y/n,z=q.z/n,w=q.w/n;
                AttitudeSample s;
                s.stamp_s=static_cast<double>(m->header.stamp.sec)+m->header.stamp.nanosec*1e-9;
                s.roll=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
                s.pitch=asin(clamp(2*(w*y-z*x),-1.0,1.0));
                s.roll_rate=m->angular_velocity.x; s.pitch_rate=m->angular_velocity.y;
                s.force_x=m->linear_acceleration.x; s.force_y=m->linear_acceleration.y; s.force_z=m->linear_acceleration.z;
                stability_guard_->observe(s,steadySeconds(),now().seconds());
            });
    }

    bool stabilityReady() {
        if (!stability_enabled_) return true; // 기존 공중 raw 시험 경로. 지상 운행 허용을 뜻하지 않습니다.
        if (!controller_ || !options_.calibration_verified) {
            stability_decision_={}; stability_decision_.reason="stability needs verified speed/steering calibration"; return false;
        }
        const auto speed=controller_->measuredSpeed();
        if (!speed || count_publishers(imu_sub_->get_topic_name())!=1) {
            stability_decision_={}; stability_decision_.reason="stability needs speed and one IMU publisher"; return false;
        }
        // 조향을 중앙으로 요청해도 이전 명령각이 남아 있는 동안 선회 위험을 낮게 평가하지 않습니다.
        const double steering=max(abs(controller_->options().target_steering_rad),
                                       abs(controller_->commandedSteering()));
        stability_decision_=stability_guard_->evaluate(steadySeconds(),now().seconds(),*speed,steering);
        if (stability_decision_.allowed)
            controller_->setStabilityLimits(stability_decision_.speed_cap_mps,stability_decision_.accel_cap_mps2);
        return stability_decision_.allowed;
    }

    template <typename T>
    T parameter(const string& name, const T& value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true; // ROS 파라미터 변경은 차단합니다. GUI 목표 변경은 검증된 서비스만 거칩니다.
        return declare_parameter<T>(name, value, descriptor);
    }

    // 종료된 세션 ID를 재사용하지 않습니다. 한 프로세스에서 4096회 이후에는 재시작이 필요합니다.
    void closeManualSession() {
        if (!manual_session_.empty()) closed_sessions_.insert(manual_session_);
        manual_session_.clear();
        last_accepted_raw_update_at_.reset();
    }

    // 두 입력 서비스는 같은 세션 방어를 쓰지만 서로 다른 모드의 시험을 시작할 수 없습니다.
    void onManual(const Manual::Request& request, Manual::Response& response) {
        if (commissioning_) {
            response.accepted = false;
            response.reason = "calibrated manual_control is unavailable in raw commission mode";
            return;
        }
        handleSession(request, response,
            [&] { return controller_->startManual(request.speed_mps, request.steering_rad,
                request.duration_s, steadySeconds(), now().seconds(), exclusiveDriverReady()); },
            [&] { return controller_->updateManual(request.speed_mps, request.steering_rad,
                                                   steadySeconds(), now().seconds()); });
    }

    void onCommissioning(const Commissioning::Request& request, Commissioning::Response& response) {
        const double received_at = steadySeconds();
        if (!commissioning_) {
            response.accepted = false;
            response.reason = "raw commissioning_control requires mode=commission";
            return;
        }
#ifdef NOMAD_ENABLE_PLANNING
        if(planning_headless_&&request.action==Commissioning::Request::STOP){
            stopAndRelease("operator STOP in headless mode");
            response.accepted=true;response.reason="headless stopped; reset required";return;
        }
        if(planning_headless_&&request.action!=Commissioning::Request::STOP){
            response.accepted=false;response.reason="headless mode: manual output disabled; use GUI profile for manual control";return;
        }
        if(request.axis==Commissioning::Request::AUTONOMOUS){onAutonomous(request,response);return;}
#endif
        if(autonomous_active_&&request.action!=Commissioning::Request::STOP){
            response.accepted=false;response.reason="autonomy owns output; STOP before manual control";return;
        }
        // STOP은 axis/목표 필드 없이도 종료된 세션을 기록해야 합니다.
        if (request.action != Commissioning::Request::STOP &&
            request.axis != Commissioning::Request::MOTOR && request.axis != Commissioning::Request::STEERING &&
            request.axis != Commissioning::Request::MANUAL) {
            response.accepted = false;
            response.reason = "commissioning axis must be MOTOR, STEERING or MANUAL";
            return;
        }
        const auto axis = static_cast<RawAxis>(request.axis);
        const double command_stamp = static_cast<double>(request.stamp.sec) + request.stamp.nanosec * 1e-9;
        const double max_command_age = limits::commandMaxAge(axis == RawAxis::Manual, raw_options_.manual_timeout_s);
        handleSession(request, response,
            [&] {
                const bool driver_ready = exclusiveDriverReady();
                const double steady_now = steadySeconds(), ros_now = now().seconds();
                const bool accepted = raw_controller_->start(axis, request.motor_erpm, request.servo_position,
                    request.duration_s, steady_now, ros_now, driver_ready, ros_now - command_stamp);
                if (accepted) {
                    commissioning_axis_ = request.axis;
                    if (axis == RawAxis::Motor)
                        RCLCPP_INFO(get_logger(), "raw START accepted axis=MOTOR erpm=%.1f duration=%.2fs; no servo output",
                                    request.motor_erpm, request.duration_s);
                    else if (axis == RawAxis::Steering)
                        RCLCPP_INFO(get_logger(), "raw START accepted axis=STEERING servo=%.4f duration=%.2fs; motor=0A, angle unmeasured",
                                    request.servo_position, request.duration_s);
                    else
                        RCLCPP_INFO(get_logger(), "raw START accepted axis=MANUAL servo=%.4f; continuous heartbeat control; motor starts at 0 ERPM",
                                    request.servo_position);
                }
                return accepted;
            },
            [&] {
                const double steady_now = steadySeconds(), ros_now = now().seconds();
                return raw_controller_->update(axis, request.motor_erpm, request.servo_position,
                                               steady_now, ros_now, ros_now - command_stamp);
            },
            max_command_age);
        // 수락된 갱신 간격만 진단하며 세션 식별자나 목표값은 기록하지 않습니다.
        if (response.accepted && request.axis == Commissioning::Request::MANUAL &&
            (request.action == Commissioning::Request::START || request.action == Commissioning::Request::UPDATE)) {
            if (request.action == Commissioning::Request::UPDATE && last_accepted_raw_update_at_) {
                const double gap_ms = (received_at - *last_accepted_raw_update_at_) * 1000.0;
                if (gap_ms > 150.0) {
                    const double stamp = static_cast<double>(request.stamp.sec) + request.stamp.nanosec * 1e-9;
                    RCLCPP_INFO(get_logger(), "raw timing action=UPDATE accepted_gap_ms=%.1f command_age_ms=%.1f",
                                gap_ms, (now().seconds() - stamp) * 1000.0);
                }
            }
            last_accepted_raw_update_at_ = received_at;
        }
        logRawTransition();
    }

    template <typename Request, typename Response, typename Start, typename Update>
    void handleSession(const Request& request, Response& response, Start start, Update update,
                       double max_command_age_s = limits::kWiredCommandMaxAgeS) {
        response.accepted = false;
        response.reason = "invalid, expired or closed manual session";
        const bool valid_id = request.session.size() == 32 &&
            all_of(request.session.begin(), request.session.end(), [](unsigned char c) { return isxdigit(c); });
        if (!valid_id) return;
        if (!activeManual()) closeManualSession();
        // STOP이 START보다 먼저 도착해도 나중의 START가 구동을 되살리지 못하게 기록합니다.
        if (request.action == Request::STOP) {
            if (closed_sessions_.size() < 4096) closed_sessions_.insert(request.session);
            else sessions_exhausted_ = true;
            if (manual_session_ == request.session) {
                stopAndRelease("GUI session stopped");
            }
            response.accepted = true;
            response.reason = "session closed; 0A is coast, observe physical stop";
            return;
        }
        if (!stabilityReady()) {
            response.reason=stability_decision_.reason;
            if (activeState()==TrialState::Running) stopAndRelease(response.reason);
            return;
        }
        const double stamp = static_cast<double>(request.stamp.sec) + request.stamp.nanosec * 1e-9;
        const double age = now().seconds() - stamp;
        if (request.stamp.nanosec >= 1000000000u || stamp <= 0.0 || age < -limits::kFutureStampToleranceS || age > max_command_age_s || sessions_exhausted_ ||
            closed_sessions_.size() >= 4096 || closed_sessions_.count(request.session)) return;
        if (request.action == Request::START) {
            if (!manual_session_.empty()) return;
            response.accepted = start();
            if (response.accepted) { manual_session_ = request.session; manual_sequence_ = request.sequence; }
            else closed_sessions_.insert(request.session);
        } else if (request.action == Request::UPDATE && manual_session_ == request.session &&
                   request.sequence > manual_sequence_) {
            manual_sequence_ = request.sequence;
            response.accepted = update();
        } else return;
        response.reason = activeReason();
        if (!activeManual()) { closeManualSession(); releaseMotor(); }
    }

    // raw ERPM/서보 명령용 독립 설정입니다. 물리 단위의 가상 보정계수는 읽지 않습니다.
    RawOptions loadRawOptions() {
        RawOptions raw;
        raw.enabled = true;
        raw.setup_acknowledged = parameter<bool>("commissioning_setup_acknowledged", false);
        raw.hardware_timeout_verified = options_.hardware_timeout_verified;
        raw.motor_max_erpm = parameter<double>("commissioning.motor_max_erpm", raw.motor_max_erpm);
        raw.overspeed_erpm = parameter<double>("commissioning.overspeed_erpm", raw.overspeed_erpm);
        raw.servo_center = parameter<double>("commissioning.servo_center", raw.servo_center);
        raw.servo_min = parameter<double>("commissioning.servo_min", raw.servo_min);
        raw.servo_max = parameter<double>("commissioning.servo_max", raw.servo_max);
        raw.max_duration_s = parameter<double>("commissioning.max_duration_s", raw.max_duration_s);
        raw.telemetry_timeout_s = parameter<double>("commissioning.telemetry_timeout_s", raw.telemetry_timeout_s);
        raw.manual_timeout_s = parameter<double>("commissioning.manual_timeout_s", raw.manual_timeout_s);
        raw.max_tick_gap_s = parameter<double>("commissioning.max_tick_gap_s", raw.max_tick_gap_s);
        raw.stationary_hold_s = parameter<double>("commissioning.stationary_hold_s", raw.stationary_hold_s);
        raw.stationary_erpm = parameter<double>("commissioning.stationary_erpm", raw.stationary_erpm);
        raw.current_limit_a = parameter<double>("commissioning.current_limit_a", raw.current_limit_a);
        raw.motor_ramp_erpm_s = parameter<double>("commissioning.motor_ramp_erpm_s", raw.motor_ramp_erpm_s);
        raw.motor_deceleration_erpm_s = parameter<double>("commissioning.motor_deceleration_erpm_s", raw.motor_deceleration_erpm_s);
        raw.stall_watchdog_enabled = parameter<bool>("commissioning.stall_watchdog_enabled", raw.stall_watchdog_enabled);
        raw.stall_timeout_s = parameter<double>("commissioning.stall_timeout_s", raw.stall_timeout_s);
        raw.stall_command_erpm = parameter<double>("commissioning.stall_command_erpm", raw.stall_command_erpm);
        raw.stall_feedback_erpm = parameter<double>("commissioning.stall_feedback_erpm", raw.stall_feedback_erpm);
        raw.servo_rate_s = parameter<double>("commissioning.servo_rate_s", raw.servo_rate_s);
        raw.manual_servo_rate_s = parameter<double>("commissioning.manual_servo_rate_s", raw.manual_servo_rate_s);
        speed_pid_min_erpm_ = parameter<double>("speed_pid_min_erpm", 50.0);
        speed_pid_min_erpm_verified_ = parameter<bool>("speed_pid_min_erpm_verified", false);
        if (!isfinite(speed_pid_min_erpm_) || speed_pid_min_erpm_ < 0.0 || speed_pid_min_erpm_ > raw.motor_max_erpm)
            throw invalid_argument("invalid expected VESC speed PID minimum ERPM");
        // 입력 눈금과 사용자 관찰은 표시용 메타데이터이며 실제 각도 보정·속도 하한이 아닙니다.
        steering_input_limit_rad_ = parameter<double>("steering_input_limit_rad", 0.5);
        observed_smooth_start_erpm_ = parameter<double>("observed_smooth_start_erpm", 0.0);
        if (!isfinite(steering_input_limit_rad_) || steering_input_limit_rad_ <= 0.0 ||
            steering_input_limit_rad_ > 0.55)
            throw invalid_argument("invalid candidate steering input limit; expected 0 < radians <= 0.55");
        if (!isfinite(observed_smooth_start_erpm_) || observed_smooth_start_erpm_ < 0.0)
            throw invalid_argument("invalid observed smooth startup ERPM; zero means unset");
        return raw;
    }

    TrialState activeState() const { return commissioning_ ? raw_controller_->state() : controller_->state(); }
    const string& activeReason() const { return commissioning_ ? raw_controller_->reason() : controller_->reason(); }
    bool activeManual() const { return commissioning_ ? raw_controller_->manualActive() : controller_->manualActive(); }
    bool activeOwnsOutput() const { return commissioning_ ? raw_controller_->ownsOutput() : controller_->ownsOutput(); }
    bool activeHealthy(double steady, double ros) const {
        return commissioning_ ? raw_controller_->healthy(steady, ros) : controller_->healthy(steady, ros);
    }
    bool activeStationary(double steady, double ros) const {
        return commissioning_ ? raw_controller_->stationary(steady, ros) : controller_->stationary(steady, ros);
    }
    bool activeReset(double steady, double ros) {
        if (!commissioning_) return controller_->reset(steady, ros);
        const bool accepted = raw_controller_->reset(steady, ros);
#ifdef NOMAD_ENABLE_PLANNING
        if(accepted&&planning_headless_){
            headless_blocked_=false;planning_soft_stopped_=false;planning_resume_pending_=false;planning_guard_->clear();
            planning_start_gate_->reset(steady,true);planning_overspeed_since_.reset();resetPlanningSpeedFeedback();
        }
#endif
        logRawTransition();
        return accepted;
    }
    void activeStop(const string& reason) {
        autonomous_active_=false;
#ifdef NOMAD_ENABLE_PLANNING
        if(planning_headless_){headless_blocked_=true;planning_soft_stopped_=false;planning_resume_pending_=false;}
        if(planning_start_gate_)planning_start_gate_->reset(steadySeconds(),true);
        resetPlanningSpeedFeedback();
        planning_overspeed_since_.reset();
        if(planning_guard_)planning_guard_->clear();
#endif
        if (commissioning_) { raw_controller_->stop(reason); logRawTransition(); }
        else controller_->stop(reason);
    }
    // The one full stop path: end autonomy/trial, close the GUI session, release the motor with 0A.
    // Takes the reason by value because callers may pass activeReason(), which activeStop overwrites.
    void stopAndRelease(string reason) {
        activeStop(reason);
        closeManualSession();
        releaseMotor();
    }

    // 실제 관찰과 대조할 수 있도록 raw 시험 종료/이상 상태는 전환 시 한 번만 기록합니다.
    void logRawTransition() {
        if (!commissioning_ || !raw_controller_) return;
        const auto state = raw_controller_->state();
        if (state == last_logged_raw_state_) return;
        last_logged_raw_state_ = state;
        if (state == TrialState::Stopped || state == TrialState::Fault)
            RCLCPP_INFO(get_logger(), "raw transition=%s axis=%u raw_erpm=%.1f reason=%s",
                        trialStateName(state), static_cast<unsigned>(commissioning_axis_),
                        last_raw_erpm_, raw_controller_->reason().c_str());
    }

    Calibration loadCalibration() {
        Calibration c = config::kCalibration;
        c.speed_gain = parameter<double>("calibration.speed_gain", c.speed_gain);
        c.speed_offset = parameter<double>("calibration.speed_offset", c.speed_offset);
        c.steering_gain = parameter<double>("calibration.steering_gain", c.steering_gain);
        c.steering_offset = parameter<double>("calibration.steering_offset", c.steering_offset);
        c.command_erpm_sign = parameter<int>("calibration.command_erpm_sign", c.command_erpm_sign);
        c.feedback_erpm_sign = parameter<int>("calibration.feedback_erpm_sign", c.feedback_erpm_sign);
        c.speed_min_mps = parameter<double>("calibration.speed_min_mps", c.speed_min_mps);
        c.speed_max_mps = parameter<double>("calibration.speed_max_mps", c.speed_max_mps);
        c.steering_min_rad = parameter<double>("calibration.steering_min_rad", c.steering_min_rad);
        c.steering_max_rad = parameter<double>("calibration.steering_max_rad", c.steering_max_rad);
        c.servo_min = parameter<double>("calibration.servo_min", c.servo_min);
        c.servo_max = parameter<double>("calibration.servo_max", c.servo_max);
        c.motor_erpm_min = parameter<double>("calibration.motor_erpm_min", c.motor_erpm_min);
        c.motor_erpm_max = parameter<double>("calibration.motor_erpm_max", c.motor_erpm_max);
        return c;
    }

    // 시작 요청은 항상 graph를 새로 검사합니다. 50Hz 제어 중에는 최대 100ms까지만 결과를 재사용합니다.
    bool exclusiveDriverReady(bool force_refresh = true) {
        if (!speed_pub_ || !current_pub_ || !servo_pub_) return false;
        const double steady_now = steadySeconds();
        if (!force_refresh && last_graph_check_at_ && steady_now - *last_graph_check_at_ >= 0.0 &&
            steady_now - *last_graph_check_at_ < 0.1) return cached_driver_ready_;
        last_graph_check_at_ = steady_now;
        graph_telemetry_publishers_=count_publishers(telemetry_sub_->get_topic_name());
        graph_speed_subscribers_=speed_pub_->get_subscription_count();
        graph_current_subscribers_=current_pub_->get_subscription_count();
        graph_servo_subscribers_=servo_pub_->get_subscription_count();
        graph_speed_publishers_=count_publishers(speed_pub_->get_topic_name());
        graph_current_publishers_=count_publishers(current_pub_->get_topic_name());
        graph_servo_publishers_=count_publishers(servo_pub_->get_topic_name());
        graph_other_publishers_=0;
        for(const auto& topic:other_command_topics_)graph_other_publishers_+=count_publishers(topic);
        graph_competing_publishers_=graph_telemetry_publishers_>1||graph_speed_publishers_>1||
            graph_current_publishers_>1||graph_servo_publishers_>1||graph_other_publishers_>0;
        cached_driver_ready_=!graph_competing_publishers_&&graph_telemetry_publishers_==1&&
            graph_speed_subscribers_>0&&graph_current_subscribers_>0&&graph_servo_subscribers_>0&&
            graph_speed_publishers_==1&&graph_current_publishers_==1&&graph_servo_publishers_==1;
        return cached_driver_ready_;
    }

    void onTelemetry(const VescState& message) {
        const auto& s = message.state;
        const double stamp = static_cast<double>(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9;
        const TelemetrySample sample{stamp, s.speed, s.current_motor, s.current_input, s.voltage_input, s.fault_code};
        if (commissioning_) raw_controller_->observe(sample, steadySeconds(), now().seconds());
        else controller_->observe(sample, steadySeconds(), now().seconds());
        last_raw_erpm_ = s.speed;
        last_voltage_ = s.voltage_input;
        last_current_ = s.current_motor;
        last_fault_ = s.fault_code;
        logRawTransition();
        if (activeState() == TrialState::Fault) releaseMotor();
    }

    void publishScalar(const rclcpp::Publisher<Scalar>::SharedPtr& publisher, double value) {
        Scalar message;
        message.data = value;
        publisher->publish(message);
    }

    // 자신이 시작한 시험에 대해서만 출력합니다. 정지 시 서보를 갑자기 중앙으로 돌리지 않습니다.
    void releaseMotor() {
        if (current_pub_ && activeOwnsOutput()) publishScalar(current_pub_, 0.0);
    }

    // raw 모터 시험의 optional 서보가 비어 있으면 서보 토픽에는 아무것도 보내지 않습니다.
    void publishOutput(OutputKind kind, double erpm, optional<double> servo) {
        if (kind == OutputKind::Release) releaseMotor();
        else if (kind == OutputKind::Drive) {
            if (servo) publishScalar(servo_pub_, *servo);
            if (erpm == 0.0) releaseMotor();
            else publishScalar(speed_pub_, erpm);
        }
    }

    void controlTick() {
        const double steady_now = steadySeconds();
        const double ros_now = now().seconds();
        const bool driver_ready = exclusiveDriverReady(false);
#ifdef NOMAD_ENABLE_PLANNING
        tickPlanning(steady_now,ros_now);
#endif
        if (!stabilityReady() && activeState()==TrialState::Running) {
            stopAndRelease(stability_decision_.reason);
        }
        if (commissioning_) {
            double minimum_drive_erpm=0;
#ifdef NOMAD_ENABLE_PLANNING
            if(autonomous_active_&&planning_measured_speed_control_&&!planning_feedback_reducing_)
                minimum_drive_erpm=planning_min_running_erpm_;
#endif
            const auto output = raw_controller_->tick(steady_now, ros_now, driver_ready,minimum_drive_erpm);
            publishOutput(output.kind, output.motor_erpm, output.servo_position);
        } else {
            const auto output = controller_->tick(steady_now, ros_now, driver_ready);
            publishOutput(output.kind, output.motor_erpm, output.servo_position);
        }

        logRawTransition();
        if(activeState()!=TrialState::Running){
#ifdef NOMAD_ENABLE_PLANNING
            if(planning_headless_&&autonomous_active_){
                headless_blocked_=true;planning_soft_stopped_=false;planning_resume_pending_=false;
            }
#endif
            autonomous_active_=false;
        }
        if (!activeManual()) closeManualSession();
        if (steady_now - last_status_at_ < 0.1) return;
        last_status_at_ = steady_now;
        const bool fresh = activeHealthy(steady_now, ros_now);
        const bool stationary = activeStationary(steady_now, ros_now);
#ifdef NOMAD_ENABLE_PLANNING
        publishVehicleState(fresh,driver_ready);
#endif
        const auto speed = commissioning_ ? optional<double>{} : controller_->measuredSpeed();
        optional<ActuatorCommand> preview;
        if (!commissioning_) {
            const auto& active = controller_->options();
            preview = convertTarget(active.target_speed_mps, active.target_steering_rad, calibration_).value;
        }
        ostringstream text;
        text << "state=" << trialStateName(activeState()) << " telemetry_fresh=" << fresh
             << " control_units=" << (commissioning_ ? "raw" : "calibrated")
             << " calibration_verified=" << (!commissioning_ && options_.calibration_verified)
             << " stationary=" << stationary << " raw_erpm=" << last_raw_erpm_ << " speed_mps=";
        if (speed) text << *speed; else text << "unavailable";
        text << " voltage_v=" << last_voltage_ << " current_motor_a=" << last_current_
             << " fault_code=" << last_fault_ << " steering_measured=false";
        if (preview) text << " preview_erpm=" << preview->motor_erpm << " preview_servo=" << preview->servo_position;
        text << " reason=" << activeReason();
        std_msgs::msg::String status;
        status.data = text.str();
        status_pub_->publish(status);
        nlohmann::json data = {
            {"state", trialStateName(activeState())}, {"telemetry_fresh", fresh},
            {"control_units", commissioning_ ? "raw" : "calibrated"}, {"commissioning", commissioning_},
            {"calibration_verified", !commissioning_ && options_.calibration_verified},
            {"hardware_timeout_verified", options_.hardware_timeout_verified},
            {"driver_ready", driver_ready}, {"stationary", stationary},
            {"raw_erpm", last_raw_erpm_}, {"voltage_v", last_voltage_}, {"current_a", last_current_},
            {"fault_code", last_fault_}, {"steering_measured", false},
            {"manual_active", activeManual()},
            {"remaining_s", commissioning_ ? raw_controller_->remaining(steady_now) : controller_->remaining(steady_now)},
            {"reason", activeReason()},
            {"manual_timeout_s", commissioning_ ? raw_options_.manual_timeout_s : options_.manual_timeout_s}
        };
        data["speed_mps"] = speed ? nlohmann::json(*speed) : nlohmann::json(nullptr);
        data["stability_enabled"]=stability_enabled_;
        data["stability_ready"]=stability_enabled_ && stability_decision_.allowed;
        data["ground_drive_supported"]=false;
        data["stability_reason"]=stability_enabled_?stability_decision_.reason:"raised bench only; stability protection disabled";
        data["stability_speed_cap_mps"]=stability_enabled_ && stability_decision_.allowed?
            nlohmann::json(stability_decision_.speed_cap_mps):nlohmann::json(nullptr);
        data["preview_erpm"] = preview ? nlohmann::json(preview->motor_erpm) : nlohmann::json(nullptr);
        data["preview_servo"] = preview ? nlohmann::json(preview->servo_position) : nlohmann::json(nullptr);
        const bool wifi_manual = commissioning_ && limits::wifiManualProfile(raw_options_.manual_timeout_s);
        data["wifi_manual_profile"] = wifi_manual;
        const double command_max_age = wifi_manual ? limits::kWifiManualCommandMaxAgeS : limits::kWiredCommandMaxAgeS;
        data["command_max_age_s"] = command_max_age;
        data["browser_freshness_s"] = command_max_age;
        data["browser_request_timeout_ms"] = wifi_manual ? 1500 : 600;
        if (commissioning_) {
            data["commissioning_setup_acknowledged"] = raw_options_.setup_acknowledged;
            data["commissioning_axis"] = commissioning_axis_;
            data["motor_max_erpm"] = raw_options_.motor_max_erpm;
            data["servo_min"] = raw_options_.servo_min;
            data["servo_max"] = raw_options_.servo_max;
            data["servo_center"] = raw_options_.servo_center;
            data["servo_center_is_candidate"] = true;
            data["steering_range_config_supported"]=true;
            data["steering_test_servo_min"]=raw_controller_->steeringTestMin();
            data["steering_test_servo_max"]=raw_controller_->steeringTestMax();
            data["steering_test_span_max"]=min(raw_options_.servo_center,1.0-raw_options_.servo_center);
            data["steering_outside_drive_range"]=raw_controller_->outsideDriveRange();
            data["max_duration_s"] = raw_options_.max_duration_s;
            data["continuous_manual_supported"] = true;
            data["continuous_manual_active"] = raw_controller_->continuousManualActive();
            // 명령값만 공개합니다. 서보 위치 센서나 실측 조향각을 의미하지 않습니다.
            data["commanded_servo_position"] = raw_controller_->commandedServoPosition();
            data["commanded_motor_erpm"] = raw_controller_->commandedMotorErpm();
            data["target_motor_erpm"] = raw_controller_->targetMotorErpm();
            data["requested_servo_position"] = raw_controller_->requestedServoPosition();
            data["servo_rate_s"] = raw_options_.manual_servo_rate_s;
            data["motor_control_mode"] = "vesc_speed_pid";
            data["speed_pid_min_erpm"] = speed_pid_min_erpm_;
            data["speed_pid_min_erpm_verified"] = speed_pid_min_erpm_verified_;
            data["steering_input_limit_rad"] = steering_input_limit_rad_;
            data["observed_smooth_start_erpm"] = observed_smooth_start_erpm_;
            data["observed_smooth_start_note"] = "공중 시험 · 사용자 관찰 · 실제 주행 최저속도 미보정";
            data["control_tick_hz"] = 50;
            data["motor_acceleration_erpm_s"] = raw_options_.motor_ramp_erpm_s;
            data["motor_deceleration_erpm_s"] = raw_options_.motor_deceleration_erpm_s;
            data["stall_watchdog_enabled"] = raw_options_.stall_watchdog_enabled;
        } else {
            data["speed_min"] = calibration_.speed_min_mps;
            data["speed_max"] = calibration_.speed_max_mps;
            data["steering_min"] = calibration_.steering_min_rad;
            data["steering_max"] = calibration_.steering_max_rad;
        }
        data["autonomous_active"]=autonomous_active_;
#ifdef NOMAD_ENABLE_PLANNING
        data["planning_supported"]=true;
        data["planning_calibration_verified"]=planning_calibration_.verified;
        data["planning_headless"]=planning_headless_;
        data["planning_resume_after_stop"]=planning_resume_after_stop_;
        data["planning_soft_stopped"]=planning_soft_stopped_;
        data["planning_estimated_execution"]=planning_allow_estimated_&&!planning_calibration_.verified;
        data["planning_reset_required"]=headless_blocked_;
        data["planning_ready"]=planningReady(steady_now,ros_now);
        data["planning_reason"]=planning_guard_->targetReason(ros_now,steady_now);
        data["planning_start_reason"]=planningStartReason(steady_now);
        data["planning_start_ready"]=planning_measured_speed_control_?
            (planningReady(steady_now,ros_now)&&fresh&&!headless_blocked_):planning_start_gate_->ready(steady_now);
        data["planning_measured_speed_control"]=planning_measured_speed_control_;
        data["planning_minimum_running_erpm"]=planning_min_running_erpm_;
        data["planning_running_target_erpm"]=planning_running_target_;
        data["planning_feedback_target_erpm"]=planning_feedback_target_;
        data["planning_feedback_reducing"]=planning_feedback_reducing_;
        data["planning_minimum_start_erpm"]=planning_measured_speed_control_?0:planning_min_start_erpm_;
        data["planning_minimum_start_mps"]=!planning_measured_speed_control_&&planning_calibration_.erpm_per_mps>0?planning_min_start_erpm_/planning_calibration_.erpm_per_mps:0;
        data["planning_feedback_limit_erpm"]=planning_feedback_limit_erpm_;
        if(commissioning_){
            const auto requested=planning_guard_->target(ros_now,steady_now);
            data["planning_requested_erpm"]=requested?nlohmann::json(requested->erpm):nlohmann::json(nullptr);
            RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
                "control_trace source=%s phase=%s requested_erpm=%.1f target_erpm=%.1f output_erpm=%.1f measured_erpm=%.1f current_a=%.2f start_gate=%s reset_required=%d",
                autonomous_active_?"LPP":"IDLE/MANUAL",trialStateName(activeState()),requested?requested->erpm:0,
                raw_controller_->targetMotorErpm(),raw_controller_->commandedMotorErpm(),last_raw_erpm_,last_current_,
                planningStartReason(steady_now).c_str(),static_cast<int>(headless_blocked_));
        }
        data["planning_speed_mps"]=isfinite(planning_speed_)?nlohmann::json(planning_speed_):nlohmann::json(nullptr);
        data["planning_steering_rad"]=isfinite(planning_angle_)?nlohmann::json(planning_angle_):nlohmann::json(nullptr);
#else
        data["planning_supported"]=false;data["planning_ready"]=false;
#endif
        status.data = data.dump();
        status_json_pub_->publish(status);
    }

    bool commissioning_ = false;
    bool autonomous_active_=false;
#ifdef NOMAD_ENABLE_PLANNING
    PlanningCalibration planning_calibration_;
    unique_ptr<PlanningCommandGuard> planning_guard_;
    unique_ptr<PlanningStartGate> planning_start_gate_;
    unique_ptr<PlanningSpeedFeedback> planning_speed_feedback_;
    double planning_running_target_=0;
    double planning_min_running_erpm_=0;
    bool planning_measured_speed_control_=false,planning_feedback_reducing_=false;
    double planning_feedback_target_=0;
    optional<double> planning_feedback_tick_;
    double planning_start_hold_s_=.75,planning_restart_cooldown_s_=1,planning_min_start_erpm_=0,planning_feedback_limit_erpm_=0;
    optional<double> planning_overspeed_since_;
    bool planning_headless_=false,planning_allow_estimated_=false,headless_blocked_=false;
    bool planning_resume_after_stop_=false,planning_soft_stopped_=false,planning_resume_pending_=false;
    rclcpp::Subscription<nomad_interfaces::msg::DriveCommand>::SharedPtr planning_sub_;
    rclcpp::Publisher<nomad_interfaces::msg::VehicleState>::SharedPtr vehicle_state_pub_;
    double autonomous_heartbeat_=0,planning_speed_=NAN,planning_angle_=NAN;
    int planning_feedback_sign_=0;
#endif
    bool stability_enabled_=false;
    string imu_frame_;
    unique_ptr<StabilityGuard> stability_guard_;
    StabilityDecision stability_decision_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    uint8_t commissioning_axis_ = 0;
    TrialState last_logged_raw_state_ = TrialState::Idle;
    RawOptions raw_options_;
    double speed_pid_min_erpm_ = 50.0;
    bool speed_pid_min_erpm_verified_ = false;
    double steering_input_limit_rad_ = 0.5;
    rclcpp::Service<SteeringRange>::SharedPtr steering_range_service_;
    double observed_smooth_start_erpm_ = 0.0;
    unique_ptr<RawTrialController> raw_controller_;
    TrialOptions options_;
    Calibration calibration_{};
    unique_ptr<TrialController> controller_;
    rclcpp::Publisher<Scalar>::SharedPtr speed_pub_, current_pub_, servo_pub_;
    size_t graph_telemetry_publishers_=0,graph_speed_subscribers_=0,graph_current_subscribers_=0,graph_servo_subscribers_=0;
    size_t graph_speed_publishers_=0,graph_current_publishers_=0,graph_servo_publishers_=0,graph_other_publishers_=0;
    bool graph_competing_publishers_=false;
    string planning_wait_reason_="LPP pause; waiting for fresh command";
    rclcpp::Subscription<VescState>::SharedPtr telemetry_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, status_json_pub_;
    rclcpp::Service<Manual>::SharedPtr manual_service_;
    rclcpp::Service<Commissioning>::SharedPtr commissioning_service_;
    string manual_session_;
    set<string> closed_sessions_;
    uint64_t manual_sequence_ = 0;
    optional<double> last_accepted_raw_update_at_;
    bool sessions_exhausted_ = false;
    rclcpp::Service<Trigger>::SharedPtr start_service_, stop_service_, reset_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr time_guard_;
    vector<string> other_command_topics_;
    optional<double> last_graph_check_at_;
    bool cached_driver_ready_ = false;
    double last_status_at_ = 0.0;
    double last_raw_erpm_ = 0.0, last_voltage_ = 0.0, last_current_ = 0.0;
    int last_fault_ = -1;
};
} // namespace

shared_ptr<ControllerNode> ControllerNode::create(const rclcpp::NodeOptions& options) {
    return std::make_shared<ControllerNodeImpl>(options);
}
} // namespace nomad

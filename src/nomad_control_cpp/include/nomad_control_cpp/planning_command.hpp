#pragma once
#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <algorithm>
#include "nomad_control_cpp/control_limits.hpp"

namespace nomad {
// DriveCommand의 물리 단위 입력. 서보 0~1과 혼용하지 않습니다.
struct PlanningCommand {
    double stamp=0,valid_for=0,speed_mps=0,steering_rad=0;
    std::string frame;
    bool stop=false;
};
struct PlanningCalibration {
    bool verified=false;
    double erpm_per_mps=0,servo_per_rad=0,servo_zero=.5;
    double max_mps=0,max_rad=0;
    // rad는 LPP 등가각, servo는 원시 VESC 명령. 비어 있으면 기존 선형식을 사용합니다.
    std::vector<double> steering_rads,steering_servos;
    bool validSteering() const {
        if(steering_rads.empty()&&steering_servos.empty())
            return std::isfinite(servo_per_rad)&&servo_per_rad!=0;
        if(steering_rads.size()<3||steering_rads.size()!=steering_servos.size())return false;
        bool zero_found=false;
        const bool increasing=steering_servos.back()>steering_servos.front();
        for(std::size_t i=0;i<steering_rads.size();++i){
            if(!std::isfinite(steering_rads[i])||!std::isfinite(steering_servos[i])||steering_servos[i]<0||steering_servos[i]>1)return false;
            if(steering_rads[i]==0)zero_found=std::abs(steering_servos[i]-servo_zero)<1e-9;
            if(i&&(steering_rads[i]<=steering_rads[i-1]||
                (increasing?steering_servos[i]<=steering_servos[i-1]:steering_servos[i]>=steering_servos[i-1])))return false;
        }
        return zero_found&&steering_rads.front()<=-max_rad&&steering_rads.back()>=max_rad;
    }
    // 검증된 표 안에서만 선형 보간합니다. 표 밖의 각도는 외삽하지 않습니다.
    double servoFor(double rad) const {
        if(!std::isfinite(rad)||!validSteering())return NAN;
        if(steering_rads.empty())return servo_zero+rad*servo_per_rad;
        if(rad<steering_rads.front()||rad>steering_rads.back())return NAN;
        const auto hi=std::lower_bound(steering_rads.begin(),steering_rads.end(),rad);
        const auto i=static_cast<std::size_t>(hi-steering_rads.begin());
        if(i==0||*hi==rad)return steering_servos[i];
        const double t=(rad-steering_rads[i-1])/(steering_rads[i]-steering_rads[i-1]);
        return steering_servos[i-1]+t*(steering_servos[i]-steering_servos[i-1]);
    }
};
struct PlanningTarget {double erpm=0,servo=.5;};

// 발행 stamp 기준 유효기간과 마지막 수신 이후 경과 시간을 함께 감시합니다.
class PlanningCommandGuard {
public:
    // max_erpm should be the raw controller motor_max_erpm so both layers refuse the same targets.
    explicit PlanningCommandGuard(const PlanningCalibration& calibration,double max_erpm=limits::kMotorMaxErpm)
        :calibration_(calibration),max_erpm_(max_erpm){}
    void clear(){command_.reset();reason_="waiting for new planning command";}
    bool calibrated() const {
        const auto& c=calibration_;
        return c.verified&&std::isfinite(c.erpm_per_mps)&&c.erpm_per_mps>0&&
            std::isfinite(c.servo_zero)&&c.servo_zero>=0&&c.servo_zero<=1&&
            std::isfinite(c.max_mps)&&c.max_mps>0&&std::isfinite(c.max_rad)&&c.max_rad>0&&c.validSteering();
    }
    bool receive(const PlanningCommand& command,double ros_now,double steady_now) {
        if(!std::isfinite(command.stamp)||command.stamp<=0||command.stamp<last_stamp_){reason_="old/invalid planning stamp";return false;}
        if(command.stamp==last_stamp_){if(command.stop){command_.reset();reason_="planning STOP";}return false;}
        if(command.stop){
            // 정상 시각의 STOP보다 먼저 발행된 이동 명령이 뒤늦게 도착해도 되살리지 않습니다.
            if(std::isfinite(ros_now)&&command.stamp<=ros_now+limits::kFutureStampToleranceS)last_stamp_=command.stamp;
            command_.reset();reason_="planning STOP";return false;
        }
        const double age=ros_now-command.stamp;
        if(command.frame!="base_link"||!std::isfinite(ros_now)||!std::isfinite(steady_now)||
           !std::isfinite(command.valid_for)||command.valid_for<=0||command.valid_for>limits::kPlanningMaxValidForS||
           age<-limits::kFutureStampToleranceS||age>command.valid_for||!std::isfinite(command.speed_mps)||!std::isfinite(command.steering_rad)){
            command_.reset();reason_="invalid/stale planning command";return false;
        }
        last_stamp_=command.stamp;
        command_=command;received_=steady_now;reason_="planning command received";return true;
    }
    std::string targetReason(double ros_now,double steady_now) const {
        const auto& c=calibration_;
        if(!calibrated())return "planning calibration unavailable";
        if(!command_)return reason_;
        const auto& cmd=*command_;
        const double age=ros_now-cmd.stamp,wall_age=steady_now-received_;
        if(!std::isfinite(age)||!std::isfinite(wall_age)||age<-limits::kFutureStampToleranceS||wall_age<0)return "planning clock invalid";
        if(age>cmd.valid_for||wall_age>limits::kPlanningReceiveLeaseS)return "planning command expired";
        if(cmd.speed_mps<0||cmd.speed_mps>c.max_mps)return "planning speed outside configured range";
        if(std::abs(cmd.steering_rad)>c.max_rad)return "planning steering outside configured range";
        const double erpm=cmd.speed_mps*c.erpm_per_mps,servo=c.servoFor(cmd.steering_rad);
        if(!std::isfinite(erpm)||erpm>max_erpm_||!std::isfinite(servo)||servo<0||servo>1)return "planning mapped actuator outside range";
        return "ready";
    }
    std::optional<PlanningTarget> target(double ros_now,double steady_now) const {
        if(targetReason(ros_now,steady_now)!="ready")return {};
        const auto& c=calibration_;const auto& cmd=*command_;
        const PlanningTarget out{cmd.speed_mps*c.erpm_per_mps,c.servoFor(cmd.steering_rad)};
        if(!std::isfinite(out.erpm)||out.erpm>max_erpm_||!std::isfinite(out.servo)||out.servo<0||out.servo>1)return {};
        return out;
    }
    const std::string& reason() const{return reason_;}
private:
    PlanningCalibration calibration_;
    double max_erpm_;
    std::optional<PlanningCommand> command_;
    double last_stamp_=0,received_=0;
    std::string reason_="waiting for planning";
};
}

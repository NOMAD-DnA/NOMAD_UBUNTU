#pragma once
#include <QtCore>
#include <cmath>
#include <algorithm>
#include "drive_view.hpp"
// 메모리 내부의 모의 장치. 소켓, SSH, ROS, USB를 생성하지 않습니다.
class Simulation {
    QElapsedTimer clock;
    QString state="IDLE",owner;
    QSet<QString> closed;
    double erpm=0,servo=.5,last=0,heartbeat=0;
    quint64 sequence=0;
    bool automatic=false;
    SimPose pose;
public:
    Simulation(){clock.start();}
    QJsonObject handle(const QString& route,const QJsonObject& data){
        const double now=clock.elapsed()/1000.;
        if(state=="RUNNING"&&now-heartbeat>2){state="STOPPED_OUTPUT";erpm=0;closed.insert(owner);owner.clear();}
        if(route=="status"){
            const double dt=std::clamp(now-last,0.,.2);last=now;
            const double speed=.5+.2*std::sin(now*.7),angle=.4*std::sin(now);
            if(state=="RUNNING"&&automatic){
                erpm+=std::clamp(speed*1000-erpm,-2400*dt,1200*dt);
                servo+=std::clamp(.5+angle/1.4-servo,-.4*dt,.4*dt);
            }
            const double equivalent=(servo-.5)*1.4;
            pose.step(erpm/1000.,equivalent,dt);
            return {{"token","local-simulation"},{"server_time_s",QDateTime::currentMSecsSinceEpoch()/1000.},
                {"state",state},{"control_units","raw"},{"connected",true},{"telemetry_fresh",true},
                {"stationary",erpm==0},{"driver_ready",true},{"hardware_timeout_verified",true},
                {"commissioning_setup_acknowledged",true},{"continuous_manual_supported",true},
                {"servo_min",0},{"servo_max",1},{"commanded_servo_position",servo},{"raw_erpm",erpm},
                {"voltage_v",16.0},{"current_a",erpm/800.},{"motor_max_erpm",1850},
                {"wifi_manual_profile",true},{"commissioning",true},{"manual_timeout_s",2},
                {"command_max_age_s",1.5},{"browser_freshness_s",1.5},{"browser_request_timeout_ms",1500},
                {"motor_control_mode","vesc_speed_pid"},{"speed_pid_min_erpm_verified",true},{"speed_pid_min_erpm",50},
                {"planning_supported",true},{"planning_ready",true},{"planning_calibration_verified",true},
                {"planning_speed_mps",speed},{"planning_steering_rad",angle},{"sim_x",pose.x},{"sim_y",pose.y},{"sim_yaw",pose.yaw},{"sim_angle",equivalent}};
        }
        if(route=="reset-pose"&&state!="RUNNING"){pose={};return {{"accepted",true}};}
        bool ok=false;const auto session=data["session"].toString();const auto seq=static_cast<quint64>(data["sequence"].toDouble());
        const int action=data["action"].toInt();
        if(route=="stop"||action==3){
            closed.insert(session);if(!owner.isEmpty())closed.insert(owner);
            owner.clear();state="STOPPED_OUTPUT";erpm=0;ok=true;
        }else if(route=="reset"&&state!="RUNNING"){state="IDLE";ok=true;}
        else if(route=="commissioning"&&!session.isEmpty()&&!closed.contains(session)){
            if(action==1&&state=="IDLE"&&data["motor_erpm"].toDouble()==0){
                owner=session;sequence=seq;automatic=data["axis"].toInt()==4;state="RUNNING";erpm=0;ok=true;
            }else if(action==2&&state=="RUNNING"&&owner==session&&seq>sequence){
                sequence=seq;ok=true;
                if(!automatic){erpm=std::clamp(data["motor_erpm"].toDouble(),0.,1850.);servo=std::clamp(data["servo_position"].toDouble(),0.,1.);}
            }
            if(ok)heartbeat=now;
        }
        return {{"accepted",ok},{"reason",ok?"모의 명령 적용":"모의 세션 상태 확인 필요"}};
    }
};

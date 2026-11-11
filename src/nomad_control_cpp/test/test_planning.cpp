#include "nomad_control_cpp/planning_command.hpp"
#include <iostream>
#include <limits>
#include <random>
#include "nomad_control_cpp/vehicle_reference.hpp"

using namespace std;
int main(){
    int failures=0;
    auto check=[&](bool ok,const char* name){if(!ok){cerr<<name<<'\n';++failures;}};
    // 합성 보정값은 단위/유효기간 검사용이며 차량 설정이 아닙니다.
    nomad::PlanningCalibration c{true,1000,-.5,.5,1,.7};
    nomad::PlanningCommandGuard g(c);
    nomad::PlanningCommand cmd{100,.3,.5,.4,"base_link",false};
    check(g.receive(cmd,100,1),"receive fresh");
    auto t=g.target(100.1,1.1);
    check(t&&t->erpm==500&&abs(t->servo-.3)<1e-9,"m/s and rad conversion/sign");
    check(!g.receive(cmd,100.2,1.2),"duplicates do not renew");
    check(!g.target(100.31,1.31),"source expiry");
    check(!g.target(100.1,1.36),"steady timeout even if ROS clock stalls");
    cmd.stop=true;g.receive(cmd,100.2,1.2);check(!g.target(100.2,1.2),"same stamp STOP");
    cmd.stop=false;cmd.stamp=101;cmd.speed_mps=-.1;g.receive(cmd,101,2);check(!g.target(101,2),"reverse rejected");
    cmd.stamp=102;cmd.speed_mps=.5;cmd.steering_rad=.8;g.receive(cmd,102,3);check(!g.target(102,3),"angle bounds");
    cmd.stamp=103;cmd.steering_rad=0;cmd.frame="map";check(!g.receive(cmd,103,4),"frame contract");
    cmd.frame="base_link";cmd.valid_for=.6;check(!g.receive(cmd,103,4),"lease maximum");
    cmd.valid_for=.3;cmd.stamp=10000;check(!g.receive(cmd,103,4),"future rejected");
    cmd.stamp=103;check(g.receive(cmd,103,4),"future rejection cannot poison subsequent stamps");
    g.clear();check(!g.target(103,4),"disarm clears cached command");
    c.verified=false;nomad::PlanningCommandGuard unverified(c);unverified.receive(cmd,103,4);check(!unverified.target(103,4),"unverified calibration cannot actuate");
    c.verified=true;c.erpm_per_mps=4000;nomad::PlanningCommandGuard excessive(c);excessive.receive(cmd,103,4);check(!excessive.target(103,4),"motor limit");
    {
        // The guard follows the configured raw motor limit instead of a fixed 1850 ERPM.
        nomad::PlanningCalibration lim{true,4000,-.5,.5,.25,.7};
        nomad::PlanningCommand slow{500,.3,.25,0,"base_link",false};
        nomad::PlanningCommandGuard defaultLimit(lim);defaultLimit.receive(slow,500,10);
        check(defaultLimit.target(500,10).has_value(),"1000 ERPM accepted under default limit");
        nomad::PlanningCommandGuard lowered(lim,900);lowered.receive(slow,500,10);
        check(!lowered.target(500,10)&&lowered.targetReason(500,10)=="planning mapped actuator outside range",
              "configured motor limit rejects targets above it");
    }
    nomad::PlanningCommandGuard stopped({true,1000,-.5,.5,1,.7});
    cmd={200,.3,.2,0,"base_link",false};stopped.receive(cmd,200,1);
    cmd.stamp=200.1;cmd.stop=true;stopped.receive(cmd,200.1,1.1);
    cmd.stamp=200.05;cmd.stop=false;
    check(!stopped.receive(cmd,200.11,1.11)&&!stopped.target(200.11,1.11),"older motion cannot supersede newer STOP");
    cmd.stamp=200.12;check(stopped.receive(cmd,200.12,1.12),"new motion after STOP can be received for explicit rearm");
    const double bad[]={NAN,INFINITY,-INFINITY};
    for(double v:bad){
        nomad::PlanningCommandGuard badGuard({true,1000,-.5,.5,1,.7});
        cmd={300,.3,v,0,"base_link",false};check(!badGuard.receive(cmd,300,2),"nonfinite speed");
        cmd.speed_mps=.2;cmd.steering_rad=v;check(!badGuard.receive(cmd,300,2),"nonfinite angle");
        cmd.steering_rad=0;check(!badGuard.receive(cmd,v,2),"nonfinite ROS clock");
        check(!badGuard.receive(cmd,300,v),"nonfinite steady clock");
    }
    // 고정 seed의 재현 가능한 경계/범위 검사. 출력이 있으면 항상 유효한 명령 범위여야 합니다.
    nomad::PlanningCalibration table{true,3721.882617894008,0,.5,.25,.3};
    table.steering_rads={-.4,-.2,0,.1,.4};
    table.steering_servos={0,.2,.5,.7,1};
    nomad::PlanningCommandGuard lookup(table);
    cmd={350,.3,.2,.05,"base_link",false};
    lookup.receive(cmd,350,4);
    auto mapped=lookup.target(350,4);
    check(mapped&&abs(mapped->servo-.6)<1e-9&&abs(mapped->erpm-744.3765235788016)<1e-8,"table interpolation and positive-left raw servo");
    check(abs(table.servoFor(-.1)-.35)<1e-9,"negative-right interpolation");
    check(table.servoFor(0)==.5,"neutral exact");
    check(table.servoFor(-.4)==0&&table.servoFor(.4)==1,"table endpoint exact");
    check(isnan(table.servoFor(.401)),"no extrapolation");
    for(int fault=0;fault<6;++fault){
        auto invalid=table;
        if(fault==0)invalid.steering_servos.pop_back();
        if(fault==1)invalid.steering_rads[1]=invalid.steering_rads[0];
        if(fault==2)invalid.steering_servos[1]=NAN;
        if(fault==3)invalid.steering_servos[2]=.6;
        if(fault==4)invalid.max_rad=.5;
        if(fault==5)invalid.steering_servos[1]=.8;
        check(!nomad::PlanningCommandGuard(invalid).calibrated(),"invalid table rejected");
        check(isnan(invalid.servoFor(.1)),"invalid table cannot be indexed");
    }
    table.verified=false;
    check(!nomad::PlanningCommandGuard(table).calibrated(),"estimated table does not bypass verification");
    check(abs(nomad::referenceSpeedMps(1000)-.26868123008291983)<1e-12,"two wheel speed reference");
    mt19937 random(20261108);uniform_real_distribution<double> speed(-2,3),angle(-3,3),age(-.2,.8);
    int outputs=0;
    for(int i=0;i<20000;++i){
        nomad::PlanningCommandGuard sample({true,1000,-.5,.5,1,.7});
        cmd={400,.3,speed(random),angle(random),"base_link",false};
        const double elapsed=age(random);sample.receive(cmd,400,5);
        const auto out=sample.target(400+elapsed,5+elapsed);
        const bool expected=elapsed>=0&&elapsed<=.3&&cmd.speed_mps>=0&&cmd.speed_mps<=1&&abs(cmd.steering_rad)<=.7;
        check(out.has_value()==expected,"randomized validity oracle");
        if(out){++outputs;check(out->erpm>=0&&out->erpm<=1850&&out->servo>=0&&out->servo<=1,"output bounds invariant");}
    }
    cout<<"20000 randomized cases, "<<outputs<<" valid outputs. Planning contract failures: "<<failures<<'\n';return failures?1:0;
}

#include "nomad_control_cpp/stability_guard.hpp"
#include <iostream>
#include <limits>

int main() {
    int failures=0, checks=0;
    auto check=[&](bool ok,const char* name){++checks;if(!ok){++failures;std::cerr<<name<<'\n';}};
    // 아래 치수는 테스트 전용이며 실제 Slash 보정값이 아닙니다.
    nomad::StabilityConfig c;
    c.geometry_verified=c.frame_verified=true;
    c.track_m=.25;c.wheelbase_m=.335;c.cg_height_m=.15;c.cg_from_rear_m=.17;
    c.max_speed_mps=2;c.max_accel_mps2=2;c.max_tilt_rad=.3;
    nomad::AttitudeSample s{100,0,0,0,0,0,0,9.80665};
    nomad::StabilityGuard guard(c);
    check(!guard.evaluate(1,100,0,0).allowed,"missing IMU blocks");
    check(guard.observe(s,1,100),"valid IMU accepted");
    auto flat=guard.evaluate(1,100,0,0);
    check(flat.allowed && flat.speed_cap_mps==2,"flat envelope");
    auto turn=guard.evaluate(1,100,.1,.5);
    check(turn.allowed && turn.speed_cap_mps<flat.speed_cap_mps,"turn lowers speed cap");
    check(!guard.evaluate(1,100,2,.5).allowed,"actual speed beyond envelope blocks");
    check(!guard.evaluate(1,100,2.1,0).allowed,"straight line overspeed blocks");
    s.stamp_s=100.01;s.roll=.15;s.force_y=9.80665*std::sin(.15);
    check(guard.observe(s,1.01,100.01),"cross slope sample");
    auto slope=guard.evaluate(1.01,100.01,.1,.5);
    check(slope.allowed && slope.speed_cap_mps<turn.speed_cap_mps,"cross slope lowers cornering cap");
    s.stamp_s+=.01;s.roll=-.15;s.force_y=-s.force_y;guard.observe(s,1.02,100.02);
    check(std::abs(guard.evaluate(1.02,100.02,.1,-.5).speed_cap_mps-slope.speed_cap_mps)<1e-8,"left right symmetric worst case");
    s.stamp_s+=.01;s.roll=0;s.force_y=0;s.pitch=.15;s.force_x=9.80665*std::sin(.15);guard.observe(s,1.03,100.03);
    auto pitch=guard.evaluate(1.03,100.03,0,0);
    // 더 높은 무게중심에서는 가속/감속 여유가 더 작아져야 합니다.
    auto tall=c;tall.cg_height_m=.3;
    nomad::StabilityGuard high(tall);high.observe(s,1.03,100.03);
    check(high.evaluate(1.03,100.03,0,0).accel_cap_mps2<pitch.accel_cap_mps2,"high CG lowers acceleration cap");
    check(!guard.evaluate(1.5,100.03,0,0).allowed,"receipt timeout");
    check(!guard.evaluate(1.03,100.5,0,0).allowed,"source timeout");
    check(!guard.evaluate(.9,100.03,0,0).allowed,"steady rollback");
    check(!guard.evaluate(1.03,99,0,0).allowed,"ROS rollback");
    check(!guard.observe(s,1.04,100.04),"duplicate source timestamp invalidates");
    check(!guard.evaluate(1.04,100.04,0,0).allowed,"replay cannot retain previous healthy sample");
    s.stamp_s=100.05;s.roll_rate=.5;guard.observe(s,1.05,100.05);
    check(!guard.evaluate(1.05,100.05,0,0).allowed,"rapid roll blocks");
    s.stamp_s=100.06;s.roll_rate=0;s.roll=.31;guard.observe(s,1.06,100.06);
    check(!guard.evaluate(1.06,100.06,0,0).allowed,"tilt threshold blocks");
    s.stamp_s=100.07;s.roll=0;s.force_z=0;guard.observe(s,1.07,100.07);
    check(!guard.evaluate(1.07,100.07,0,0).allowed,"loss of support blocks");
    s.stamp_s=100.08;s.force_z=9.80665;s.force_y=8;guard.observe(s,1.08,100.08);
    check(!guard.evaluate(1.08,100.08,0,0).allowed,"measured lateral force blocks");
    s.stamp_s=100.09;s.force_y=0;s.roll=std::numeric_limits<double>::quiet_NaN();
    check(!guard.observe(s,1.09,100.09),"NaN rejected");
    auto bad=c;bad.cg_height_m=0;check(!bad.valid(),"zero CG height rejected");
    bad=c;bad.cg_from_rear_m=.4;check(!bad.valid(),"CG outside wheelbase rejected");
    bad=c;bad.cg_lateral_m=.13;check(!bad.valid(),"CG outside track rejected");
    bad=c;bad.geometry_verified=false;check(!bad.valid(),"unverified geometry rejected");
    bad=c;bad.frame_verified=false;check(!bad.valid(),"unverified frame rejected");
    bad=c;bad.margin=.8;check(!bad.valid(),"overly optimistic margin rejected");
    std::cout<<checks<<" stability checks, "<<failures<<" failures\n";
    return failures?1:0;
}

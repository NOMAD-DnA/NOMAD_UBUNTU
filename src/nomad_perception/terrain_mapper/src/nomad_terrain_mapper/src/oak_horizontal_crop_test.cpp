#include <nomad_costmap/oak_horizontal_crop.hpp>
#include <iostream>
#include <limits>
using namespace nomad;
int main() {
 int checks=0;
 auto check=[&](bool ok){++checks;if(!ok)throw std::runtime_error("crop check failed "+std::to_string(checks));};
 M4 vehicle=M4::Identity();vehicle.block<3,3>(0,0)=Eigen::AngleAxisd(0.7,V3::UnitZ()).toRotationMatrix();vehicle.block<3,1>(0,3)=V3(12,-8,3);
 // Public vehicle forward may not be +X (the installed Pi4-aligned frame).
 V3 axis(0,0,-1);
 M4 nonstandard=vehicle;nonstandard.block<3,3>(0,0)=vehicle.block<3,3>(0,0)*Eigen::AngleAxisd(-1.5707963267948966,V3::UnitY()).toRotationMatrix();
 Points local={V3(0,0,0),V3(1,0,0),V3(2,0,0),V3(2.001,0,0),V3(-0.01,0,0),V3(0,2,0),V3(1.5,1.5,0),V3(1,0,100),V3(1,0,-100)};
 for(const auto& transform:{vehicle,nonstandard}) {
   V3 forward = transform.block<3,3>(0,0)*(transform.isApprox(vehicle)?V3::UnitX():axis);
   forward.z()=0;forward.normalize();V3 side(-forward.y(),forward.x(),0);
   Points optical,world;
   for(size_t i=0;i<local.size();++i){optical.emplace_back(double(i),22,33);world.push_back(transform.block<3,1>(0,3)+forward*local[i].x()+side*local[i].y()+V3(0,0,local[i].z()));}
   auto removed=crop_oak_horizontal(optical,world,transform,transform.isApprox(vehicle)?V3::UnitX():axis,2);
   check(removed==3);check(world.size()==6);check(optical.size()==6);
   int ids[]={0,1,2,5,7,8};for(size_t i=0;i<6;++i)check(optical[i].x()==ids[i]);
   check(std::abs(world[4].z()-(transform(2,3)+100))<1e-9);
 }
 // Capture-time world<-IMU times inverse fixed mount recovers vehicle including offset.
 M4 mount=M4::Identity();mount.block<3,3>(0,0)=Eigen::AngleAxisd(.4,V3::UnitZ()).toRotationMatrix();mount.block<3,1>(0,3)=V3(.6,-.3,.2);
 check((vehicle*mount*mount.inverse()).isApprox(vehicle));
 Points empty;check(crop_oak_horizontal(empty,empty,vehicle,V3::UnitX(),2)==0);
 for(double range:{0.,-1.,31.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){bool failed=false;try{crop_oak_horizontal(empty,empty,vehicle,V3::UnitX(),range);}catch(...){failed=true;}check(failed);}
 for(double range:{.1,1.,3.,6.,30.}){Points o={V3(9,8,7)},w={V3(range,0,555)};check(crop_oak_horizontal(o,w,M4::Identity(),V3::UnitX(),range)==0);w={V3(range+.001,0,555)};check(crop_oak_horizontal(o,w,M4::Identity(),V3::UnitX(),range)==1);}
 Points o={V3(1,2,3)},w={V3(std::numeric_limits<double>::quiet_NaN(),0,0)};check(crop_oak_horizontal(o,w,M4::Identity(),V3::UnitX(),2)==1);
 bool failed=false;try{Points a={V3::Zero()},b;crop_oak_horizontal(a,b,vehicle,V3::UnitX(),2);}catch(...){failed=true;}check(failed);
 std::cout<<checks<<" OAK horizontal crop checks passed\n";
}

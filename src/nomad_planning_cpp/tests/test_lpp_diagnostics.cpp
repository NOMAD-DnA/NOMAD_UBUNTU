#include "nomad_planning_cpp/lpp_diagnostics.hpp"
#include <iostream>
#include <stdexcept>
using Log = nomad_planning_cpp::LppDiagnostics;
using Event = Log::Event;
void expect(Event actual, Event wanted) {
  if (actual != wanted) throw std::runtime_error("incorrect LPP diagnostic transition");
}
int main() {
  Log log;
  const std::string rpp="no collision-free RPP motion; recovery disabled";
  expect(log.update(false,false,false,0,0,"WAIT","NO_GOAL"),Event::none);
  expect(log.update(true,false,false,1,0,"READY",rpp),Event::pause);
  for(int n=0;n<200;++n)expect(log.update(true,false,false,1,0,"READY",rpp),Event::none);
  for(const auto* reason:{"observed-space safety stop","observation boundary",
                         "TRAJECTORY_FOOTPRINT_OUTSIDE_MAP","ODOMETRY_STALE"})
    expect(log.update(true,false,false,1,0,"BLOCKED",reason),Event::pause);
  expect(log.update(true,false,true,1,0,"READY","tracking"),Event::recovered);
  expect(log.update(true,false,true,1,0,"READY","tracking"),Event::none);
  expect(log.update(true,false,false,1,0,"READY",rpp),Event::pause);
  expect(log.update(true,false,false,2,0,"READY",rpp),Event::pause);
  expect(log.update(true,false,false,2,1,"READY",rpp),Event::pause);
  expect(log.update(true,true,false,2,1,"ARRIVED","goal reached"),Event::none);
  expect(log.update(true,false,true,3,1,"READY","tracking"),Event::none);
  expect(log.update(true,false,false,4,1,"STALE","ODOMETRY_STALE"),Event::pause);
  expect(log.update(false,false,false,5,1,"WAIT","NO_GOAL"),Event::none);
  expect(log.update(true,false,false,6,1,"STALE","ODOMETRY_STALE"),Event::pause);
  expect(log.update(true,false,false,7,1,"BLOCKED",rpp+" | rpp=0.800:GEOMETRY_STEERING_LIMIT"),Event::pause);
  for(int n=0;n<200;++n)expect(log.update(true,false,false,7,1,"BLOCKED",rpp+" | rpp=0.800:GEOMETRY_STEERING_LIMIT"),Event::none);
  expect(log.update(true,false,false,7,1,"BLOCKED",rpp+" | rpp=0.800:BRAKING_INSUFFICIENT"),Event::pause);
  std::cout << "LPP diagnostic transitions passed\n";
}

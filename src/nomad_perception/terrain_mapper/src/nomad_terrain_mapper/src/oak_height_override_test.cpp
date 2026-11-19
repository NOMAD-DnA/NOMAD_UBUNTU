#include <nomad_costmap/oak_height_override.hpp>
#include <iostream>
#include <limits>
using namespace nomad;
int main() {
  int checks=0;
  auto check=[&](bool ok){++checks;if(!ok)throw std::runtime_error("height check failed "+std::to_string(checks));};
  Json original={{"oak_lidar_max_abs_z_difference_m",.12},{"oak_lidar_xy_radius_m",.12},{"oak_lidar_height_gate_enabled",true}};
  auto unchanged=original; apply_oak_height_override(Json::object(),unchanged);check(unchanged==original);
  for(double cm:{.1,5.,12.,20.,100.}) {
    auto s=original;apply_oak_height_override({{"oak_lidar_max_abs_z_difference_cm",cm}},s);
    check(std::abs(s.at("oak_lidar_max_abs_z_difference_m").get<double>()-cm/100.)<1e-12);
    check(s.at("oak_lidar_xy_radius_m")==original.at("oak_lidar_xy_radius_m"));
    check(s.at("oak_lidar_height_gate_enabled")==true);
    // Use the installed, unchanged Ground height gate, not a duplicate comparison.
    auto remaining=[&](double dz,double xy,bool reference=true) {
      Config cfg;cfg.oak_lidar_height_gate=true;cfg.oak_lidar_xy_radius=.12;
      cfg.oak_lidar_z_reject=s.at("oak_lidar_max_abs_z_difference_m");
      Ground ground(cfg,Mounts{});
      if(reference)ground.samples[{0,0}].sources[0].push_back(Capture{1,Points{V3(0,0,0)}});
      ground.samples[{0,0}].sources[1].push_back(Capture{1,Points{V3(xy,0,dz)}});
      ground.snapshot();
      size_t n=0,lidar=0;
      for(const auto&cell:ground.samples){for(const auto&c:cell.second.sources[1])n+=c.points.size();for(const auto&c:cell.second.sources[0])lidar+=c.points.size();}
      check(lidar==(reference?1:0));return n;
    };
    const double m=cm/100.;
    check(remaining(m-.00001,0)==1);
    check(remaining(m,0)==0);
    check(remaining(m+.00001,0)==0);
    check(remaining(-m,0)==0);
    check(remaining(m+.01,.121)==1); // outside XY reference radius
    check(remaining(m+.01,0,false)==1); // absent reference is not rejection
  }
  for(double cm:{0.,-1.,100.1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    bool failed=false;try{auto s=original;apply_oak_height_override({{"oak_lidar_max_abs_z_difference_cm",cm}},s);}catch(...){failed=true;}check(failed);
  }
  std::cout<<checks<<" OAK height override and real Ground gate checks passed\n";
}

#include "nomad_costmap/indoor.hpp"
#include <fstream>
#include <iostream>
#include <cfenv>

// Offline JSONL replay of already transformed LiDAR returns. No ROS or sensors.
int main(int argc,char** argv) {
  std::fesetround(FE_TONEAREST);
  try {
    nomad::require(argc==2,"Usage: indoor_probe indoor.json < events.jsonl");
    std::ifstream file(argv[1]);nomad::require(bool(file),"Cannot read config");
    nomad::Json config;file>>config;auto o=nomad::IndoorOptions::load(config);
    nomad::IndoorMap map(o);nomad::Pose pose;pose.stamp=1;pose.p.z()=o.fixed_z;
    pose.q=Eigen::Quaterniond::FromTwoVectors(o.camera_forward_axis_imu,nomad::V3::UnitX());
    nomad::M4 mount=nomad::M4::Identity();bool initialized=false;
    std::string line;
    while(std::getline(std::cin,line)) {
      if(line.empty())continue;
      const auto event=nomad::Json::parse(line);
      if(event.contains("base_pose")) {
        pose=nomad::indoor_imu_pose(nomad::pose_from_json(event.at("base_pose")),mount,o.fixed_z);
      }
      nomad::Frame frame;frame.source=0;frame.stamp=event.at("stamp_ns");
      if(!initialized) {pose.stamp=frame.stamp;map.initialize(pose,mount);initialized=true;}
      for(const auto& ray:event.at("returns")) {
        const auto p=ray.at("point"),a=ray.at("origin");
        frame.points.emplace_back(p[0],p[1],p[2]);frame.ray_origins.emplace_back(a[0],a[1],a[2]);
        frame.stamps.push_back(ray.value("stamp_ns",frame.stamp+int64_t(frame.stamps.size())));
        auto point_pose=pose;point_pose.stamp=frame.stamps.back();frame.capture_poses.push_back(point_pose);
      }
      map.ingest(frame);pose.stamp=map.capture;
      const auto s=map.snapshot(pose,mount);
      auto floats=[](const auto& v){nomad::Json a=nomad::Json::array();for(auto x:v)a.push_back(nomad::num(x));return a;};
      std::cout<<nomad::Json{{"capture_ns",s.capture},{"resolution_m",s.resolution},{"origin",{s.origin_x,s.origin_y}},
        {"width",s.width},{"height",s.height},{"slope_deg",floats(s.slope_deg)},{"slope_cost",floats(s.slope_cost)},
        {"terrain_cost",floats(s.terrain_cost)},{"global",s.global},{"local",s.local},{"provenance",map.provenance()}}.dump()<<'\n';
    }
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}

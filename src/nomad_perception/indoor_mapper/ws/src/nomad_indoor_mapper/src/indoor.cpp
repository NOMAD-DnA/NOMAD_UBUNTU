#include "nomad_costmap/indoor.hpp"
#include <cmath>

namespace nomad {
IndoorOptions IndoorOptions::load(const Json& j) {
  require(j.is_object(), "Indoor config must be an object");
  const std::set<std::string> keys{"build_id", "floor_z_cm", "vio_fixed_z_m", "resolution_m",
    "map_length_m", "cost_area", "max_map_cells", "map_keep_radius_m", "startup_radius_m",
    "slope_score_per_deg", "slope_block_deg", "initialization_stationary_hold_s", "max_pending_frames",
    "lock_obstacle_cells", "lidar_half_fov_deg", "max_pose_gap_ms", "imu_frame", "T_imu_from_lidar", "camera_forward_axis_imu", "slope_forward_axis_base_link", "moving_free_region"};
  for (auto it=j.begin(); it!=j.end(); ++it)
    require(keys.count(it.key()) || it.key().rfind("_",0)==0, "Unknown indoor setting: " + it.key());
  IndoorOptions o;
  auto number = [&](const char* key, double fallback) {
    const auto value = j.value(key, Json(fallback));
    require(value.is_number(), std::string(key)+" must be numeric");
    const double n=value.get<double>();
    require(std::isfinite(n), std::string(key)+" must be finite");
    return n;
  };
  if(j.contains("lock_obstacle_cells")) {
    require(j.at("lock_obstacle_cells").is_boolean(), "lock_obstacle_cells must be boolean");
    o.lock_obstacle_cells=j.at("lock_obstacle_cells").get<bool>();
  }
  o.half_fov_deg=number("lidar_half_fov_deg",80.);
  require(o.half_fov_deg>=10. && o.half_fov_deg<=100., "LiDAR half FOV outside 10..100deg");
  o.floor_z=number("floor_z_cm",5.)/100.;
  o.fixed_z=number("vio_fixed_z_m",0.);
  o.stationary_hold=number("initialization_stationary_hold_s",2.);
  require(o.stationary_hold>=0. && o.stationary_hold<=60., "Stationary hold outside 0..60s");
  const double gap=number("max_pose_gap_ms",200.);
  require(gap>0. && gap<=1000., "Pose gap outside (0,1000]ms");
  o.pose_gap_ns=nearest_even(gap*1e6);
  const double pending=number("max_pending_frames",80.);
  require(pending>=1 && pending<=1000 && pending==std::floor(pending), "Invalid queue capacity");
  o.pending_limit=size_t(pending);
  o.imu_frame=j.value("imu_frame",o.imu_frame);
  require(!o.imu_frame.empty(), "Empty VIO IMU frame");
  o.imu_lidar=rigid(j.at("T_imu_from_lidar"));
  if(j.contains("camera_forward_axis_imu")) {
    const auto& a=j.at("camera_forward_axis_imu");
    require(a.is_array() && a.size()==3,"camera_forward_axis_imu requires three values");
    o.camera_forward_axis_imu=V3(a[0].get<double>(),a[1].get<double>(),a[2].get<double>());
  }
  require(o.camera_forward_axis_imu.allFinite() && o.camera_forward_axis_imu.norm()>1e-8,"Invalid camera forward axis");
  o.camera_forward_axis_imu.normalize();
  auto& c=o.fusion;
  c.resolution=number("resolution_m",.125);
  require(c.resolution>=.01 && c.resolution<=1., "Resolution outside .01..1m");
  c.map_length=number("map_length_m",10.);
  require(c.map_length==0. || (c.map_length>=4. && c.map_length<=100.), "Map length must be 0 or 4..100m");
  require(c.map_length==0. || std::pow(std::llround(c.map_length/c.resolution),2)<=4000000., "Map exceeds 4 million cells");
  const auto area=j.value("cost_area",std::string("known_cells"));
  require(area=="known_cells" || area=="map_window", "Invalid cost area");
  c.cost_known_cells=area=="known_cells";
  const double cap=number("max_map_cells",0);
  require(cap==std::floor(cap) && (cap==0 || (cap>=1000 && cap<=1000000)), "Invalid cell cap");
  c.max_map_cells=size_t(cap);
  c.map_keep_radius=number("map_keep_radius_m",6.);
  require(c.map_keep_radius>=1. && c.map_keep_radius<=30., "Invalid keep radius");
  c.slope_score_per_deg=number("slope_score_per_deg",8.);
  c.slope_block_deg=number("slope_block_deg",10.);
  require(c.slope_score_per_deg>0 && c.slope_score_per_deg<=100 && c.slope_block_deg>0 &&
    c.slope_block_deg<=90 && c.slope_score_per_deg*c.slope_block_deg<=100.+1e-9, "Invalid fusion coefficients");
  const double radius=number("startup_radius_m",1.5);
  require(radius>0 && radius<=30., "Startup radius outside (0,30]m");
  c.startup_free_region_enabled=true;
  // The legacy fusion's protected-class selector is generalized to every height obstacle.
  // No person tracking/expiry/semantic projection is instantiated in this mapper.
  c.person_class_id=10;
  c.person_ray_restore=false;
  c.startup_person_cost_overlay=false;
  c.semantic={{"startup_free_region",{{"enabled",true},{"radius_m",radius}}}};
  auto moving=j.value("moving_free_region",Json::object());
  const std::set<std::string> moving_keys{"enabled","radius_m","rear_length_m","rear_half_width_m","driven_path_memory","driven_front_m"};
  require(moving.is_object(),"moving_free_region must be an object");
  for(auto it=moving.begin();it!=moving.end();++it) require(moving_keys.count(it.key()),"Unknown moving region setting");
  moving["override_policy"]="semantic_zero_measured_slope_kept_except_person";
  c.semantic["moving_front_free_region"]=moving;
  if(j.contains("slope_forward_axis_base_link")) {
    auto axis=j.at("slope_forward_axis_base_link");
    require(axis.is_array() && axis.size()==3,"Expected three forward-axis values");
    c.slope_forward_axis_base_link=V3(axis[0],axis[1],axis[2]);
    require(c.slope_forward_axis_base_link.allFinite() && c.slope_forward_axis_base_link.norm()>1e-8,"Invalid forward axis");
  }
  MovingFrontFreeRegion validate_moving(c);
  return o;
}

Pose indoor_imu_pose(Pose base, const M4& base_imu, double z) {
  require(std::isfinite(z),"Nonfinite fixed Z");
  base.p.z()=z; // Incoming Z is ignored, including its numerical value.
  base=checked_pose(base);
  rigid(matrix_json(base_imu));
  Pose imu=base;
  imu.p=base.q*base_imu.block<3,1>(0,3)+base.p;
  imu.q=Eigen::Quaterniond(base.q.toRotationMatrix()*base_imu.block<3,3>(0,0)).normalized();
  return imu;
}

ScanPair indoor_scan_pair(const ScanInput& scan,const CloudInput& cloud) {
  require(scan.stamp_ns>0 && cloud.stamp_ns==scan.stamp_ns,"Scan/cloud stamps differ");
  require(!cloud.points.empty() && cloud.points.size()==cloud.offsets.size() && cloud.points.size()<=1000000,"Invalid point/time buffers");
  require(std::isfinite(scan.angle_min) && std::isfinite(scan.angle_increment) && scan.angle_increment>0 &&
    std::isfinite(scan.range_min) && std::isfinite(scan.range_max) && scan.range_min>=0 && scan.range_max>scan.range_min &&
    !scan.ranges.empty() && scan.ranges.size()<=1000000,"Invalid scan bounds");
  ScanPair pair; pair.stamp_ns=scan.stamp_ns;
  double previous=-1.;
  for(size_t i=0;i<cloud.points.size();++i) {
    const auto& p=cloud.points[i]; const double dt=cloud.offsets[i];
    require(p.allFinite() && std::isfinite(dt) && dt>=0 && dt<=.3 && dt>previous,"Invalid per-return data");
    previous=dt;
    const auto bin=int64_t(std::ceil((std::atan2(p.y(),p.x())-scan.angle_min)/scan.angle_increment));
    if(bin<0 || bin>=int64_t(scan.ranges.size())) continue;
    const double range=scan.ranges[size_t(bin)];
    if(!std::isfinite(range) || range<scan.range_min || range>scan.range_max ||
      std::abs(range-p.head<2>().norm())>=.003) continue;
    // Keep measured XYZ. The former camera-facing hemisphere crop is intentionally absent.
    pair.points_lidar_m.push_back(p); pair.point_time_offsets_s.push_back(dt); pair.bin_indices.push_back(bin);
  }
  require(!pair.points_lidar_m.empty(),"No matched LiDAR returns");
  return pair;
}

bool ray_crosses_cell(const V3& origin,const V3& hit,Key key,double resolution) {
  if(key==key_of(hit,resolution)) return false;
  const Eigen::Vector2d a=origin.head<2>(),delta=hit.head<2>()-a;
  if(delta.squaredNorm()<=1e-20) return false;
  double enter=0.,leave=1.;
  for(int axis=0;axis<2;++axis) {
    const double lower=double(axis?key.second:key.first)*resolution,upper=lower+resolution;
    if(std::abs(delta[axis])<=1e-15) {
      if(a[axis]<lower || a[axis]>=upper) return false;
    } else {
      double near=(lower-a[axis])/delta[axis],far=(upper-a[axis])/delta[axis];
      if(near>far) std::swap(near,far);
      enter=std::max(enter,near);leave=std::min(leave,far);
      if(leave<=enter) return false;
    }
  }
  return leave<1.-1e-12 && leave>0.;
}

void IndoorMap::initialize(const Pose& imu,const M4& mount) {
  rigid(matrix_json(mount));
  require(!reference_set || (mount-base_imu).cwiseAbs().maxCoeff()<=1e-9,"Vehicle/IMU mount changed");
  require(startup.initialize_from_imu(imu,mount),"Invalid startup anchor");
  base_imu=mount;reference_set=true;
}

void IndoorMap::ingest(const Frame& f) {
  require(startup.initialized() && reference_set,"Map must be initialized");
  require(f.source==0 && f.stamp>last_scan && !f.points.empty() && f.points.size()==f.stamps.size() &&
    f.ray_origins.size()==f.points.size() && f.capture_poses.size()==f.points.size(),"Invalid/newer per-return LiDAR frame required");
  int64_t frame_capture=f.stamp;
  for(size_t i=0;i<f.points.size();++i) {
    require(f.points[i].allFinite() && f.ray_origins[i].allFinite() && f.stamps[i]>=f.stamp &&
      f.stamps[i]-f.stamp<=300000000 && (i==0 || f.stamps[i]>f.stamps[i-1]),"Invalid frame geometry/timestamps");
    checked_pose(f.capture_poses[i]);
    require(f.capture_poses[i].stamp==f.stamps[i],"Per-return pose must use the actual acquisition time");
    require(f.points[i].head<2>().cwiseAbs().maxCoeff()/options.fusion.resolution<1e12,"Coordinates outside map range");
    frame_capture=std::max(frame_capture,f.stamps[i]);
  }
  std::map<Key,int64_t> endpoints,passes;
  uint64_t used=0,rejected_fov=0;
  const double cosine=std::cos(options.half_fov_deg*pi/180.);
  // Collect a complete scan before applying state. Its endpoints always win over its passes.
  for(size_t i=0;i<f.points.size();++i) {
    const Pose& pose=f.capture_poses[i];
    const M3 body_rotation=pose.q.toRotationMatrix()*base_imu.block<3,3>(0,0).transpose();
    const V3 vehicle=pose.p-body_rotation*base_imu.block<3,1>(0,3);
    const V3 forward=pose.q*options.camera_forward_axis_imu;
    const Eigen::Vector2d direction=(f.points[i]-f.ray_origins[i]).head<2>();
    const double heading_norm=forward.head<2>().norm(),ray_norm=direction.norm();
    require(vehicle.allFinite() && heading_norm>1e-8,"Camera forward has no valid XY projection");
    if(ray_norm<=1e-10 || direction.dot(forward.head<2>())/(ray_norm*heading_norm)<cosine-1e-12) {
      ++rejected_fov;continue;
    }
    ++used;
    const Key endpoint=key_of(f.points[i],options.fusion.resolution);
    const bool occupied=(f.points[i]-vehicle).head<2>().squaredNorm()<=options.ray_range_m*options.ray_range_m+1e-12;
    if(occupied) endpoints[endpoint]=std::max(endpoints[endpoint],f.stamps[i]);
    for(Key k:trace_ray_cells(f.ray_origins[i],f.points[i],vehicle,options.fusion.resolution,options.ray_range_m)) {
      if(occupied && k==endpoint) continue;
      passes[k]=std::max(passes[k],f.stamps[i]);
    }
  }
  // Discard replayed or chronologically older endpoints before resolving scan precedence.
  for(auto it=endpoints.begin();it!=endpoints.end();) {
    const auto old=cells.find(it->first);
    if(old!=cells.end() && (it->second<=old->second.cleared ||
      it->second<std::max(old->second.last_point,old->second.ray_observed))) it=endpoints.erase(it);
    else ++it;
  }
  for(const auto& [k,t]:passes) {
    if(endpoints.count(k)) continue;
    auto old=cells.find(k);
    if(old!=cells.end() && (t<=old->second.cleared || t<std::max(old->second.last_point,old->second.ray_observed))) continue;
    auto& cell=cells[k];
    // A ray still reobserves the cell; a latched obstacle is never cleared by it.
    if(options.lock_obstacle_cells && cell.obstacle) {cell.ray_observed=t;continue;}
    if(cell.obstacle)++rays_cleared;
    cell.obstacle=false;cell.ray_observed=t;cell.cleared=frame_capture;
    cell.source="lidar_xy_ray_paved";
  }
  for(const auto& [k,t]:endpoints) {
    auto& cell=cells[k];cell.obstacle=true;cell.last_point=t;cell.last_obstacle=t;
    cell.source="lidar_endpoint_obstacle_z_ignored";
  }
  rays_used+=used;rays_outside_fov+=rejected_fov;
  last_scan=f.stamp;capture=std::max(capture,frame_capture);++frames;
}

void IndoorMap::prune(const MapWindow& w) {
  for(auto it=cells.begin();it!=cells.end();) it=w.contains(it->first)?std::next(it):cells.erase(it);
  std::set<Key> keys;for(const auto& [k,c]:cells) { (void)c;keys.insert(k); }
  const auto drop=cells_over_cap(keys,w,[&](Key k){const auto& c=cells.at(k);return std::max(c.last_point,c.cleared);});
  for(Key k:drop) cells.erase(k);
  // Startup priors outside a rolling map are retired, not repeatedly re-seeded.
  auto priors=startup.cells();
  for(Key k:priors) if(!w.contains(k)) {startup.retire_slope(k);startup.retire_semantic(k);}
  moving.prune(w);
}

CostSnapshot IndoorMap::snapshot(const Pose& imu,const M4& base_imu) {
  const M3 rotation=imu.q.toRotationMatrix()*base_imu.block<3,3>(0,0).transpose();
  const V3 position=imu.p-rotation*base_imu.block<3,1>(0,3);
  const MapWindow window=make_window(options.fusion,position.head<2>());
  prune(window);
  require(moving.update_from_imu(imu,base_imu),"Invalid moving region pose");
  SlopeSnapshot slopes; SemanticSnapshot sem;
  slopes.capture=capture;sem.stamp=capture;
  for(const auto& [k,c]:cells) {
    const int64_t effective=std::max(c.last_point,c.ray_observed);
    SlopeRow s;s.ix=k.first;s.iy=k.second;s.observed=effective;
    if(!c.obstacle) {s.angle=0.;s.tilt=0.;} // Assumed flat, no TLS, height or confidence invented.
    slopes.rows.push_back(s);
    SemanticRow r;r.ix=k.first;r.iy=k.second;r.class_id=c.obstacle?10:6;r.cost=c.obstacle?100:0;
    r.last_geometry_ns=c.last_point;
    // Legacy category-time field carries real LiDAR hit/clear time; camera fields stay zero.
    r.last_semantic_ns=effective;sem.rows.push_back(r);
  }
  // Keep the original 70/30 fusion and hard block. Observed local costs no longer expire by age.
  return build_cost(slopes,sem,options.fusion,&startup,&moving,options.fusion.cost_known_cells?nullptr:&window);
}

Json IndoorMap::provenance() const {
  Json rows=Json::array();
  for(const auto& [k,c]:cells) rows.push_back({{"ix",k.first},{"iy",k.second},{"obstacle",c.obstacle},
    {"last_lidar_point_ns",c.last_point},{"last_obstacle_point_ns",c.last_obstacle},
    {"ray_clear_ns",options.lock_obstacle_cells && c.obstacle?c.cleared:c.ray_observed},{"last_ray_observation_ns",c.ray_observed},{"replay_barrier_ns",c.cleared},{"source",c.source}});
  return {{"build_id",indoor_build_id},{"legacy_floor_z_ignored",true},{"floor_z_m",options.floor_z},{"vio_fixed_z_m",options.fixed_z},
    {"slope_source","user_declared_flat_assumption"},{"camera_observations",0},
    {"frames",frames},{"rays_cleared",rays_cleared},{"rays_used",rays_used},{"rays_outside_fov",rays_outside_fov},
    {"lock_obstacle_cells",options.lock_obstacle_cells},{"horizontal_fov_deg",2.*options.half_fov_deg},{"lidar_half_fov_deg",options.half_fov_deg},{"ray_range_m",2.},{"cell_age_expiry",false},{"startup",startup.metadata()},{"cells",rows}};
}
}

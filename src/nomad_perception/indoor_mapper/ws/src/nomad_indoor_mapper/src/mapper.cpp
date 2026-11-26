// Indoor variant of nomad_terrain_mapper. No camera, label or factory subscriptions.
#include "nomad_costmap/indoor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nomad_interfaces/msg/module_status.hpp>
#include <nomad_interfaces/msg/planning_cost_layers.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <cfenv>
#include <deque>
#include <iostream>

namespace nomad {
using Status=nomad_interfaces::msg::ModuleStatus;
using Layers=nomad_interfaces::msg::PlanningCostLayers;
using Grid=nav_msgs::msg::OccupancyGrid;
using Header=std_msgs::msg::Header;
static int64_t stamp(const Header& h) {return int64_t(h.stamp.sec)*1000000000+h.stamp.nanosec;}
static int64_t steady_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void set_stamp(Header& h,int64_t ns) {h.frame_id="odom";h.stamp.sec=ns/1000000000;h.stamp.nanosec=ns%1000000000;}
static rclcpp::QoS map_qos() {return rclcpp::QoS(1).reliable().transient_local();}
static M4 pose_matrix(const Pose& p) {M4 m=M4::Identity();m.block<3,3>(0,0)=p.q.toRotationMatrix();m.block<3,1>(0,3)=p.p;return m;}

class IndoorMapper:public rclcpp::Node {
  IndoorOptions options;
  IndoorMap map;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  std::optional<M4> fixed;
  std::vector<Pose> poses;
  Status localization;
  int64_t localization_wall=0,pose_wall=0,lidar_wall=0,lidar_capture=0;
  int64_t last_clock=0,still_since=0,last_publish=0,last_map_wall=0;
  bool initialized=false,valid=false,clock_fault=false;
  std::string reason="Waiting for VIO and LiDAR",fatal;
  struct PairSlot {std::optional<ScanInput> scan;std::optional<CloudInput> cloud;};
  std::map<int64_t,PairSlot> pairs;
  std::deque<int64_t> pair_order;
  struct Pending {ScanPair pair;int64_t receipt;};
  std::deque<Pending> pending;
  int64_t recovery_after=0;
  uint64_t rejected=0,dropped=0;
  int fast_poses=0;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::array<rclcpp::Publisher<Grid>::SharedPtr,3> grids;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr,4> images;
  rclcpp::Publisher<Layers>::SharedPtr layers,diagnostic_layers;
  rclcpp::Publisher<Status>::SharedPtr status;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  rclcpp::TimerBase::SharedPtr timer;

  bool localization_ready() const {
    const int64_t now=this->now().nanoseconds(),wall=steady_ns(),ns=stamp(localization.header);
    const int64_t duration=int64_t(localization.valid_for.sec)*1000000000+localization.valid_for.nanosec;
    return localization_wall>0 && localization.header.frame_id=="odom" && localization.state==Status::READY &&
      duration>0 && duration<=1000000000 && now-ns>=-50000000 && now-ns<=duration && wall-localization_wall<=1000000000;
  }
  bool vio_ready() const {
    if(poses.empty() || !fixed || !localization_ready()) return false;
    const auto age=this->now().nanoseconds()-poses.back().stamp;
    return age>=-50000000 && age<=600000000 && steady_ns()-pose_wall<=1000000000;
  }
  void on_pose(const nav_msgs::msg::Odometry& m) {
    try {
      require(localization_ready(),"Localization not READY");
      const int64_t ns=stamp(m.header),now=this->now().nanoseconds();
      require(m.header.frame_id=="odom" && m.child_frame_id=="base_link" && ns>0 &&
        (poses.empty() || ns>poses.back().stamp) && now-ns>=-50000000 && now-ns<=600000000,"Invalid/stale/reordered VIO pose");
      const auto tf=tf_buffer.lookupTransform("base_link",options.imu_frame,tf2::TimePointZero);
      require(tf.header.frame_id=="base_link" && tf.child_frame_id==options.imu_frame && stamp(tf.header)==0,"Expected static base<-IMU transform");
      const auto& t=tf.transform.translation;const auto& q=tf.transform.rotation;
      Pose mount_pose;mount_pose.stamp=1;mount_pose.p=V3(t.x,t.y,t.z);mount_pose.q=Eigen::Quaterniond(q.w,q.x,q.y,q.z);
      const M4 mount=pose_matrix(checked_pose(mount_pose));
      if(fixed && (mount-*fixed).cwiseAbs().maxCoeff()>1e-9) {fatal="VIO static transform changed; restart with an empty map";return;}
      const auto& p=m.pose.pose.position;const auto& r=m.pose.pose.orientation;
      Pose base;base.stamp=ns;base.p=V3(p.x,p.y,options.fixed_z);base.q=Eigen::Quaterniond(r.w,r.x,r.y,r.z);
      const Pose imu=indoor_imu_pose(base,mount,options.fixed_z);
      if(!poses.empty()) {
        const double speed=(imu.p-poses.back().p).norm()/((ns-poses.back().stamp)/1e9);
        fast_poses=speed>10.?fast_poses+1:0;
        if(fast_poses>=5) {fatal="VIO divergence: speed exceeded 10m/s for 5 samples";return;}
      }
      poses.push_back(imu);if(poses.size()>10000) poses.erase(poses.begin(),poses.end()-10000);
      fixed=mount;pose_wall=steady_ns();
    } catch(const std::exception& e) {++rejected;reason=e.what();}
  }
  void join(int64_t ns) {
    auto& slot=pairs.at(ns);
    if(slot.scan && slot.cloud) {
      try {
        auto pair=indoor_scan_pair(*slot.scan,*slot.cloud);
        if(pending.size()>=options.pending_limit) {
          dropped+=pending.size();pending.clear();recovery_after=this->now().nanoseconds();valid=false;
          reason="LiDAR queue overload: waiting for a newly captured scan";
        }
        pending.push_back({std::move(pair),steady_ns()});lidar_wall=steady_ns();lidar_capture=ns;
      } catch(const std::exception& e) {++rejected;reason=e.what();}
      pairs.erase(ns);pair_order.erase(std::remove(pair_order.begin(),pair_order.end(),ns),pair_order.end());
    }
    while(pair_order.size()>30) {pairs.erase(pair_order.front());pair_order.pop_front();++dropped;}
  }
  bool header_ok(const Header& h) const {return h.frame_id=="g2_laser" && stamp(h)>0;}
  PairSlot& slot(int64_t ns) {if(!pairs.count(ns)) pair_order.push_back(ns);return pairs[ns];}
  void on_scan(const sensor_msgs::msg::LaserScan& m) {
    if(!header_ok(m.header) || m.ranges.size()>1000000) {++rejected;return;}
    ScanInput s;s.stamp_ns=stamp(m.header);s.ranges.assign(m.ranges.begin(),m.ranges.end());
    s.angle_min=m.angle_min;s.angle_increment=m.angle_increment;s.range_min=m.range_min;s.range_max=m.range_max;
    slot(s.stamp_ns).scan=std::move(s);join(stamp(m.header));
  }
  void on_cloud(const sensor_msgs::msg::PointCloud& m) {
    if(!header_ok(m.header) || m.points.size()>1000000) {++rejected;return;}
    CloudInput c;c.stamp_ns=stamp(m.header);
    for(const auto& p:m.points)c.points.emplace_back(p.x,p.y,p.z);
    for(const auto& channel:m.channels)if(channel.name=="stamps")c.offsets.assign(channel.values.begin(),channel.values.end());
    slot(c.stamp_ns).cloud=std::move(c);join(stamp(m.header));
  }
  bool stationary() const {
    if(poses.size()<10)return false;
    const auto& last=poses.back();
    auto first=std::lower_bound(poses.begin(),poses.end(),last.stamp-2000000000,[](const Pose& p,int64_t ns){return p.stamp<ns;});
    if(poses.end()-first<10 || last.stamp-first->stamp<1800000000)return false;
    for(auto it=first;it!=poses.end();++it)
      if((last.p-it->p).norm()>.03 || Eigen::AngleAxisd(last.q.conjugate()*it->q).angle()>2.*pi/180.)return false;
    return true;
  }
  Grid grid(const CostSnapshot& s,const std::vector<int8_t>& v) {
    Grid m;set_stamp(m.header,s.capture);m.info.resolution=s.resolution;m.info.width=s.width;m.info.height=s.height;
    m.info.origin.position.x=s.origin_x;m.info.origin.position.y=s.origin_y;m.info.origin.orientation.w=1.;m.data=v;return m;
  }
  void publish(const CostSnapshot& s) {
    const int64_t now=this->now().nanoseconds();
    require(s.capture>=last_publish && now-s.capture>=-50000000 && now-s.capture<=2000000000,"Stale/future map capture");
    Layers bundle;bundle.grid=grid(s,s.global);bundle.slope_cost=s.slope_cost;bundle.terrain_cost=s.terrain_cost;
    const std::array<const std::vector<float>*,4> values{&s.slope_score,&s.semantic_score,&s.slope_deg,&s.slope_magnitude_deg};
    for(size_t i=0;i<images.size();++i) {
      sensor_msgs::msg::Image image;image.header=bundle.grid.header;image.height=s.height;image.width=s.width;
      image.encoding="32FC1";image.step=image.width*4;image.data.resize(size_t(image.step)*image.height);
      std::memcpy(image.data.data(),values[i]->data(),image.data.size());images[i]->publish(image);
    }
    diagnostic_layers->publish(bundle);layers->publish(bundle);
    grids[0]->publish(bundle.grid);grids[1]->publish(grid(s,s.local));grids[2]->publish(grid(s,s.support));
    diagnostic_msgs::msg::DiagnosticArray d;d.header=bundle.grid.header;
    diagnostic_msgs::msg::DiagnosticStatus record;record.name="semantic_validation/timing";record.hardware_id="indoor_lidar_only";
    record.message="flat_assumption; no measured slope; camera support=0; actual LiDAR acquisition stamps";
    for(const auto& kv:std::map<std::string,std::string>{{"build_id",indoor_build_id},
      {"floor_z_m",std::to_string(options.floor_z)},{"vio_fixed_z_m",std::to_string(options.fixed_z)},
      {"frames",std::to_string(map.frames)},{"rays_cleared",std::to_string(map.rays_cleared)},
      {"rejected",std::to_string(rejected)},{"dropped",std::to_string(dropped)}}) {
      diagnostic_msgs::msg::KeyValue value;value.key=kv.first;value.value=kv.second;record.values.push_back(value);
    }
    d.status.push_back(record);diagnostics->publish(d);
    last_publish=s.capture;last_map_wall=steady_ns();valid=true;reason="Validated indoor LiDAR snapshot";
  }
  void tick() {
    const int64_t now=this->now().nanoseconds(),wall=steady_ns();
    if(last_clock && now<last_clock-50000000)clock_fault=true;
    last_clock=now;
    try {
      require(!clock_fault && fatal.empty(),clock_fault?"ROS clock moved backwards; restart with empty map":fatal);
      require(vio_ready(),"Waiting for fresh READY localization and VIO");
      if(!initialized) {
        dropped+=pending.size();pending.clear();
        if(stationary()) {
          if(!still_since)still_since=wall;
          if(wall-still_since>=int64_t(options.stationary_hold*1e9)) {
            map.initialize(poses.back(),*fixed);initialized=true;
          }
        } else still_since=0;
        require(initialized,"Waiting for stationary VIO initialization");
      }
      require(lidar_capture>0 && now-lidar_capture>=-50000000 && now-lidar_capture<=1000000000 && wall-lidar_wall<=1000000000,"Waiting for fresh paired LiDAR");
      bool changed=false;
      // One bounded batch per tick. Old scans cannot clear or recreate current occupancy.
      for(int count=0;count<8 && !pending.empty();++count) {
        auto& item=pending.front();
        const int64_t ns=item.pair.stamp_ns;
        if(ns<=map.last_scan || (recovery_after && ns<=recovery_after) || now-ns>1000000000) {
          pending.pop_front();++dropped;continue;
        }
        const int64_t end=ns+nearest_even(item.pair.point_time_offsets_s.back()*1e9);
        if(end>poses.back().stamp) {
          if(wall-item.receipt<=1500000000)break;
          pending.pop_front();++dropped;continue;
        }
        try {
          Frame f=deskew(item.pair,poses,options.imu_lidar,0.,options.pose_gap_ns);
          map.ingest(f);changed=true;
          if(recovery_after && ns>recovery_after)recovery_after=0;
        } catch(const std::exception& e) {++rejected;reason=e.what();}
        pending.pop_front();
      }
      require(!recovery_after,"Recovering: require newly captured and processed LiDAR");
      if(changed)publish(map.snapshot(poses.back(),*fixed));
    } catch(const std::exception& e) {valid=false;reason=e.what();}
    Status s;set_stamp(s.header,this->now().nanoseconds());s.valid_for.sec=1;
    const auto age=this->now().nanoseconds()-last_publish;
    const bool ready=valid && initialized && !clock_fault && fatal.empty() && vio_ready() && !recovery_after &&
      last_publish>0 && age>=-50000000 && age<=2000000000 && wall-last_map_wall<=3000000000;
    s.state=ready?Status::READY:last_publish?Status::STOPPED:Status::WAITING;s.detail=ready?reason:"Not ready: "+reason;
    status->publish(s);
  }
public:
  explicit IndoorMapper(const IndoorOptions& o):Node("nomad_indoor_mapper"),options(o),map(o),tf_buffer(get_clock()),tf_listener(tf_buffer) {
    grids[0]=create_publisher<Grid>("/nomad/navigation_costmap",map_qos());
    grids[1]=create_publisher<Grid>("/nomad/navigation_local_costmap",map_qos());
    grids[2]=create_publisher<Grid>("/nomad/camera_observation_support",map_qos());
    layers=create_publisher<Layers>("/nomad/planning_cost_layers",map_qos());
    diagnostic_layers=create_publisher<Layers>("/nomad/semantic_validation/cost_layers",map_qos());
    diagnostics=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/nomad/semantic_validation/capture_timing",map_qos());
    status=create_publisher<Status>("/nomad/perception/status",rclcpp::QoS(1).reliable());
    const std::array<std::string,4> names{"slope_score","semantic_score","slope_deg","slope_magnitude_deg"};
    for(size_t i=0;i<names.size();++i)images[i]=create_publisher<sensor_msgs::msg::Image>("/nomad/semantic_validation/"+names[i],map_qos());
    subs.push_back(create_subscription<Status>("/nomad/localization/status",rclcpp::QoS(10).reliable(),[this](Status::SharedPtr s){localization=*s;localization_wall=steady_ns();}));
    subs.push_back(create_subscription<nav_msgs::msg::Odometry>("/nomad/localization/odometry",rclcpp::QoS(10).reliable(),[this](nav_msgs::msg::Odometry::SharedPtr m){on_pose(*m);}));
    subs.push_back(create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS().keep_last(5),[this](sensor_msgs::msg::LaserScan::SharedPtr m){on_scan(*m);}));
    subs.push_back(create_subscription<sensor_msgs::msg::PointCloud>("/point_cloud",rclcpp::SensorDataQoS().keep_last(5),[this](sensor_msgs::msg::PointCloud::SharedPtr m){on_cloud(*m);}));
    timer=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    RCLCPP_INFO(get_logger(),"indoor_mapper_started legacy_floor_z_cm_ignored=%.3f vio_fixed_z_m=%.3f initial_radius_m=%.3f cameras_required=false ray_fov_deg=%.4f ray_range_m=2 no_cell_age_expiry=true lock_obstacle_cells=%s",options.floor_z*100,options.fixed_z,map.startup.radius_m,2.*options.half_fov_deg,options.lock_obstacle_cells?"true":"false");
  }
};
}

int main(int argc,char** argv) {
  std::fesetround(FE_TONEAREST);
  try {
    std::string config;
    for(int i=1;i+1<argc;++i)if(std::string(argv[i])=="--indoor-config")config=argv[++i];
    if(config.empty())throw std::runtime_error("Required: --indoor-config /absolute/path/indoor.json");
    std::ifstream input(config);if(!input)throw std::runtime_error("Cannot read indoor config");
    nomad::Json j;input>>j;auto options=nomad::IndoorOptions::load(j);
    rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<nomad::IndoorMapper>(options));
    if(rclcpp::ok())rclcpp::shutdown();
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"Indoor mapper failed: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 2;
  }
}

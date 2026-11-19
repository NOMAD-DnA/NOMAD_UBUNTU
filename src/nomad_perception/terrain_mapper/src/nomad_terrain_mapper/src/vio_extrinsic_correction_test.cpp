#include <nomad_costmap/vio_extrinsic_correction.hpp>
#include <iostream>
using namespace nomad;

int main() {
  int checks = 0;
  auto check = [&](bool ok) {
    ++checks;
    if (!ok) throw std::runtime_error("VIO extrinsic check " + std::to_string(checks));
  };
  auto pose = [](double w, double x, double y, double z, V3 t) {
    M4 m = M4::Identity();
    m.block<3,3>(0,0) = Eigen::Quaterniond(w,x,y,z).normalized().toRotationMatrix();
    m.block<3,1>(0,3) = t;
    return m;
  };
  // Independent fixture: the actual left and right VIO calibration entries.
  const M4 old_left = pose(.012215014987780827,.999922769439659,
      .0003027675657712804,-.002270880175625642,
      V3(-.005870614692479775,.006331938845222788,-.00047068094732958496));
  const M4 new_left = pose(-.009513829865992664,.9999521180918164,
      .0002533552252682195,-.002276922506534182,
      V3(-.0058706146924797755,.006331938845222788,-.00047068094732958496));
  const M4 old_right = pose(.01221501495164468,.9999227694400875,
      .0003027675680415415,-.00227088018099979,
      V3(.06908102739681782,.006373163488373425,-.0008116250337895812));
  const M4 new_right = pose(-.009513829902130148,.9999521180914599,
      .0002533552276542368,-.0022769225118541674,
      V3(.06908102739681868,.006373163488915295,-.0008116250335319637));
  const M4 correction = mapper_vio_extrinsic_correction();
  check((correction * old_left).isApprox(new_left,1e-12));
  check((correction * old_right).isApprox(new_right,1e-10));
  check(std::abs(Eigen::AngleAxisd(correction.block<3,3>(0,0)).angle()*180/M_PI-2.49)<1e-10);
  check(std::abs(correction.block<3,3>(0,0).determinant()-1)<1e-12);
  check((correction.row(3)-M4::Identity().row(3)).norm()<1e-12);

  M4 ia=M4::Identity(), ib=old_left;
  ia.block<3,3>(0,0)=Eigen::AngleAxisd(M_PI,V3::UnitX()).toRotationMatrix();
  ia.block<3,1>(0,3)=V3(-.0315,.006439,-.002);
  const std::array<M4,2> original{ib,ia};
  const auto corrected=mapper_vio_sensor_mounts(original);
  check(original[0].isApprox(ib,0));
  check(original[1].isApprox(ia,0));
  // The rigid sensor relationships and projected optical coordinates must remain.
  check((corrected[0].inverse()*corrected[1]).isApprox(ib.inverse()*ia,1e-12));
  M4 camera_lidar=M4::Identity();
  camera_lidar.block<3,3>(0,0)=Eigen::AngleAxisd(.61,V3(1,2,3).normalized()).toRotationMatrix();
  camera_lidar.block<3,1>(0,3)=V3(.07,-.13,.25);
  M4 camera_zed=M4::Identity();
  camera_zed.block<3,1>(0,3)=V3(.15,-.2,.03);
  M4 world=M4::Identity();
  world.block<3,3>(0,0)=Eigen::AngleAxisd(.39,V3(2,-1,3).normalized()).toRotationMatrix();
  world.block<3,1>(0,3)=V3(12,-8,3);
  for (const V3& p : Points{V3(.2,.3,1),V3(-.4,.7,3),V3(.8,-.5,7)}) {
    Eigen::Vector4d ph; ph<<p,1;
    const Eigen::Vector4d lidar_world=world*corrected[0]*camera_lidar*ph;
    const Eigen::Vector4d zed_world=world*corrected[0]*camera_zed*ph;
    check(((world*corrected[0]).inverse()*lidar_world).isApprox(camera_lidar*ph,1e-12));
    check(((world*corrected[0]*camera_zed).inverse()*zed_world).isApprox(ph,1e-12));
    const Eigen::Vector4d oak_world=world*corrected[1]*ph;
    check(((world*corrected[0]).inverse()*oak_world).isApprox(ib.inverse()*ia*ph,1e-12));
  }
  // Known flat and sloped planes must recover their true angle, without
  // subtracting 2.49 degrees from each slope or rotating public poses twice.
  for (double degrees : {0.,10.,-10.,20.}) {
    const M3 tilt=Eigen::AngleAxisd(degrees*M_PI/180,V3::UnitY()).toRotationMatrix();
    Points out;
    for (const V3& p : Points{V3(0,0,-.25),V3(1,0,-.25),V3(0,1,-.25)}) {
      Eigen::Vector4d actual; actual<<tilt*p,1;
      const Eigen::Vector4d camera=new_left.inverse()*actual;
      const Eigen::Vector4d result=correction*old_left*camera;
      check(result.isApprox(actual,1e-12));
      out.push_back(result.head<3>());
    }
    const V3 normal=(out[1]-out[0]).cross(out[2]-out[0]).normalized();
    const double recovered=std::atan2(std::hypot(normal.x(),normal.y()),std::abs(normal.z()))*180/M_PI;
    check(std::abs(recovered-std::abs(degrees))<1e-10);
  }
  std::cout<<checks<<" VIO extrinsic correction checks passed\n";
}

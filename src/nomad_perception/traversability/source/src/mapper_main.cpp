#include "nomad_costmap/ros.hpp"
#include <cfenv>
#include <iostream>
#include <sys/prctl.h>
#include <unistd.h>
#include <csignal>
int main(int argc, char** argv)
{
  std::fesetround(FE_TONEAREST);
  try{
    nomad::Arguments args(argc, argv);
    int ros_argc = 1;
    rclcpp::init(ros_argc, argv);
    std::string mode = args.get("--mode", "mapper");
    if (mode != "mapper")
    {
      pid_t parent = std::stoi(args.get("--parent-pid",std::to_string(getppid())));
      if (prctl(PR_SET_PDEATHSIG, SIGTERM)||getppid() != parent)
        throw std::runtime_error("Native parent disappeared");
    }
    int result = mode == "vio"?nomad::run_vio(args):mode == "points"?nomad::run_points(args):mode == "slope"?nomad::run_slope(args):mode == "mapper"?nomad::run_mapper(args):throw std::runtime_error("Unknown mode " + mode);
    if (rclcpp::ok())
      rclcpp::shutdown();
    return result;
  }
  catch (const std::exception& e){
    std::cerr << "Native costmap failed: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 2;
  }
}

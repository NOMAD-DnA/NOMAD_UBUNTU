#include "nomad_costmap/ros.hpp"
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("nomad_native_qos_contract_test");
  auto sub=node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/nomad_cpp_contract_test_points",nomad::sensor_qos(),[](sensor_msgs::msg::PointCloud2::SharedPtr){});
  auto qos=sub->get_actual_qos().get_rmw_qos_profile();
  if(qos.depth!=5||qos.history!=RMW_QOS_POLICY_HISTORY_KEEP_LAST||
     qos.reliability!=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT||
     qos.durability!=RMW_QOS_POLICY_DURABILITY_VOLATILE)
    throw std::runtime_error("RMW actual subscription QoS differs from selected queue-5 contract");
  std::cout<<"RMW actual sensor subscription: KEEP_LAST depth="<<qos.depth<<" BEST_EFFORT VOLATILE\n";
  sub.reset();node.reset();rclcpp::shutdown();
}

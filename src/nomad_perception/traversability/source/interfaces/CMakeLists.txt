cmake_minimum_required(VERSION 3.16)
project(nomad_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanningCostLayers.msg" "msg/DriveCommand.msg" "msg/VehicleState.msg"
  "msg/ModuleStatus.msg" "msg/PlannedMotion.msg" "msg/GnssQuality.msg"
  DEPENDENCIES std_msgs builtin_interfaces nav_msgs)
ament_export_dependencies(rosidl_runtime_cpp)
ament_package()

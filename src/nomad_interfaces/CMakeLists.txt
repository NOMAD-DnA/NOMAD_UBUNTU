cmake_minimum_required(VERSION 3.8)
project(nomad_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg" "msg/VehicleState.msg" "msg/ModuleStatus.msg" "msg/PlannedMotion.msg"
  DEPENDENCIES std_msgs builtin_interfaces nav_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()

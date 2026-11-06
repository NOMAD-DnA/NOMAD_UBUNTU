cmake_minimum_required(VERSION 3.16)
project(nomad_monitor_probe VERSION 0.1.0 LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nomad_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Threads REQUIRED)

# Adjust imported targets only in this configure. Do not modify ROS installs.
function(nomad_cpp_only_interface target)
  if(NOT TARGET "${target}")
    return()
  endif()
  get_property(visited_targets GLOBAL PROPERTY NOMAD_CPP_ONLY_VISITED_TARGETS)
  if("${target}" IN_LIST visited_targets)
    return()
  endif()
  set_property(GLOBAL APPEND PROPERTY NOMAD_CPP_ONLY_VISITED_TARGETS "${target}")
  get_target_property(dependencies "${target}" INTERFACE_LINK_LIBRARIES)
  if(NOT dependencies)
    return()
  endif()
  set(cpp_dependencies "")
  foreach(dependency IN LISTS dependencies)
    if(dependency MATCHES "__rosidl_generator_py")
      continue()
    endif()
    list(APPEND cpp_dependencies "${dependency}")
    if(TARGET "${dependency}")
      nomad_cpp_only_interface("${dependency}")
    endif()
  endforeach()
  set_property(TARGET "${target}" PROPERTY INTERFACE_LINK_LIBRARIES "${cpp_dependencies}")
endfunction()

set(nomad_cpp_dependencies
  rclcpp::rclcpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp
  nomad_interfaces::nomad_interfaces__rosidl_typesupport_cpp)
foreach(dependency IN LISTS nomad_cpp_dependencies)
  nomad_cpp_only_interface("${dependency}")
endforeach()

add_executable(nomad_monitor_probe monitor_probe.cpp)
target_compile_features(nomad_monitor_probe PRIVATE cxx_std_17)
set_target_properties(nomad_monitor_probe PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(nomad_monitor_probe PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(nomad_monitor_probe PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nomad_monitor_probe PRIVATE ${nomad_cpp_dependencies} ${OpenCV_LIBRARIES} Threads::Threads)
install(TARGETS nomad_monitor_probe RUNTIME DESTINATION bin)

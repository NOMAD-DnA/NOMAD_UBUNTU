cmake_minimum_required(VERSION 3.16)
project(nomad_external_frame_alias VERSION 0.1.0 LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# This configure only: native ROS exports may aggregate Python generators into
# C++ imported targets. Do not modify the installed ROS environment.
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
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
foreach(dependency IN LISTS nomad_cpp_dependencies)
  nomad_cpp_only_interface("${dependency}")
endforeach()

add_executable(nomad_external_frame_alias frame_alias.cpp)
target_compile_features(nomad_external_frame_alias PRIVATE cxx_std_17)
set_target_properties(nomad_external_frame_alias PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(nomad_external_frame_alias PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(nomad_external_frame_alias PRIVATE ${nomad_cpp_dependencies})
install(TARGETS nomad_external_frame_alias RUNTIME DESTINATION bin)

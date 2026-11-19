cmake_minimum_required(VERSION 3.16)
project(nomad_terrain_core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Same floating-point flags as the legacy nomad_costmap_cpp build.
add_compile_options(-fno-fast-math -ffp-contract=off)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
add_library(nomad_terrain_core SHARED src/common.cpp src/slope.cpp src/camera_model.cpp src/cell_executor.cpp)
target_include_directories(nomad_terrain_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(nomad_terrain_core PUBLIC Eigen3::Eigen Threads::Threads)
target_compile_options(nomad_terrain_core PRIVATE -Wall -Wextra -Wpedantic)
install(DIRECTORY include/ vendor/include/ DESTINATION include)
install(TARGETS nomad_terrain_core EXPORT nomad_terrain_coreTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
ament_export_targets(nomad_terrain_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Threads)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(nomad_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config worlds sensors assets DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/oak_bridge.py
  scripts/lidar_bridge.py
  scripts/generate_forest_map.py
  scripts/forest_models.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()

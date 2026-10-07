cmake_minimum_required(VERSION 3.8)
project(nomad_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config worlds models urdf assets maps DESTINATION share/${PROJECT_NAME})
install(FILES README.md GPS.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/gazebo_sensor_adapter.py
  scripts/lidar_bridge.py
  scripts/gps_visualizer.py
  scripts/command_watchdog.py
  scripts/teleop.py
  scripts/generate_vehicle.py
  scripts/generate_world.py
  scripts/select_map.py
  scripts/generate_vehicle_test_map.py
  scripts/generate_vio_test_maps.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()

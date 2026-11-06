cmake_minimum_required(VERSION 3.16)
project(nomad_fcn_preview LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Standalone CMake against the already sourced native ROS installation.
# No SDK download, colcon build, or package installation is performed here.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h HINTS /usr/local/cuda/include)
find_library(CUDA_RUNTIME_LIBRARY cudart HINTS /usr/local/cuda/lib64)
if(NOT CUDA_RUNTIME_INCLUDE_DIR OR NOT CUDA_RUNTIME_LIBRARY)
  message(FATAL_ERROR "Existing CUDA runtime headers/library not found")
endif()
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include /usr/local/TensorRT/include)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/TensorRT/lib)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "Existing native TensorRT development headers/library not found")
endif()

add_executable(nomad_fcn_preview fcn_image_node.cpp)
target_include_directories(nomad_fcn_preview PRIVATE ${TENSORRT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CUDA_RUNTIME_INCLUDE_DIR})
target_link_libraries(nomad_fcn_preview ${TENSORRT_LIBRARY} ${CUDA_RUNTIME_LIBRARY} ${OpenCV_LIBS})
ament_target_dependencies(nomad_fcn_preview rclcpp sensor_msgs)
target_compile_options(nomad_fcn_preview PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS nomad_fcn_preview RUNTIME DESTINATION bin)

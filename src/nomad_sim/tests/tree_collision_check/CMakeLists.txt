cmake_minimum_required(VERSION 3.16)
project(nomad_tree_collision_check LANGUAGES CXX)

# Use the workspace's custom Box2D config, not an unrelated system Box2D build.
# The explicit *_DIR values are supplied by the reproducible command.
find_package(mvsim-simulator CONFIG REQUIRED)

add_executable(test_tree_collision ../test_tree_collision.cpp)
target_compile_features(test_tree_collision PRIVATE cxx_std_17)
target_compile_options(test_tree_collision PRIVATE -Wall -Wextra -Wshadow -Werror=return-type)
target_link_libraries(test_tree_collision PRIVATE mvsim::mvsim-simulator)

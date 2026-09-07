cmake_minimum_required(VERSION 3.8)
project(prop_maneuvers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# ── Shared library ───────────────────────────────────────────────────────────
add_library(prop_maneuvers_lib src/geometry.cpp)

install(TARGETS prop_maneuvers_lib DESTINATION lib)

# ── Simulator support ────────────────────────────────────────────────────────
add_executable(scan_to_cloud src/scan_to_cloud_node.cpp)
ament_target_dependencies(scan_to_cloud rclcpp sensor_msgs laser_geometry)

install(TARGETS scan_to_cloud DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Install ──────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Repo C++ style (.clang-format) is 4-space/Allman, which uncrustify's ROS
  # profile can never accept; clang-format in pre-commit is the source of truth.
  set(ament_cmake_uncrustify_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry prop_maneuvers_lib)

  # Nothing else compiles constants.hpp until Task 7 builds a node, so a break
  # in it would otherwise go unnoticed for several tasks. This costs nothing and
  # catches it immediately.
  ament_add_gtest(test_constants_compiles test/test_constants_compiles.cpp)
  ament_target_dependencies(test_constants_compiles rclcpp)
  target_link_libraries(test_constants_compiles prop_maneuvers_lib)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

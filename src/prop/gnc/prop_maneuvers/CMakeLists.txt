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
add_library(prop_maneuvers_lib src/geometry.cpp src/driver.cpp src/spinner.cpp)
# Split out deliberately: with four sources on one line, cmake-format wraps the
# call onto a hanging continuation that ament_lint_cmake then rejects as "weird
# indentation". Both tools accept this form. Do not fold it back in.
target_sources(prop_maneuvers_lib PRIVATE src/target_lock.cpp src/maneuvers.cpp)
ament_target_dependencies(
  prop_maneuvers_lib
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS prop_maneuvers_lib DESTINATION lib)

# ── Maneuver programs ────────────────────────────────────────────────────────
add_executable(face_object src/face_object_node.cpp)
target_link_libraries(face_object prop_maneuvers_lib)
ament_target_dependencies(
  face_object
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

add_executable(circle_object src/circle_object_node.cpp)
target_link_libraries(circle_object prop_maneuvers_lib)
ament_target_dependencies(
  circle_object
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

add_executable(approach_object src/approach_object_node.cpp)
target_link_libraries(approach_object prop_maneuvers_lib)
ament_target_dependencies(
  approach_object
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS face_object circle_object approach_object
        DESTINATION lib/${PROJECT_NAME})

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
  # ament_flake8's ROS profile demands single quotes; the repo's pre-commit
  # formatter enforces double. They cannot both be satisfied, and pre-commit is
  # the source of truth here, same as clang-format above. Genuine flake8
  # findings (import grouping, missing docstrings) are fixed in the source
  # rather than silenced -- this only mutes the quote-style disagreement.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
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

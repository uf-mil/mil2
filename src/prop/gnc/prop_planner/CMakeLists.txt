cmake_minimum_required(VERSION 3.8)
project(prop_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# The maps and the searches are plain C++ with no ROS in them, so they build and
# test on their own.
add_library(
  prop_planner_core
  src/geometry.cpp src/obstacle_map.cpp src/visibility_planner.cpp
  src/evidence_grid.cpp src/grid_planner.cpp)
target_include_directories(
  prop_planner_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Occupancy grid from the lidar, and A* through it.
add_executable(occupancy_grid src/occupancy_grid_node.cpp)
target_link_libraries(occupancy_grid prop_planner_core)
ament_target_dependencies(
  occupancy_grid
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_executable(grid_planner src/grid_planner_node.cpp)
target_link_libraries(grid_planner prop_planner_core)
ament_target_dependencies(
  grid_planner
  geometry_msgs
  nav_msgs
  rclcpp
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros)

# Capsule obstacle map from tracked_markers, and a visibility graph over it.
add_executable(planner src/planner.cpp)
target_link_libraries(planner prop_planner_core)
ament_target_dependencies(
  planner
  geometry_msgs
  mil_msgs
  nav_msgs
  rclcpp
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

install(
  TARGETS prop_planner_core occupancy_grid grid_planner planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/inspect_map.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(grid_planner_test test/grid_planner_test.cpp)
  target_link_libraries(grid_planner_test prop_planner_core)

  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND
    AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_uncrustify
    ament_cmake_lint_cmake)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

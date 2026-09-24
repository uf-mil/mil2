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
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# The map and the search are plain C++ with no ROS in them, so they build and
# test on their own.
add_library(prop_planner_core src/obstacle_map.cpp src/visibility_planner.cpp)
target_include_directories(
  prop_planner_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(planner src/planner.cpp)
target_link_libraries(planner prop_planner_core)
ament_target_dependencies(
  planner
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

install(
  TARGETS prop_planner_core planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/inspect_map.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  # The map and the search hold no ROS, so they test as plain C++.
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_obstacle_map test/obstacle_map.cpp)
  target_link_libraries(test_obstacle_map prop_planner_core)

  ament_add_gtest(test_visibility_planner test/visibility_planner.cpp)
  target_link_libraries(test_visibility_planner prop_planner_core)

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

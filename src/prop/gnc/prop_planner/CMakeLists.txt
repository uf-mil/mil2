cmake_minimum_required(VERSION 3.8)
project(prop_planner)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(prop_planner_core src/geometry.cpp src/obstacle_map.cpp
                              src/visibility_planner.cpp)
target_compile_features(prop_planner_core PUBLIC cxx_std_17)
target_include_directories(prop_planner_core PUBLIC include)

add_executable(prop_planner_node src/prop_planner_node.cpp)
target_compile_features(prop_planner_node PUBLIC cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(prop_planner_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(prop_planner_node prop_planner_core)
ament_target_dependencies(
  prop_planner_node
  nav_msgs
  std_srvs
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS prop_planner_node prop_planner_core
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/mock_map.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(core_test geometry obstacle_map visibility_planner)
    ament_add_gtest(test_${core_test} test/${core_test}.cpp)
    target_link_libraries(test_${core_test} prop_planner_core)
  endforeach()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_prop_planner test/test_prop_planner.py TIMEOUT 90 ENV
    PROP_PLANNER_EXECUTABLE=$<TARGET_FILE:prop_planner_node>)
endif()

ament_package()

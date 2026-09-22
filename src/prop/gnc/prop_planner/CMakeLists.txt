cmake_minimum_required(VERSION 3.8)
project(prop_planner)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(prop_planner_node src/prop_planner_node.cpp)
target_compile_features(prop_planner_node PUBLIC cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(prop_planner_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(prop_planner_node nav_msgs rclcpp)

install(TARGETS prop_planner_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_prop_planner test/test_prop_planner.py
    TIMEOUT 90
    ENV PROP_PLANNER_EXECUTABLE=$<TARGET_FILE:prop_planner_node>)
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(subjugator_trajectory_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_util)

add_executable(trajectory_planner src/node.cpp src/TrajectoryPlanner.cpp)
ament_target_dependencies(trajectory_planner rclcpp nav_msgs geometry_msgs
                          Eigen3 nav2_util)
target_include_directories(trajectory_planner PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                      include)

install(TARGETS trajectory_planner DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_trajectory_planner.py)
endif()

ament_package()

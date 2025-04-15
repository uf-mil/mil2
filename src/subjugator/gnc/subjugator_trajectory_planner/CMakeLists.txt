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

add_executable(trajectory_planner src/node.cpp src/trajectory_planner.cpp)
ament_target_dependencies(trajectory_planner rclcpp nav_msgs geometry_msgs
                          Eigen3)
target_include_directories(trajectory_planner PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                      include)

install(TARGETS trajectory_planner DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()

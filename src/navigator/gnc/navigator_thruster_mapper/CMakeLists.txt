cmake_minimum_required(VERSION 3.8)
project(navigator_thruster_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Build thruster_map library
add_library(thruster_mapper src/thruster_mapper.cpp)
target_link_libraries(thruster_mapper Eigen3::Eigen)

# Build thruster_mapper_node executable
add_executable(thruster_mapper_node src/thruster_mapper_node.cpp)
target_link_libraries(thruster_mapper_node thruster_mapper)
ament_target_dependencies(
  thruster_mapper_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  urdf
  tf2_ros)

install(TARGETS thruster_mapper_node DESTINATION lib/${PROJECT_NAME})

ament_package()

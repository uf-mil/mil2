cmake_minimum_required(VERSION 3.8)
project(navigator_thrust_mapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(thruster_mapper_node src/thruster_mapper_node.cpp)
ament_target_dependencies(thruster_mapper_node rclcpp geometry_msgs sensor_msgs
                          std_msgs)

install(TARGETS thruster_mapper_node DESTINATION lib/${PROJECT_NAME})

ament_package()

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
find_package(Eigen3 REQUIRED)

include_directories(include)

# Build thruster_map library
add_library(thruster_map src/thruster_map.cpp)
target_link_libraries(thruster_map PUBLIC Eigen3::Eigen)

# Build thruster_mapper_node executable
add_executable(thruster_mapper_node src/thruster_mapper_node.cpp)
target_link_libraries(thruster_mapper_node thruster_map Eigen3::Eigen)
ament_target_dependencies(thruster_mapper_node rclcpp geometry_msgs sensor_msgs
                          std_msgs)

# Build test executable
add_executable(thruster_map_test src/thruster_map_test.cpp)
target_link_libraries(thruster_map_test thruster_map Eigen3::Eigen)

install(TARGETS thruster_mapper_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS thruster_map_test DESTINATION lib/${PROJECT_NAME})

ament_package()

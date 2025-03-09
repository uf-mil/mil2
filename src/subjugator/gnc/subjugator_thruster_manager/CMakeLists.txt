cmake_minimum_required(VERSION 3.8)
project(subjugator_thruster_manager)

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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(subjugator_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(thruster_manager src/thruster_manager.cpp)
ament_target_dependencies(thruster_manager rclcpp geometry_msgs subjugator_msgs)
target_include_directories(thruster_manager PRIVATE ${EIGEN3_INCLUDE_DIR} include)

add_executable(forward_to_sim src/forward_to_sim.cpp)
ament_target_dependencies(forward_to_sim rclcpp subjugator_msgs)

install(TARGETS
  thruster_manager
  forward_to_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

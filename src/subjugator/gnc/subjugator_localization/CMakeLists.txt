cmake_minimum_required(VERSION 3.8)
project(subjugator_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install launch files.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/reset_srv_node.py DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/depth_to_pose_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()

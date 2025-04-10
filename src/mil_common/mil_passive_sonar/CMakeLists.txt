cmake_minimum_required(VERSION 3.5)
project(mil_passive_sonar)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(mil_msgs REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

ament_export_dependencies(mil_msgs rclcpp)

ament_package()

install(PROGRAMS scripts/ping_publisher.py scripts/fakeping_pipeline.sh
                 scripts/pipeline.sh DESTINATION lib/${PROJECT_NAME})

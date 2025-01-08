cmake_minimum_required(VERSION 3.5)
project(mil_passive_sonar)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProcessedPing.msg"
  DEPENDENCIES std_msgs geometry_msgs service_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(
	rosidl_default_runtime
	std_msgs
	geometry_msgs
	service_msgs
	rclcpp
)

ament_package()

install(PROGRAMS scripts/ping_publisher.py scripts/fakeping_pipeline.sh scripts/pipeline.sh DESTINATION lib/${PROJECT_NAME})

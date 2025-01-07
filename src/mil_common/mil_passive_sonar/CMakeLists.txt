cmake_minimum_required(VERSION 3.5)
project(mil_passive_sonar)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(Boost REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(msg_files
  "msg/ProcessedPing.msg"
  #  "msg/Debug.msg"
  #  "msg/HydrophoneSamples.msg"
  #  "msg/HydrophoneSamplesStamped.msg"
  #  "msg/Ping.msg"
  #  "msg/Triggered.msg"
)

#set(srv_files
  #  "srv/FindPinger.srv"
  #)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  #${srv_files}
  DEPENDENCIES std_msgs geometry_msgs service_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(
	rosidl_default_runtime
	#	tf2
	std_msgs
	geometry_msgs
	service_msgs
	rclcpp
)

ament_package()


# Build sylphase_ros_bridge
#add_executable(sylphase_sonar_ros_bridge src/sylphase_ros_bridge.cpp)
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(
#	sylphase_sonar_ros_bridge
#	rclcpp::rclcpp
#	tf2::tf2
#	${std_msgs_TARGETS}
#	${geometry_msgs_TARGETS}
#	${cpp_typesupport_target}
#)
#target_include_directories(sylphase_sonar_ros_bridge PUBLIC
#  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
#)

install(PROGRAMS scripts/ping_publisher.py scripts/fakeping_pipeline.sh scripts/pipeline.sh DESTINATION lib/${PROJECT_NAME})


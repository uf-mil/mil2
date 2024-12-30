cmake_minimum_required(VERSION 3.5)
project(mil_passive_sonar)

find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_runtime REQUIRED)
find_package(message_generation REQUIRED)
find_package(rospy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roscpp REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(msg_files
  "msg/ProcessedPing.msg"
  "msg/Debug.msg"
  "msg/HydrophoneSamples.msg"
  "msg/HydrophoneSamplesStamped.msg"
  "msg/Ping.msg"
  "msg/Triggered.msg"
)

set(srv_files
  "srv/FindPinger.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS tf std_msgs message_runtime message_generation rospy
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Build sylphase_ros_bridge
add_executable(sylphase_sonar_ros_bridge src/sylphase_ros_bridge.cpp)
target_link_libraries(sylphase_sonar_ros_bridge ${catkin_LIBRARIES})
add_dependencies(sylphase_sonar_ros_bridge ${catkin_EXPORTED_TARGETS})

install(PROGRAMS scripts/ping_publisher scripts/ping_printer scripts/ping_logger scripts/ping_plotter scripts/hydrophones DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

ament_package()


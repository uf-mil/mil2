cmake_minimum_required(VERSION 3.5)
project(mil_poi)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(msg_files
  "msg/POI.msg"
  "msg/POIArray.msg"
)

set(srv_files
  "srv/AddPOI.srv"
  "srv/DeletePOI.srv"
  "srv/MovePOI.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/poi.test)
endif()

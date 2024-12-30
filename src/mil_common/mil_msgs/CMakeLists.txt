cmake_minimum_required(VERSION 3.5)
project(mil_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(action_files
  "action/MoveTo.action"
  "action/BagOnline.action"
)

set(msg_files
  "msg/DepthStamped.msg"
  "msg/ObjectInImage.msg"
  "msg/ObjectsInImage.msg"
  "msg/PerceptionObject.msg"
  "msg/PerceptionObjectArray.msg"
  "msg/Point2D.msg"
  "msg/PoseTwistStamped.msg"
  "msg/PoseTwist.msg"
  "msg/RangeStamped.msg"
  "msg/VelocityMeasurements.msg"
  "msg/VelocityMeasurement.msg"
)

set(srv_files
  "srv/CameraToLidarTransform.srv"
  "srv/ObjectDBQuery.srv"
  "srv/SetGeometry.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs action_msgs geometry_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

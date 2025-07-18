cmake_minimum_required(VERSION 3.8)
project(subjugator_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/Servo.srv"
  "msg/Alarm.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/ThrusterEfforts.msg"
  "msg/Trajectory.msg"
  "msg/Waypoint.msg"
  "srv/BMatrix.srv"
  "srv/OcrRequest.srv"
  "srv/TBDetectionSwitch.srv"
  "srv/VisionRequest2D.srv"
  "srv/Bounds.srv"
  "srv/PathPlan.srv"
  "srv/VisionRequest.srv"
  "srv/SearchPose.srv"
  "srv/TorpBoardPoseRequest.srv"
  "srv/GuessRequest.srv"
  "srv/SetValve.srv"
  "srv/MotionPlan.srv"
  "srv/Sonar.srv"
  "action/SearchObject.action"
  "action/NavigateAround.action"
  "action/NavigateThrough.action"
  "action/Movement.action"
  "action/Wait.action"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs)

ament_package()

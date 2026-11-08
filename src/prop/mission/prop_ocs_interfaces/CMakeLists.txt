cmake_minimum_required(VERSION 3.8)
project(prop_ocs_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/DockCommand.msg"
  "msg/DockStatus.msg"
  "msg/DockResult.msg"
  "msg/DockReceipt.msg"
  "msg/Buoy.msg"
  "msg/SafePassageReport.msg"
  DEPENDENCIES
  builtin_interfaces
  geographic_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

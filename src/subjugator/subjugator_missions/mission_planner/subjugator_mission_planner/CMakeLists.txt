cmake_minimum_required(VERSION 3.5)
project(subjugator_mission_planner)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(
  ${PROJECT_NAME} "subjugator_mission_planner/action/NavigateAround.action"
  "subjugator_mission_planner/action/NavigateThrough.action"
  "subjugator_mission_planner/action/SearchObject.action")

# Export dependencies for generated interfaces
ament_export_dependencies(rosidl_default_runtime)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install executable scripts
install(
  PROGRAMS scripts/mission_planner.py scripts/search_object_server.py
           scripts/navigate_around_object_server.py
           scripts/navigate_through_object_server.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

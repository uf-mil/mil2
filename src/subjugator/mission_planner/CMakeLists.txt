cmake_minimum_required(VERSION 3.14)
project(mission_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mil_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

# ---- Operations library (leaf nodes)
add_library(
  operations
  subjugator_operations/src/publish_goal.cpp
  subjugator_operations/src/at_goal_pose.cpp
  subjugator_operations/src/detect_target.cpp
  subjugator_operations/src/hone_bearing.cpp)

ament_target_dependencies(
  operations
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mil_msgs
  behaviortree_cpp)

# ---- Node
add_executable(mission_planner_node src/mission_planner_node.cpp)
target_link_libraries(mission_planner_node operations)
ament_target_dependencies(
  mission_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mil_msgs
  behaviortree_cpp
  ament_index_cpp)

install(
  TARGETS operations mission_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY subjugator_operations/include/ DESTINATION include)
install(DIRECTORY subjugator_missions/xml/ DESTINATION share/${PROJECT_NAME}/bt)
ament_package()

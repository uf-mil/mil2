cmake_minimum_required(VERSION 3.14)
project(mission_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include
  ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_missions/include)

# ---- Operations library (leaf nodes)
add_library(operations subjugator_operations/src/publish_goal.cpp
                       subjugator_operations/src/at_goal_pose.cpp)
ament_target_dependencies(operations rclcpp geometry_msgs nav_msgs
                          behaviortree_cpp_v3)

# ---- Missions library (subtree builders)
add_library(
  missions
  subjugator_missions/src/move_mission.cpp
  subjugator_missions/src/relative_motion_mission.cpp
  subjugator_missions/src/square_test_mission.cpp)
ament_target_dependencies(missions rclcpp behaviortree_cpp_v3)

# ---- Node
add_executable(mission_planner_node src/mission_planner_node.cpp)
target_link_libraries(mission_planner_node operations missions)
# Rely on ament to link BehaviorTree.CPP instead of direct target_link_libraries
ament_target_dependencies(mission_planner_node rclcpp geometry_msgs nav_msgs
                          behaviortree_cpp_v3)

install(
  TARGETS operations missions mission_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY subjugator_operations/include/ DESTINATION include)
install(DIRECTORY subjugator_missions/include/ DESTINATION include)

ament_package()

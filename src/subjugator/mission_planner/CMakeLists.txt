cmake_minimum_required(VERSION 3.14)
project(mission_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mil_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(subjugator_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(subjugator_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(subjugator_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

# ---- Operations library (leaf nodes)
add_library(
  operations
  subjugator_operations/src/publish_goal.cpp
  subjugator_operations/src/at_goal_pose.cpp
  subjugator_operations/src/detect_target.cpp
  subjugator_operations/src/hone_bearing.cpp
  subjugator_operations/src/center_camera.cpp
  subjugator_operations/src/check_yolo_model.cpp
  subjugator_operations/src/track_largest_poles.cpp
  subjugator_operations/src/poles_big_enough.cpp
  subjugator_operations/src/determine_channel_side.cpp
  subjugator_operations/src/select_target.cpp
  subjugator_operations/src/select_basket.cpp
  subjugator_operations/src/select_face_symbol.cpp
  subjugator_operations/src/any_poles_detected.cpp
  subjugator_operations/src/has_found_pair.cpp
  subjugator_operations/src/actuate_servo.cpp
  subjugator_operations/src/record_target_scale.cpp
  subjugator_operations/src/confirm_grasp_by_scale.cpp
  subjugator_operations/src/track_best_pair.cpp
  subjugator_operations/src/hone_midpoint.cpp
  subjugator_operations/src/log_to_file.cpp
  subjugator_operations/src/nav_channel_control.cpp
  subjugator_operations/src/align_depth.cpp
  subjugator_operations/src/align_yaw.cpp
  subjugator_operations/src/go_to_pinger.cpp
  subjugator_operations/src/descend_until_detected.cpp
  subjugator_operations/src/search_for_target.cpp
  subjugator_operations/src/ros_timeout.cpp
  subjugator_operations/src/ros_delay.cpp)

ament_target_dependencies(
  operations
  rclcpp
  geometry_msgs
  std_srvs
  nav_msgs
  mil_msgs
  tf2
  subjugator_msgs
  sensor_msgs
  yolo_msgs
  subjugator_msgs
  behaviortree_cpp
  lifecycle_msgs
  rcl_interfaces
  ament_index_cpp)

# Node
add_executable(mission_planner_node src/mission_planner_node.cpp)
target_link_libraries(mission_planner_node operations yaml-cpp)
ament_target_dependencies(
  mission_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  mil_msgs
  sensor_msgs
  yolo_msgs
  subjugator_msgs
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
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../gnc/subjugator_vision/models/
        DESTINATION share/${PROJECT_NAME}/models)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_select_target_logic test/test_select_target_logic.cpp)
  target_include_directories(
    test_select_target_logic
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_select_basket_logic test/test_select_basket_logic.cpp)
  target_include_directories(
    test_select_basket_logic
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_select_face_symbol_logic
                  test/test_select_face_symbol_logic.cpp)
  target_include_directories(
    test_select_face_symbol_logic
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_search_pattern test/test_search_pattern.cpp)
  target_include_directories(
    test_search_pattern
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_detection_gate test/test_detection_gate.cpp)
  target_include_directories(
    test_detection_gate
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_target_projection test/test_target_projection.cpp)
  target_include_directories(
    test_target_projection
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_ros_time_budget test/test_ros_time_budget.cpp)
  target_include_directories(
    test_ros_time_budget
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)

  ament_add_gtest(test_quat_math test/test_quat_math.cpp)
  target_include_directories(
    test_quat_math
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/subjugator_operations/include)
  ament_target_dependencies(test_quat_math geometry_msgs)
endif()

ament_package()

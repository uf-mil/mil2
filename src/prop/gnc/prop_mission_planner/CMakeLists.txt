cmake_minimum_required(VERSION 3.24)
project(prop_mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(prop_maneuvers REQUIRED)
find_package(rclcpp REQUIRED)

# Every node, plus the shared Context. Nodes register themselves from static
# initialisers (REGISTER in context.hpp), so anything linking this must use
# WHOLE_ARCHIVE or the linker drops the registrations.
#
# cmake-format keeps re-wrapping this call onto a hanging continuation with an
# odd indent, which ament_lint_cmake then rejects as "weird indentation";
# disable formatting for it so the lint-clean, even-indented form sticks.
# cmake-format: off
add_library(
  operations STATIC
  src/context.cpp
  src/static_object.cpp
  src/ros_timeout.cpp
  src/ros_delay.cpp
  src/maneuver_node.cpp
  src/maneuver_nodes.cpp)
# cmake-format: on
# cmake-format: off
target_include_directories(
  operations PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# cmake-format: on
target_link_libraries(operations PUBLIC prop_maneuvers::prop_maneuvers_lib)
ament_target_dependencies(operations PUBLIC rclcpp behaviortree_cpp)

add_executable(mission_planner_node src/mission_planner_node.cpp)
# cmake-format: off
target_link_libraries(
  mission_planner_node PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,operations>"
  ament_index_cpp::ament_index_cpp)
# cmake-format: on

install(TARGETS mission_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY missions/ DESTINATION share/${PROJECT_NAME}/bt)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_object_ref test/test_object_ref.cpp)
  target_link_libraries(test_object_ref operations)

  ament_add_gtest(test_ros_time_budget test/test_ros_time_budget.cpp)
  target_link_libraries(test_ros_time_budget operations)

  ament_add_gtest(test_basic_nodes test/test_basic_nodes.cpp)
  target_link_libraries(test_basic_nodes
                        "$<LINK_LIBRARY:WHOLE_ARCHIVE,operations>")

  ament_add_gtest(test_maneuver_nodes test/test_maneuver_nodes.cpp)
  target_link_libraries(test_maneuver_nodes
                        "$<LINK_LIBRARY:WHOLE_ARCHIVE,operations>")

  # Separate from test_maneuver_nodes: these feed in odometry, which the
  # suite-wide Context keeps, so they cannot share a process with the
  # no-odometry test.
  ament_add_gtest(test_maneuver_nodes_live test/test_maneuver_nodes_live.cpp)
  target_link_libraries(test_maneuver_nodes_live
                        "$<LINK_LIBRARY:WHOLE_ARCHIVE,operations>")

  ament_add_gtest(test_missions test/test_missions.cpp)
  target_link_libraries(test_missions
                        "$<LINK_LIBRARY:WHOLE_ARCHIVE,operations>")
  target_compile_definitions(
    test_missions PRIVATE MISSIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/missions")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

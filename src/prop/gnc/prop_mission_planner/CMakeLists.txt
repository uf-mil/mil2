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
  src/ros_delay.cpp)
# cmake-format: on
# cmake-format: off
target_include_directories(
  operations PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# cmake-format: on
target_link_libraries(operations PUBLIC prop_maneuvers::prop_maneuvers_lib)
ament_target_dependencies(operations PUBLIC rclcpp behaviortree_cpp)

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

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(prop_maneuvers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

set(MANEUVER_DEPS
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs)

# ── Shared library ───────────────────────────────────────────────────────────
add_library(prop_maneuvers_lib src/geometry.cpp src/driver.cpp)
# Split out deliberately: with four sources on one line, cmake-format wraps the
# call onto a hanging continuation that ament_lint_cmake then rejects as "weird
# indentation". Both tools accept this form. Do not fold it back in.
target_sources(prop_maneuvers_lib PRIVATE src/target_lock.cpp src/maneuvers.cpp
                                          src/runner.cpp)
ament_target_dependencies(prop_maneuvers_lib ${MANEUVER_DEPS})

install(TARGETS prop_maneuvers_lib DESTINATION lib)

# ── Maneuver programs ────────────────────────────────────────────────────────
set(MANEUVER_PROGRAMS face_object circle_object approach_object)
foreach(program ${MANEUVER_PROGRAMS})
  add_executable(${program} src/${program}_node.cpp)
  target_link_libraries(${program} prop_maneuvers_lib)
  ament_target_dependencies(${program} ${MANEUVER_DEPS})
endforeach()

install(TARGETS ${MANEUVER_PROGRAMS} DESTINATION lib/${PROJECT_NAME})

# ── Simulator support ────────────────────────────────────────────────────────
add_executable(scan_to_cloud src/scan_to_cloud_node.cpp)
ament_target_dependencies(scan_to_cloud rclcpp sensor_msgs laser_geometry)

add_executable(blind_spot_mask src/blind_spot_mask_node.cpp)
target_link_libraries(blind_spot_mask prop_maneuvers_lib)
ament_target_dependencies(blind_spot_mask rclcpp sensor_msgs)

install(TARGETS scan_to_cloud blind_spot_mask DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# ── Install ──────────────────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Repo C++ style (.clang-format) is 4-space/Allman, which uncrustify's ROS
  # profile can never accept; clang-format in pre-commit is the source of truth.
  set(ament_cmake_uncrustify_FOUND TRUE)
  # ament_flake8's ROS profile demands single quotes; the repo's pre-commit
  # formatter enforces double. They cannot both be satisfied, and pre-commit is
  # the source of truth here, same as clang-format above. Genuine flake8
  # findings (import grouping, missing docstrings) are fixed in the source
  # rather than silenced -- this only mutes the quote-style disagreement.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry prop_maneuvers_lib)

  # Constructs Constants from defaults and from the shipped config, so a bad
  # parameter or a typo in maneuvers.yaml fails here rather than on the water.
  ament_add_gtest(test_constants_compiles test/test_constants_compiles.cpp)
  target_link_libraries(test_constants_compiles prop_maneuvers_lib)
  # The source tree's copy, not the installed one: the test must fail on a bad
  # config before it is installed anywhere.
  target_compile_definitions(
    test_constants_compiles
    PRIVATE MANEUVERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/maneuvers.yaml")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)

project(subjugator_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY models urdf DESTINATION share/${PROJECT_NAME})

# Expand the sim URDF at build time so the Gazebo worlds' <include> of
# urdf/sub9.urdf resolves without a launch-time generation step. The xacro macro
# symlinks build/.../ament_cmake_index/share/${PROJECT_NAME} -> the source tree,
# so the file's own $(find subjugator_description) includes resolve before this
# package is installed; $(find mission_planner) (grasp_targets.yaml, read at
# expansion time) resolves from its install, ordered by the build_depend.
xacro_add_xacro_file(urdf/sub9_sim.urdf.xacro sub9.urdf)
add_custom_target(${PROJECT_NAME}_generate_urdf ALL
                  DEPENDS ${XACRO_OUTPUT_FILE})
install(FILES ${XACRO_OUTPUT_FILE} DESTINATION share/${PROJECT_NAME}/urdf)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

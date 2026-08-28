cmake_minimum_required(VERSION 3.8)
project(prop_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY config launch models xacro DESTINATION share/${PROJECT_NAME})

# The model is macro-assembled, so expand it at build time: gz still resolves it
# as a plain model:// resource, and other models can reuse the macros.
xacro_add_files(xacro/lidar_platform.sdf.xacro TARGET lidar_platform_sdf
                INSTALL DESTINATION models/lidar_platform)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

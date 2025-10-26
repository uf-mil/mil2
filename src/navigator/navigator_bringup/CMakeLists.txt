cmake_minimum_required(VERSION 3.5)

project(navigator_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_index_python REQUIRED) find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)
find_package(navigator_description REQUIRED)
find_package(navigator_gazebo REQUIRED)

# Install project launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install URDF files install(DIRECTORY urdf DESTINATION
# share/${PROJECT_NAME}/urdf)

# Install project configuration files install(DIRECTORY config/ DESTINATION
# share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

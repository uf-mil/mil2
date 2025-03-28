cmake_minimum_required(VERSION 3.5)
project(subjugator_gazebo)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)

find_package(subjugator_description REQUIRED)
find_package(mil_msgs REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
else()
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

#
# Add Plugins Here
#

# Plugin 0: Example Plugin
add_library(ExamplePlugin SHARED
  src/ExamplePlugin.cc
)

# Plugin 1: Underwater Camera
add_library(UnderwaterCamera SHARED
  src/UnderwaterCamera.cc
)

# Plugin 2: DVL Bridge
add_library(DVLBridge SHARED
  src/DVLBridge.cc
)

# Plugin 3: Active Sonar
add_library(ActiveSonar SHARED
  src/ActiveSonar.cc
)

# Ensures plugin can see include directory to access its header files
target_include_directories(
  ExamplePlugin PRIVATE include
)

target_include_directories(
  UnderwaterCamera PRIVATE include
)

target_include_directories(
  DVLBridge PRIVATE include
)

target_include_directories(
  ActiveSonar PRIVATE include
)

# Add additional needed libraries
target_link_libraries(ExamplePlugin
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}  # Links gazebo simulator version to your plugin.
)

# Link Libraries for Underwater Camera plugin
target_link_libraries(UnderwaterCamera
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  ${OpenCV_LIBS}
)

# Link libraries for DVL Bridge plugin
target_link_libraries(DVLBridge
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

# Link libraries for Active Sonar plugin
target_link_libraries(ActiveSonar
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

# Specify dependencies using ament_target_dependencies
ament_target_dependencies(ExamplePlugin
  #
  # Here is where you would place any dependencies for your plugin
  #
)

ament_target_dependencies(UnderwaterCamera
  rclcpp
  sensor_msgs
)

ament_target_dependencies(DVLBridge
  nav_msgs
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(ActiveSonar
  mil_msgs
)

# Install the plugins
install(TARGETS UnderwaterCamera ExamplePlugin DVLBridge ActiveSonar
LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)


# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/subjugator_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()

cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(subjugator_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(subjugator_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

include_directories(${OpenCV_INCLUDE_DIRS})


# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Garden
else()
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()


# Following 'add_library' directive defines a library target.
# The 'SHARED' keyword indicates that a shared library should be compiled, and
# is followed by the list of source files for the target.
add_library(UnderwaterCamera
  SHARED
  src/UnderwaterCamera.cc
)

# Following 'target_include_directories' directive specifies 'include' as the
# include directory to use when compiling the library target.
# The 'PRIVATE' keyword specifies that this directive will populate the
# INCLUDE_DIRECTORIES property for the library target.
target_include_directories(
  UnderwaterCamera PRIVATE include
)

# Following 'target_link_libraries' directive specifies to use the 
# gz-sim library when linking the library target.
# The 'PRIVATE' keyword stipulates that the gz-sim library will not
# automatically be included if the library target were to
# be linked to anoter target. 
# ${GZ_SIM_VER} is substituted by the value that is was set to above.
target_link_libraries(UnderwaterCamera
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  ${OpenCV_LIBS}
)


# Specify dependancies using ament_target_dependencies
ament_target_dependencies(UnderwaterCamera
  rclcpp
  sensor_msgs
)

# # Following directives similarly specify another target: 'FullSystem'.
# add_library(FullSystem
#   SHARED
#   src/FullSystem.cc
# )

# target_include_directories(
#   FullSystem PRIVATE include
# )

# target_link_libraries(FullSystem PRIVATE
#   gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})



# Following 'install' directive ensures that the compiled libraries will be copied
# to the subfolder 'lib/subjugator_gazebo' of the install directory.
install(
  TARGETS
   UnderwaterCamera
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
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
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
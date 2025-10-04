cmake_minimum_required(VERSION 3.0.2)
project(navigator_kill_board)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
catkin_package(
  INCLUDE_DIRS
  navigator_kill_board
  LIBRARIES
  navigator_kill_board
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  std_srvs
  DEPENDS)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files
include_directories(navigator_kill_board ${catkin_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME} navigator_kill_board/heartbeat_server.cpp
                            navigator_kill_board/simulated_kill_board.cpp)

# Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Example executable for heartbeat server
add_executable(heartbeat_server_node nodes/heartbeat_server_node.cpp)

target_link_libraries(heartbeat_server_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# Example executable for simulated kill board
add_executable(simulated_kill_board_node nodes/simulated_kill_board_node.cpp)

target_link_libraries(simulated_kill_board_node ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

# Mark executables for installation
install(
  TARGETS ${PROJECT_NAME} heartbeat_server_node simulated_kill_board_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY navigator_kill_board/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}_test test/test_navigator_kill_board.launch
                    test/test_navigator_kill_board.cpp)

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}
                        ${catkin_LIBRARIES})
endif()

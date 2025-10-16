cmake_minimum_required(VERSION 3.8)
project(navigator_kill_board)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files
include_directories(include)

# Declare a C++ library
add_library(${PROJECT_NAME} src/heartbeat_server.cpp
                            src/simulated_kill_board.cpp)

# Specify dependencies for the library
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs)

# Example executable for heartbeat server
add_executable(heartbeat_server_node src/heartbeat_server_node.cpp)

target_link_libraries(heartbeat_server_node ${PROJECT_NAME})
ament_target_dependencies(heartbeat_server_node rclcpp std_msgs)

# Example executable for simulated kill board
add_executable(simulated_kill_board_node src/simulated_kill_board_node.cpp)

target_link_libraries(simulated_kill_board_node ${PROJECT_NAME})
ament_target_dependencies(simulated_kill_board_node rclcpp std_msgs std_srvs)

# ##############################################################################
# Install ##
# ##############################################################################

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install executables
install(TARGETS heartbeat_server_node simulated_kill_board_node
        DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING
  PATTERN "*.h")

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Add gtest based cpp tests
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/test_navigator_kill_board.cpp)

  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs std_srvs)
endif()

# Export package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs std_srvs)

ament_package()

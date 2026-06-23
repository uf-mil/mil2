cmake_minimum_required(VERSION 3.8)
project(subjugator_thruster_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(subjugator_msgs REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(thruster_manager src/thruster_manager.cpp)
target_link_libraries(thruster_manager rclcpp::rclcpp ${geometry_msgs_TARGETS}
                      ${subjugator_msgs_TARGETS} Eigen3::Eigen)
target_include_directories(thruster_manager PRIVATE include)

add_executable(forward_to_sim src/forward_to_sim.cpp)
ament_target_dependencies(forward_to_sim rclcpp subjugator_msgs)
target_include_directories(forward_to_sim PRIVATE include)

install(TARGETS thruster_manager forward_to_sim DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# include/ already contains the subjugator_thruster_manager/ subdir, so install
# its contents straight into include/ (not include/${PROJECT_NAME}, which would
# double-nest the path for downstream consumers of the exported header).
install(DIRECTORY include/ DESTINATION include)

# Export the headers (lut.h thrust curve) so other packages (e.g. the Gazebo
# thruster plugin) can share the same force<->effort model.
ament_export_include_directories(include)

ament_package()

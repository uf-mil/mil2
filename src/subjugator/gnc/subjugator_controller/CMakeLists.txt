cmake_minimum_required(VERSION 3.8)
project(subjugator_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(subjugator_controller src/node.cpp src/controller.cpp)
ament_target_dependencies(subjugator_controller rclcpp nav_msgs control_toolbox)
target_include_directories(subjugator_controller PRIVATE ${EIGEN3_INCLUDE_DIR} include)

install(TARGETS
  subjugator_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pcl_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mil_msgs REQUIRED)

# PCL stuff
find_package(PCL 1.2 REQUIRED)

# OPENCV stuff
find_package(OpenCV 4.0 REQUIRED)

# include directories
include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# collecting nesccary files
file(GLOB SOURCES src/*.cpp)

# creating library called pcl_stack
add_library(pcl_stack ${SOURCES})

# link all relative librarues

target_link_libraries(pcl_stack ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# adds dependencies to the library

ament_target_dependencies(
  pcl_stack
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  cv_bridge
  mil_msgs)

# create an exec that runs everything

add_executable(pcl_stack_node nodes/main.cpp)
target_link_libraries(pcl_stack_node pcl_stack)
add_dependencies(pcl_stack_node pcl_stack)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

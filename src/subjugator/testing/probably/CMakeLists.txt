cmake_minimum_required(VERSION 3.8)
project(probably)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mrpt-slam REQUIRED)
find_package(yolo_msgs REQUIRED)

add_executable(ekf src/ekf.cpp)
target_link_libraries(ekf rclcpp::rclcpp mrpt::slam ${yolo_msgs_TARGETS})
target_include_directories(ekf PRIVATE include/probably)
target_precompile_headers(ekf PRIVATE include/probably/pch.h)

install(TARGETS ekf
	DESTINATION lib/probably)

ament_package()

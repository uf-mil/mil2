cmake_minimum_required(VERSION 3.8)
project(magnetic_compensation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# create the component library
add_library(
  ${PROJECT_NAME} SHARED src/component.cpp # adjust if your source file is named
                                           # differently
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_eigen
  Eigen3)

rclcpp_components_register_nodes(
  ${PROJECT_NAME} "mil::magnetic_compensation::HardsoftCompensator")

# install lib
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(rclcpp rclcpp_components sensor_msgs tf2 tf2_eigen
                          Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests test/test_basic_shift.py)
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(
      ${_test_name}
      ${_test_path}
      APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT
      60
      WORKING_DIRECTORY
      ${CMAKE_SOURCE_DIR})
  endforeach()
endif()

ament_package()

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Install project launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install project configuration files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install the pure-Python calibration module (importable as
# magnetic_compensation.ellipsoid by the config generator and the tests)
ament_python_install_package(${PROJECT_NAME})

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
  # C++ unit tests: correction math + parameter validation (compensator.hpp)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_compensator test/test_compensator.cpp)
  target_include_directories(test_compensator
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_compensator Eigen3::Eigen)

  # Pure-math Python tests for the ellipsoid fit (numpy/scipy only)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ellipsoid test/test_ellipsoid.py TIMEOUT 60
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

  # End-to-end launch tests: live correction path + constructor param validation
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_basic_shift.py TIMEOUT 60)
  add_launch_test(test/test_param_validation.py TIMEOUT 60)
endif()

ament_package()

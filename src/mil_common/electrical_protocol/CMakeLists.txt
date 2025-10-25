cmake_minimum_required(VERSION 3.5)
project(electrical_protocol)

# Default to C++20 (for better constexpr support)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

# Standard deps
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Self deps
find_package(mil_tools REQUIRED)

# External deps
find_package(backward_ros REQUIRED)

# Define your library target
add_library(${PROJECT_NAME} src/driver.cpp)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})
include_directories(include)
# ament_target_dependencies(${PROJECT_NAME} rclcpp)
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Add cppystruct library from ext folder Set the path to the external library
set(CPPYSTRUCT_LIB "${CMAKE_SOURCE_DIR}/../../../ext/cppystruct")
message(STATUS "CPPYSTRUCT_LIB INCLUDE PATH: ${CPPYSTRUCT_LIB}/include")

# Include cppystruct header files
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CPPYSTRUCT_LIB}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/../CPPYSTRUCT/include>
         $<INSTALL_INTERFACE:include>)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include DESTINATION include/${PROJECT_NAME})

install(PROGRAMS test/calculator_device.py test/calculator_device_noros.py
        DESTINATION lib/${PROJECT_NAME})

# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)
# ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
# ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests test/test_simulated_basic.py test/test_simulated_noros.py)
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_packet_test test/test_packet.cpp)
  target_include_directories(
    ${PROJECT_NAME}_packet_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_packet_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_driver_test test/test_driver.cpp)
  target_include_directories(
    ${PROJECT_NAME}_driver_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include> ${mil_tools_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_driver_test ${PROJECT_NAME}
                        mil_tools::mil_tools)

endif()

ament_package()

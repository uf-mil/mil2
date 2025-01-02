cmake_minimum_required(VERSION 3.5)
project(electrical_protocol)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# include_directories(include ${rclcpp_INCLUDE_DIRS})
# link_directories(${rclcpp_LIBRARY_DIRS})

# Add a c++ library
# add_library(${PROJECT_NAME} src/some_file.cpp)
# target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})
#
# # Install C++ headers
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.hpp")

# Install python scripts
# install(PROGRAMS test/calculator_device.py DESTINATION bin)
install(PROGRAMS
  test/calculator_device.py
  DESTINATION lib/${PROJECT_NAME} )

# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)
# ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
# ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/test_simulated_basic.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()

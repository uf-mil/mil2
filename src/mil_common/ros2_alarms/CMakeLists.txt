cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ros2_alarms)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_alarms_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Declare a C++ library
set(ROS_ALARMS_SRCS
  src/${PROJECT_NAME}/broadcaster.cpp
  src/${PROJECT_NAME}/alarm_proxy.cpp
  src/${PROJECT_NAME}/heartbeat_monitor.cpp
)

add_library(${PROJECT_NAME} SHARED ${ROS_ALARMS_SRCS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ros2_alarms_msgs
  std_msgs
)

# Set include directories for C++ files
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install the library and include directories
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Test configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # C++ test files
  set(CPP_TESTFILES
    test/rclcpp/alarm_proxy_test.cpp
  )

  foreach(_test_path ${CPP_TESTFILES})
    get_filename_component(_test_name ${_test_path} NAME_WE)

    # Add the C++ test executable
    add_executable(${_test_name} ${_test_path})

    # Link with the ros2_alarms library
    target_link_libraries(${_test_name} ${PROJECT_NAME} gtest gmock)
    
    # Add dependencies
    ament_target_dependencies(${_test_name}
      rclcpp
      ros2_alarms_msgs
      std_msgs
    )

    # Include the necessary directories for tests
    target_include_directories(${_test_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    # Register the test with CTest
    add_test(NAME ${_test_name} COMMAND ${_test_name})
  endforeach()


  set(_pytest_tests
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

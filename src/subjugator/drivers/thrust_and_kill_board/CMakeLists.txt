cmake_minimum_required(VERSION 3.8)
project(thrust_and_kill_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(electrical_protocol REQUIRED)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# Install programs
install(PROGRAMS thrust_and_kill_board/driver.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake_python)

ament_package()

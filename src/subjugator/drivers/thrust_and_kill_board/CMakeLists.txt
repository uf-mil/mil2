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
install(PROGRAMS
  thrust_and_kill_board/driver.py
  thrust_and_kill_board/adrian.py
  DESTINATION lib/${PROJECT_NAME} )

ament_export_dependencies(ament_cmake_python)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

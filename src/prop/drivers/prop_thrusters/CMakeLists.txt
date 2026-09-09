cmake_minimum_required(VERSION 3.8)
project(prop_thrusters)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/thrusters.py scripts/thruster_test.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND
    AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_uncrustify
    ament_cmake_lint_cmake)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

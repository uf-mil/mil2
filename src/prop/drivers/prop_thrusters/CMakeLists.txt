cmake_minimum_required(VERSION 3.8)
project(prop_thrusters)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/thrusters.py scripts/thruster_test.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

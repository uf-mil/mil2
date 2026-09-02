cmake_minimum_required(VERSION 3.8)
project(prop_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

# Applies the thrust that sim_thrusters.py works out onto the hull.
add_library(PropThrusters SHARED src/PropThrusters.cc)
target_include_directories(PropThrusters PRIVATE include)
target_link_libraries(PropThrusters gz-sim8::gz-sim8)

install(TARGETS PropThrusters LIBRARY DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sim_thrusters.py scripts/sim_sensors.py
        DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

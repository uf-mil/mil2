cmake_minimum_required(VERSION 3.8)
project(prop_path_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/prop_path_planner.txt"
     "Prop path planner\n")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

file(GLOB_RECURSE PROP_PATH_PLANNER_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(prop_path_planner ${PROP_PATH_PLANNER_SOURCES})

ament_target_dependencies(prop_path_planner rclcpp nav_msgs)

target_include_directories(prop_path_planner
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS prop_path_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_package()

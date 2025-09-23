cmake_minimum_required(VERSION 3.14)
project(mission_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mission_planner_node mission_planner_node.cpp)
target_include_directories(mission_planner_node PUBLIC include)
ament_target_dependencies(mission_planner_node rclcpp)

install(TARGETS mission_planner_node RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

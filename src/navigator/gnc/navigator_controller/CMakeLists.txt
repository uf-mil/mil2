cmake_minimum_required(VERSION 3.8)
project(navigator_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(navigator_msgs REQUIRED)
find_package(ftxui REQUIRED)

include_directories(include)

set(DEPS rclcpp geometry_msgs std_msgs navigator_msgs)

add_executable(server_node src/server_node.cpp)
ament_target_dependencies(server_node ${DEPS})

add_executable(keyboard_client_node src/keyboard_client_node.cpp)
ament_target_dependencies(keyboard_client_node ${DEPS})
target_link_libraries(keyboard_client_node ftxui::component ftxui::dom
                      ftxui::screen)

install(TARGETS server_node keyboard_client_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()

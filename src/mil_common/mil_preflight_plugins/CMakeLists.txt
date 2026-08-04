cmake_minimum_required(VERSION 3.8)
project(mil_preflight_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(subjugator_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mil_preflight REQUIRED)

add_library(topic_plugin SHARED src/topic_plugin.cpp)

ament_target_dependencies(topic_plugin rclcpp pluginlib mil_preflight)

add_library(setup_plugin SHARED src/setup_plugin.cpp)

ament_target_dependencies(setup_plugin rclcpp pluginlib mil_preflight)

add_library(node_plugin SHARED src/node_plugin.cpp)

ament_target_dependencies(node_plugin rclcpp pluginlib mil_preflight)

add_library(actuator_plugin SHARED src/actuator_plugin.cpp)

ament_target_dependencies(actuator_plugin rclcpp subjugator_msgs pluginlib
                          mil_preflight)

install(TARGETS topic_plugin setup_plugin node_plugin actuator_plugin
        LIBRARY DESTINATION lib)

pluginlib_export_plugin_description_file(mil_preflight plugins.xml)

ament_export_libraries(topic_plugin setup_plugin node_plugin actuator_plugin)

ament_package()

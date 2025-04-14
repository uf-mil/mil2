cmake_minimum_required(VERSION 3.8)
project(mil_preflight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(subjugator_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono filesystem system)

set(_OLD_CMAKE_POSITION_INDEPENDENT_CODE ${CMAKE_POSITION_INDEPENDENT_CODE})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(FTXUI_BINARY_DIR ${CMAKE_BINARY_DIR}/ftxui-build)
set(FTXUI_SOURCE_DIR ${CMAKE_SOURCE_DIR}/../../../ext/FTXUI)
add_subdirectory(${FTXUI_SOURCE_DIR} ${FTXUI_BINARY_DIR})

set(CMAKE_POSITION_INDEPENDENT_CODE ${_OLD_CMAKE_POSITION_INDEPENDENT_CODE})

add_executable(${PROJECT_NAME} src/frontend.cpp)
include_directories(include ${ftxui_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} Boost::system Boost::filesystem)

add_executable(mil_preflight_backend src/backend.cpp)

ament_target_dependencies(mil_preflight_backend rclcpp std_msgs Boost)

install(TARGETS ${PROJECT_NAME} mil_preflight_backend
        DESTINATION bin # Install to install/bin/
)

install(FILES cfg/config.json DESTINATION cfg)

add_library(ftx_ui SHARED src/uis/ftxui/ui.cpp src/uis/ftxui/widgets.cpp)

target_link_libraries(
  ftx_ui
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component Boost::thread Boost::chrono Boost::filesystem
          Boost::system)

add_library(topic_plugin SHARED src/plugins/topic_plugin.cpp)

ament_target_dependencies(topic_plugin rclcpp Boost)

add_library(setup_plugin SHARED src/plugins/setup_plugin.cpp)

ament_target_dependencies(setup_plugin rclcpp Boost)

add_library(node_plugin SHARED src/plugins/node_plugin.cpp)

ament_target_dependencies(node_plugin rclcpp Boost)

add_library(actuator_plugin SHARED src/plugins/actuator_plugin.cpp)

ament_target_dependencies(actuator_plugin rclcpp subjugator_msgs Boost)

install(TARGETS topic_plugin setup_plugin node_plugin actuator_plugin ftx_ui
        LIBRARY DESTINATION lib)

ament_package()

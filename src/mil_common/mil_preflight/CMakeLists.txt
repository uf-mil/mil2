cmake_minimum_required(VERSION 3.8)
project(mil_preflight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono filesystem system)
find_package(ftxui REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED src/frontend.cpp)
add_executable(${PROJECT_NAME} src/mil_preflight.cpp src/tui.cpp)
include_directories(include ${ftxui_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

target_link_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  Boost::system
  Boost::filesystem
  ftxui::screen
  ftxui::dom
  ftxui::component)

add_executable(mil_preflight_backend src/backend.cpp)

ament_target_dependencies(mil_preflight_backend rclcpp Boost pluginlib)

install(TARGETS ${PROJECT_NAME} mil_preflight_backend
        DESTINATION bin # Install to install/bin/
)

install(
  DIRECTORY cfg/
  DESTINATION bin
  FILES_MATCHING
  PATTERN "*")

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)

install(TARGETS ${PROJECT_NAME}_lib LIBRARY DESTINATION lib)

ament_package()

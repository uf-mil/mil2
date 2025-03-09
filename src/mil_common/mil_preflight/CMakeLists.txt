cmake_minimum_required(VERSION 3.8)
project(mil_preflight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS json thread chrono filesystem system)

include(FetchContent)

FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v5.0.0
)

FetchContent_GetProperties(ftxui)
if(NOT ftxui_POPULATED)
  FetchContent_Populate(ftxui)

  # Force FTXUI to build shared libraries instead of static
  set(FTXUI_BUILD_SHARED_LIBS ON CACHE BOOL "Build FTXUI as shared libraries" FORCE)
  add_subdirectory(${ftxui_SOURCE_DIR} ${ftxui_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

FetchContent_MakeAvailable(ftxui)

add_executable(${PROJECT_NAME} src/frontend.cpp)
include_directories(include ${ftxui_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  Boost::system
  Boost::filesystem
)

add_executable(mil_preflight_backend src/backend.cpp)

ament_target_dependencies(mil_preflight_backend
  rclcpp
  std_msgs
  Boost
)

install(TARGETS ${PROJECT_NAME} mil_preflight_backend
  DESTINATION bin  # Install to install/bin/
)

install(FILES cfg/config.json
  DESTINATION cfg
)

add_library(ftx_ui SHARED src/uis/ftxui/ui.cpp src/uis/ftxui/widgets.cpp)

target_link_libraries(ftx_ui
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component
  Boost::json
  Boost::thread
  Boost::chrono
  Boost::filesystem
  Boost::system
)

add_library(topic_plugin SHARED src/plugins/topic_plugin.cpp)

ament_target_dependencies(topic_plugin
  rclcpp
  Boost
)

add_library(setup_plugin SHARED src/plugins/setup_plugin.cpp)

ament_target_dependencies(setup_plugin
  rclcpp
  Boost
)

add_library(node_plugin SHARED src/plugins/node_plugin.cpp)

ament_target_dependencies(node_plugin
  rclcpp
  Boost
)

add_library(actuator_plugin SHARED src/plugins/actuator_plugin.cpp)

ament_target_dependencies(actuator_plugin
  rclcpp
  Boost
)

install(TARGETS topic_plugin setup_plugin node_plugin actuator_plugin ftx_ui
  LIBRARY DESTINATION lib
)

# add_executable(ftxui_test src/test.cpp)
# target_link_libraries(ftxui_test
#   PRIVATE ftxui::screen
#   PRIVATE ftxui::dom
#   PRIVATE ftxui::component
#   Boost::json
#   Boost::thread
#   Boost::chrono
# )

# install(TARGETS ftxui_test
#   DESTINATION bin  # Install to install/bin/
# )

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

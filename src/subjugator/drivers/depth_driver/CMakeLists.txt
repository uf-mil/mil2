cmake_minimum_required(VERSION 3.8)
project(depth_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(depth_driver src/depth_driver.cpp)
target_include_directories(
  depth_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(depth_driver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(depth_driver geometry_msgs "rclcpp" "std_msgs" "nav_msgs"
                          "mil_msgs")

install(TARGETS depth_driver DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()

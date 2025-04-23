cmake_minimum_required(VERSION 3.8)
project(subjugator_path_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mil_tools REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

add_executable(subjugator_path_planner src/node.cpp src/PathPlanner.cpp)
ament_target_dependencies(
  subjugator_path_planner
  Eigen3
  mil_tools
  nav_msgs
  rclcpp
  tf2_geometry_msgs
  mil_tools)
target_include_directories(subjugator_path_planner
                           PUBLIC ${mil_tools_INCLUDE_DIRS} include)

install(TARGETS subjugator_path_planner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)

endif()

ament_package()

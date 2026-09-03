cmake_minimum_required(VERSION 3.8)
project(pcd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# pcl_filter node
add_executable(pcl_filter_node src/pcl_filter_node.cpp)
ament_target_dependencies(pcl_filter_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcl_filter_node ${PCL_LIBRARIES})

# pcl_clustering node
add_executable(pcl_clustering_node src/pcl_clustering_node.cpp)
ament_target_dependencies(pcl_clustering_node rclcpp sensor_msgs
                          visualization_msgs pcl_conversions)
target_link_libraries(pcl_clustering_node ${PCL_LIBRARIES})

# Install
install(TARGETS pcl_filter_node pcl_clustering_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

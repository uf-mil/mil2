cmake_minimum_required(VERSION 3.8)
project(online_bagger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mil_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system program_options)
find_package(ftxui REQUIRED COMPONENTS screen dom component)

include_directories(include ${Boost_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_server src/server.cpp)
add_executable(${PROJECT_NAME}_client src/client.cpp)

ament_target_dependencies(${PROJECT_NAME}_server 
        rclcpp 
        mil_msgs
        std_msgs
        rclcpp_action
        rosbag2_cpp
        Boost
)

ament_target_dependencies(${PROJECT_NAME}_client
        rclcpp 
        mil_msgs
        std_msgs
        rclcpp_action
        Boost
)

target_link_libraries(${PROJECT_NAME}_client
ftxui::screen
ftxui::dom
ftxui::component
)

install(TARGETS ${PROJECT_NAME}_server ${PROJECT_NAME}_client
        DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

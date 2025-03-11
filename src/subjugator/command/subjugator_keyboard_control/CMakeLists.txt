cmake_minimum_required(VERSION 3.8)
project(subjugator_keyboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(subjugator_keyboard_control src/node.cpp src/SubjugatorKeyboardControl.cpp)

target_include_directories(subjugator_keyboard_control PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(subjugator_keyboard_control PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
        subjugator_keyboard_control
        rclcpp
        geometry_msgs
)

install(TARGETS subjugator_keyboard_control
        DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(teleop_twist_keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)

target_include_directories(teleop_twist_keyboard PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(teleop_twist_keyboard PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(teleop_twist_keyboard rclcpp geometry_msgs)

# Should this be installing with or without quotes?
ament_target_dependencies(
        teleop_twist_keyboard
        rclcpp
        geometry_msgs
        std_msgs
        nav_msgs
        mil_msgs
)

install(TARGETS teleop_twist_keyboard
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()

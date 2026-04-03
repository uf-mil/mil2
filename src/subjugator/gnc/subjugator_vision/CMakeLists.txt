cmake_minimum_required(VERSION 3.8)
project(subjugator_vision)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(mil_msgs REQUIRED)

include_directories(include)

# Python scripts
install(PROGRAMS scripts/save_down_images.py scripts/save_front_images.py
                 scripts/swap_yolo_model.py DESTINATION lib/${PROJECT_NAME})

# YOLO models
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()

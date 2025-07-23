cmake_minimum_required(VERSION 3.8)
project(subjugator_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    src/run_yolo.py
    src/save/save_down_images.py
    src/save/save_front_images.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models/
  DESTINATION lib/${PROJECT_NAME}/models
)

ament_package()

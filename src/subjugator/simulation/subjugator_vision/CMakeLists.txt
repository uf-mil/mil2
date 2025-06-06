cmake_minimum_required(VERSION 3.8)
project(subjugator_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python script directly
install(
  PROGRAMS
    save_front_images.py
    save_down_images.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

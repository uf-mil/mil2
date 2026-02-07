cmake_minimum_required(VERSION 3.8)
project(subjugator_vision)

# find ament packages we rely on
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED) # needed for header path if you later add C++
find_package(mil_msgs REQUIRED) # message header path

# install Python scripts (marked executable in on-disk perms)
install(PROGRAMS src/swap_yolo_model.py src/save/save_down_images.py
                 src/save/save_front_images.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()

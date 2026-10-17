cmake_minimum_required(VERSION 3.8)
project(object_classifier)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/frame_capture_node.py DESTINATION lib/${PROJECT_NAME})

# helper modules imported by the nodes; installed next to them so imports work
install(FILES src/camera.py src/utils.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

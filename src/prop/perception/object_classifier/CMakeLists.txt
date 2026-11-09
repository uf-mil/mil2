cmake_minimum_required(VERSION 3.8)
project(object_classifier)

find_package(ament_cmake REQUIRED)

install(PROGRAMS src/frame_capture_node.py src/detection_node.py
        DESTINATION lib/${PROJECT_NAME})

install(FILES src/camera.py src/utils.py src/detection.py src/tracking.py
              src/localization.py src/undistort.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

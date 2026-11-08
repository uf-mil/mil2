cmake_minimum_required(VERSION 3.8)
project(prop_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Plain scripts rather than a Python package, so python3 main.py still works
# from here. The nodes import their neighbours by file name, so they all go in
# the same directory.
install(PROGRAMS beacon_node.py camera_node.py vision_node.py
        DESTINATION lib/${PROJECT_NAME})
install(
  FILES camera.py
        config.py
        detection.py
        localization.py
        tracking.py
        undistort.py
        utils.py
        camera_params.json
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# prop_vision/ is the importable part: calibration and beacons for the nodes
# here, dock_perception for prop_missions.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_beacons test/test_beacons.py)
endif()

ament_package()

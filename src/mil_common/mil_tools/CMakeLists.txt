cmake_minimum_required(VERSION 3.5)
project(mil_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(SOURCE_FILES src/geometry/Rotation.cpp src/fs/path.cpp src/os.cpp
                 src/os/FileDescriptor.cpp src/string.cpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3 tf2_geometry_msgs)
ament_export_dependencies(rclcpp Eigen3 tf2_geometry_msgs)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_fs_path_test test/fs/path.cpp)
  target_include_directories(
    ${PROJECT_NAME}_fs_path_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_fs_path_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_string_test test/string.cpp)
  target_include_directories(
    ${PROJECT_NAME}_string_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_string_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_os_test test/os.cpp)
  target_include_directories(
    ${PROJECT_NAME}_os_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_os_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_random_test test/random.cpp)
  target_include_directories(
    ${PROJECT_NAME}_random_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_random_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_overloaded_test test/overloaded.cpp)
  target_include_directories(
    ${PROJECT_NAME}_overloaded_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_overloaded_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_itertools_test test/itertools.cpp)
  target_include_directories(
    ${PROJECT_NAME}_itertools_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_itertools_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_rotation_test test/geometry/Rotation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_rotation_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_rotation_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_math_test test/math.cpp)
  target_include_directories(
    ${PROJECT_NAME}_math_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_math_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_slerp_test test/geometry/Slerp.cpp)
  target_include_directories(
    ${PROJECT_NAME}_slerp_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_slerp_test ${PROJECT_NAME})
endif()

ament_package()

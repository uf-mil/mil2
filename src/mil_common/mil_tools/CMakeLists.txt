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

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}
    src/fs/path.cpp
    src/os.cpp
    src/os/FileDescriptor.cpp
    src/string.cpp
    src/hardware/system_info.cpp
)
include_directories(include)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
    DIRECTORY include
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_fs_path_test test/fs/path.cpp)
  target_include_directories(${PROJECT_NAME}_fs_path_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_fs_path_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_string_test test/string.cpp)
  target_include_directories(${PROJECT_NAME}_string_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_string_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_os_test test/os.cpp)
  target_include_directories(${PROJECT_NAME}_os_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_os_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_random_test test/random.cpp)
  target_include_directories(${PROJECT_NAME}_random_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_random_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_overloaded_test test/overloaded.cpp)
  target_include_directories(${PROJECT_NAME}_overloaded_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_overloaded_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_itertools_test test/itertools.cpp)
  target_include_directories(${PROJECT_NAME}_itertools_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_itertools_test ${PROJECT_NAME})
  
  ament_add_gtest(${PROJECT_NAME}_cpu_architecture_test test/hardware/system_info.cpp)
  target_include_directories(${PROJECT_NAME}_cpu_architecture_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_cpu_architecture_test ${PROJECT_NAME})
endif()
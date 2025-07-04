cmake_minimum_required(VERSION 3.8)
project(mil_buoyancy_fix)

# Find necessary ROS 2 and Gazebo packages
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# This will be compiled into libcustom_buoyancy_plugin.so
add_library(custom_buoyancy_plugin SHARED
  src/Buoyancy.cc
)

# Tell the compiler where to find the header files
target_include_directories(custom_buoyancy_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link your plugin against the necessary Gazebo libraries
target_link_libraries(custom_buoyancy_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

# Use the more robust install command to ensure environment hooks are created
install(TARGETS custom_buoyancy_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the package.xml file
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Register the package with the ament index
ament_package()

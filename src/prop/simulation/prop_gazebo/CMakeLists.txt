cmake_minimum_required(VERSION 3.8)
project(prop_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

# Applies the thrust that sim_thrusters.py works out onto the hull.
add_library(PropThrusters SHARED src/PropThrusters.cc)
target_include_directories(PropThrusters PRIVATE include)
target_link_libraries(PropThrusters gz-sim8::gz-sim8)

install(TARGETS PropThrusters LIBRARY DESTINATION lib/${PROJECT_NAME})

# The models are written as xacro so the boat and the bare lidar rig can share
# one description of the VLP-16. Gazebo only reads SDF, so expand them here.
find_program(XACRO_EXECUTABLE NAMES xacro REQUIRED)

file(GLOB xacro_includes CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/xacro/components/*.xacro"
     "${CMAKE_CURRENT_SOURCE_DIR}/xacro/macros/*.xacro")

set(generated_models "")
foreach(model prop lidar_platform)
  set(model_dir "${CMAKE_CURRENT_BINARY_DIR}/models/${model}")
  set(model_sdf "${model_dir}/model.sdf")
  add_custom_command(
    OUTPUT "${model_sdf}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${model_dir}"
    COMMAND
      ${XACRO_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/xacro/${model}.sdf.xacro"
      -o "${model_sdf}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/xacro/${model}.sdf.xacro"
            ${xacro_includes}
    COMMENT "xacro ${model}.sdf.xacro -> models/${model}/model.sdf"
    VERBATIM)
  list(APPEND generated_models "${model_sdf}")
  install(FILES "${model_sdf}"
          DESTINATION share/${PROJECT_NAME}/models/${model})
endforeach()

add_custom_target(expand_models ALL DEPENDS ${generated_models})

# models/ carries the model.config files; model.sdf is installed above.
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sim_thrusters.py scripts/sim_sensors.py
        DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND
    AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_uncrustify
    ament_cmake_lint_cmake)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

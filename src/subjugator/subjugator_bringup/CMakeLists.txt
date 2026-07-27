cmake_minimum_required(VERSION 3.5)

project(subjugator_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the python module so `from subjugator_bringup.task_runner import ...`
# works
ament_python_install_package(${PROJECT_NAME})

# run_task entrypoint: `ros2 run subjugator_bringup run_task 5`
install(
  PROGRAMS scripts/run_task.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_task)

find_package(subjugator_description REQUIRED)
find_package(subjugator_gazebo REQUIRED)

# Install project launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install project configuration files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
      test/test_tasks_table.py
      test/test_cli.py
      test/test_btbudget.py
      test/test_btlog.py
      test/test_collectors.py
      test/test_probes.py
      test/test_task5_scorer.py
      test/test_basket_constants.py
      test/test_report.py
      test/test_lifecycle.py
      test/test_runner.py
      test/test_settle_gate.py)
  # The list above names every test file the run_task harness will own. Files
  # that a later task has not created yet are skipped rather than breaking the
  # configure step; each one starts running the moment it lands on disk.
  foreach(_test_path ${_pytest_tests})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_test_path})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(
        ${_test_name}
        ${_test_path}
        APPEND_ENV
        PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT
        120
        WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR})
    else()
      message(STATUS "run_task: skipping ${_test_path} (not yet created)")
    endif()
  endforeach()
endif()

ament_package()

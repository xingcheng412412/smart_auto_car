cmake_minimum_required(VERSION 3.8)
project(lawn_mower_recorder)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RecordStatus.msg"
  "srv/ProcessSwitch.srv"
  "srv/RecordMarkPoint.srv"
  DEPENDENCIES std_msgs
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES
  lawn_mower_recorder/recorder_core.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(PROGRAMS
  lawn_mower_recorder/recorder_enhanced.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(_PYTEST_SAVED "$ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD}")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_package_contract test/test_package_contract.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_recorder_core test/test_recorder_core.py
    ENV
      "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
  ament_add_pytest_test(test_launch_parameters test/test_launch_parameters.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "${_PYTEST_SAVED}")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

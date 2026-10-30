cmake_minimum_required(VERSION 3.8)
project(lawn_mower_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_client REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathStatus.msg"
  "srv/MowerTrackControl.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(tracker_path src/path_loader.cpp)
target_include_directories(tracker_path PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tracker_core src/tracker_core.cpp src/obstacle_planner.cpp)
target_include_directories(tracker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(vehicle_command src/vehicle_command.cpp)
target_include_directories(vehicle_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vehicle_command can_client)

add_library(tracker_node src/tracker_node.cpp src/obstacle_cloud.cpp)
target_include_directories(tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tracker_node can_client geometry_msgs rclcpp sensor_msgs nav_msgs)
target_link_libraries(tracker_node
  tracker_path tracker_core vehicle_command "${cpp_typesupport_target}"
)

add_executable(lawn_mower_tracker_node src/main.cpp)
target_include_directories(lawn_mower_tracker_node PRIVATE include)
ament_target_dependencies(lawn_mower_tracker_node rclcpp)
target_link_libraries(lawn_mower_tracker_node tracker_node)

install(TARGETS
  tracker_path tracker_core vehicle_command tracker_node lawn_mower_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(_PYTEST_DISABLE_PLUGIN_AUTOLOAD_SAVED "$ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD}")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_path_loader test/test_path_loader.cpp)
  target_include_directories(test_path_loader PRIVATE include)
  target_link_libraries(test_path_loader tracker_path)
  ament_add_gtest(test_obstacle_planner test/test_obstacle_planner.cpp)
  target_include_directories(test_obstacle_planner PRIVATE include)
  target_link_libraries(test_obstacle_planner tracker_core)
  ament_add_gtest(test_tracker_core test/test_tracker_core.cpp)
  target_include_directories(test_tracker_core PRIVATE include)
  target_compile_definitions(test_tracker_core PRIVATE
    LAWN_MOWER_WORKSPACE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../.."
  )
  target_link_libraries(test_tracker_core tracker_core tracker_path)
  ament_add_gtest(test_vehicle_command test/test_vehicle_command.cpp)
  target_include_directories(test_vehicle_command PRIVATE include)
  ament_target_dependencies(test_vehicle_command can_client)
  target_link_libraries(test_vehicle_command vehicle_command)
  ament_add_gtest(test_tracker_node test/test_tracker_node.cpp)
  target_include_directories(test_tracker_node PRIVATE include)
  ament_target_dependencies(test_tracker_node can_client geometry_msgs rclcpp sensor_msgs nav_msgs)
  target_link_libraries(test_tracker_node tracker_node "${cpp_typesupport_target}")
  set_tests_properties(test_tracker_node PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp/lawn_mower_tracker_test_logs"
  )
  ament_add_pytest_test(test_package_contract
    test/test_package_contract.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(test_launch_parameters
    test/test_launch_parameters.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(test_recorded_files_default
    test/test_recorded_files_default.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "${_PYTEST_DISABLE_PLUGIN_AUTOLOAD_SAVED}")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

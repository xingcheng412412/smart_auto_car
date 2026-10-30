cmake_minimum_required(VERSION 3.8)
project(can_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleCmd.msg"
  "msg/VehicleFeedback.msg"
  "msg/CANFrame.msg"
  DEPENDENCIES std_msgs
)

add_executable(vehicle_core src/vehicle_core.cpp)
target_include_directories(vehicle_core PRIVATE include)
ament_target_dependencies(vehicle_core rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vehicle_core "${cpp_typesupport_target}")

install(TARGETS vehicle_core
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
)

install(PROGRAMS
  upper_computer/app.py
  script/socketcan_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_watchdog test/test_command_watchdog.cpp)
  target_include_directories(test_command_watchdog PRIVATE include)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

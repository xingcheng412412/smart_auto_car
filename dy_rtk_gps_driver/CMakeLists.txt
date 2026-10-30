cmake_minimum_required(VERSION 3.8)
project(rtk_gps_driver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Simulation only needs the existing message ABI, not serial/navigation libraries.
option(RTK_INTERFACES_ONLY "Build only RTK message interfaces for simulation" OFF)
if(RTK_INTERFACES_ONLY)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RTKGPSLog.msg" DEPENDENCIES std_msgs)
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
  return()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(GeographicLib REQUIRED)

# Interface definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RTKGPSLog.msg"
  DEPENDENCIES std_msgs
)

# Library (without ROS message dependencies)
add_library(rtk_gps_driver_lib SHARED
  src/gnss_origin_storage.cpp
  src/nmea_parser.cpp
  src/pose_localizer.cpp
  src/serial_port.cpp
)

target_include_directories(rtk_gps_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Boost_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

ament_target_dependencies(rtk_gps_driver_lib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(rtk_gps_driver_lib
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  Threads::Threads
)

# Node executable (depends on messages)
add_executable(rtk_gps_node_cpp
  src/rtk_gps_driver_node.cpp
)

target_include_directories(rtk_gps_node_cpp PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${Boost_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

ament_target_dependencies(rtk_gps_node_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# Link the generated typesupport library
rosidl_get_typesupport_target(celsius_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(rtk_gps_node_cpp
  rtk_gps_driver_lib
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  Threads::Threads
  "${celsius_typesupport_target}"
)

# Install
install(TARGETS rtk_gps_driver_lib rtk_gps_node_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
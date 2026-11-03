cmake_minimum_required(VERSION 3.8)
project(topic_sensor_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(topic_sensor_bridge SHARED
  src/topic_sensor_hardware.cpp
)

target_include_directories(topic_sensor_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(topic_sensor_bridge
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  realtime_tools
)

pluginlib_export_plugin_description_file(hardware_interface topic_sensor_bridge.xml)

install(
  TARGETS topic_sensor_bridge
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(topic_sensor_bridge)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  realtime_tools
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(transcend_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)          # Float32MultiArray — replaces transcend_interfaces

add_library(transcend_hardware SHARED
  src/transcend_hardware_interface.cpp
)

target_include_directories(transcend_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(transcend_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs                               # was transcend_interfaces
)

pluginlib_export_plugin_description_file(
  hardware_interface transcend_hardware_plugin.xml)

install(TARGETS transcend_hardware
  EXPORT export_transcend_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES transcend_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(transcend_hardware)
ament_export_targets(export_transcend_hardware)

ament_package()
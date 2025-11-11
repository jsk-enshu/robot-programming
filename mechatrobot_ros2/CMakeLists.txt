cmake_minimum_required(VERSION 3.8)
project(mechatrobot_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
)

# Declare a C++ library for hardware interface plugin
add_library(mechatrobot_hardware_interface SHARED
  src/robot_control.cpp
)
ament_target_dependencies(mechatrobot_hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  hardware_interface
  pluginlib
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface mechatrobot_hardware_interface.xml)

# Install library
install(TARGETS
  mechatrobot_hardware_interface
  DESTINATION lib
)

# Install Python scripts
install(PROGRAMS
  scripts/motor-command-by-face.py
  scripts/led_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

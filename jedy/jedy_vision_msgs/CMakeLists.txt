cmake_minimum_required(VERSION 3.16)
project(jedy_vision_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Detection.msg
  msg/DetectionArray.msg
  msg/Gesture.msg
  srv/WhatDoYouSee.srv
  srv/ReadGesture.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

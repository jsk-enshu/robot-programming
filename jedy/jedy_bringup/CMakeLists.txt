cmake_minimum_required(VERSION 3.16)
project(jedy_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/jedy_stop scripts/jedy_wait_port
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

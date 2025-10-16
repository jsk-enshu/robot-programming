cmake_minimum_required(VERSION 3.5)
project(jedy_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/track_box_to_cmd_vel_0.py
    scripts/track_box_to_cmd_vel_1.py
    scripts/send_joint_position.py
    scripts/send_joint_position_action.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

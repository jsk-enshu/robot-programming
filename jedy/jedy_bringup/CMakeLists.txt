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

# The 2025 names (jedy_gazebo.launch.py legacy:=true) and the 2025 exercise
# scripts that use them, under their 2025 names (ros2 run jedy_bringup <x>.py).
install(
  PROGRAMS
    scripts/jedy_legacy_bridge
    scripts/send_joint_position.py
    scripts/send_joint_position_action.py
    scripts/track_box_to_cmd_vel_0.py
    scripts/track_box_to_cmd_vel_1.py
    scripts/checkerboard_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.16)
project(jedy_bringup)

find_package(ament_cmake REQUIRED)
# Gazebo Harmonic, for the system plugins in src/ (servo OFF, mimic joints).
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

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

# Python nodes installed without their .py extension. They use rclpy and ROS
# messages only, so the system python runs them.
#   jedy_sim_servo, jedy_sim_sounds: the Gazebo servo switch and motor sounds
#   jedy_wheel_odometry: wheel odometry (an estimate, not a measurement)
#   jedy_joy, jedy_joy_monitor, jedy_joy_buttons: driving from a gamepad
foreach(node
    jedy_sim_servo
    jedy_sim_sounds
    jedy_wheel_odometry
    jedy_joy
    jedy_joy_monitor
    jedy_joy_buttons)
  install(PROGRAMS scripts/${node}.py
          DESTINATION lib/${PROJECT_NAME} RENAME ${node})
endforeach()

# Motor sounds for jedy_sim_sounds (recorded from the real Jedy).
install(DIRECTORY sounds DESTINATION share/${PROJECT_NAME})

# The Python modules the nodes above import (jedy_sim.sim_servo,
# jedy_odometry.wheel_angle, jedy_teleop.deadband / .mapping). They are plain
# importable packages on PYTHONPATH through the env hook, so whatever
# interpreter runs the rest of the stack can import them.
install(DIRECTORY jedy_sim jedy_odometry jedy_teleop
        DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)

# Servo OFF in Gazebo. gz finds it through GZ_SIM_SYSTEM_PLUGIN_PATH (env hook).
add_library(jedy_servo_switch SHARED src/jedy_servo_switch.cpp)
target_link_libraries(jedy_servo_switch gz-sim::core gz-plugin::register)
install(TARGETS jedy_servo_switch LIBRARY DESTINATION lib)

# Mimic joints (the second gripper finger) in Gazebo: DART has no mimic
# constraint, so this system makes each follower track its leader.
add_library(jedy_mimic_joint SHARED src/jedy_mimic_joint.cpp)
target_link_libraries(jedy_mimic_joint gz-sim::core gz-plugin::register)
install(TARGETS jedy_mimic_joint LIBRARY DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/jedy_bringup.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_servo test/test_sim_servo.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_wheel_angle test/test_wheel_angle.py)
  ament_add_pytest_test(test_deadband test/test_deadband.py)
endif()

ament_package()

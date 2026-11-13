cmake_minimum_required(VERSION 3.16)
project(jedy_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_uv_virtualenv REQUIRED)
# Pin the venv interpreter to the ROS python: without it uv picks the newest
# python on the machine and --system-site-packages then exposes an
# ABI-mismatched numpy/rclpy.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Gazebo Harmonic, for the servo-off system plugin (src/jedy_servo_switch.cpp).
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# actuo (shared memory, collision gate) from the checkout actuo_ros was built
# from; requirements.in.in carries an @ACTUO_REQUIREMENT@ line for it.
find_package(actuo_ros REQUIRED)
actuo_requirement(ACTUO_REQUIREMENT EXTRAS safety)
configure_file(requirements.in.in "${CMAKE_CURRENT_BINARY_DIR}/requirements.in" @ONLY)
ament_generate_virtualenv(
  REQUIREMENTS "${CMAKE_CURRENT_BINARY_DIR}/requirements.in"
  PYTHON ${Python3_EXECUTABLE})

ament_uv_install_python_scripts(
  PROGRAMS
    scripts/jedy_fake_servicer.py
    scripts/jedy_sim_servo.py
    scripts/jedy_sim_sounds.py
  STRIP_PY_EXTENSION
)
# Motor sounds for jedy_sim_sounds (recorded from the real Jedy).
install(DIRECTORY sounds DESTINATION share/${PROJECT_NAME})

# Servo OFF in Gazebo. gz finds it through GZ_SIM_SYSTEM_PLUGIN_PATH (env hook).
add_library(jedy_servo_switch SHARED src/jedy_servo_switch.cpp)
target_link_libraries(jedy_servo_switch gz-sim::core gz-plugin::register)
install(TARGETS jedy_servo_switch LIBRARY DESTINATION lib)

# Mimic joints (the second gripper finger) in Gazebo: DART has no mimic
# constraint, so this system makes each follower track its leader.
add_library(jedy_mimic_joint SHARED src/jedy_mimic_joint.cpp)
target_link_libraries(jedy_mimic_joint gz-sim::core gz-plugin::register)
install(TARGETS jedy_mimic_joint LIBRARY DESTINATION lib)

# The simulators' servo switch (jedy_sim.sim_servo), shared by
# jedy_sim_servo; on PYTHONPATH through the env hook.
install(DIRECTORY jedy_sim DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/jedy_sim.dsv.in")

if(BUILD_TESTING)
  ament_uv_check_virtualenv()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_servo test/test_sim_servo.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()

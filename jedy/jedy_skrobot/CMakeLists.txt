cmake_minimum_required(VERSION 3.16)
project(jedy_skrobot)

find_package(ament_cmake REQUIRED)
find_package(ament_uv_virtualenv REQUIRED)
# Pin the venv to the ROS interpreter, or uv picks the newest python on the
# machine and --system-site-packages then hands it an ABI-mismatched
# numpy/rclpy (an ABI mismatch otherwise).
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_generate_virtualenv(PYTHON ${Python3_EXECUTABLE})

# jedy_skrobot/ (connect, JedyInterface) on the workspace's PYTHONPATH, so
# programs outside this package (e.g. students' own) can import it.
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

ament_uv_install_python_scripts(
  PROGRAMS
    scripts/jedy_interface.py
    # The exercise samples: scikit-robot ports of the 2025 EusLisp ones. Same
    # code for Gazebo and the real robot. Installed through the venv
    # wrapper because scikit-robot lives in the venv, not in the system python.
    scripts/enshu/sample_reset_pose.py
    scripts/enshu/sample_joint.py
    scripts/enshu/sample_ik.py
    scripts/enshu/keyboard_ik.py
    scripts/enshu/sample_servo.py
    scripts/enshu/sample_controllers.py
    scripts/enshu/checkerboard_pose.py
    scripts/enshu/color_clusters.py
    scripts/enshu/checkpoint_go_forward.py
    scripts/enshu/checkpoint_go_forward_imu.py
    scripts/enshu/track_box.py
    scripts/enshu/show_camera.py
    # Steady motions, e.g. to record the robot's sounds for the simulator.
    scripts/tools/drive_steady.py
    scripts/tools/swing_joint.py
  STRIP_PY_EXTENSION
)

# Imported by the script at runtime, not entry points of their own.
install(
  FILES
    scripts/jedy_fullbody_robot_interface.py
    scripts/servicer_robot_interface.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  ament_uv_check_virtualenv()
endif()

ament_package()

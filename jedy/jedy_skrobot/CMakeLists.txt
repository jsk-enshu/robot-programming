cmake_minimum_required(VERSION 3.16)
project(jedy_skrobot)

find_package(ament_cmake REQUIRED)
find_package(ament_uv_virtualenv REQUIRED)
# Pin the venv to the ROS interpreter, or uv picks the newest python on the
# machine and --system-site-packages then hands it an ABI-mismatched
# numpy/rclpy (an ABI mismatch otherwise).
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_generate_virtualenv(PYTHON ${Python3_EXECUTABLE})

# A student's own ~/ros2_ws/jedy_skrobot.py (or skrobot.py) is imported instead
# of the package when the venv's ipython/python runs in that directory, and the
# error ("cannot import name 'connect' from 'jedy_skrobot'
# (/home/.../jedy_skrobot.py)") reads like a broken install. This .pth runs at
# start-up of every venv interpreter and makes such an import fail with a
# message naming the file and saying to rename it.
#
# Copied into the BUILD venv, not installed next to the installed one: the
# venv's console scripts (ipython, pip, ...) keep the build venv's interpreter
# in their #! line, so `venv/bin/ipython` under install/ runs
# build/jedy_skrobot/venv/bin/python and reads that venv's site-packages. The
# installed venv is a copy of the build one, so it gets the files too.
set(_venv_site_packages
  "${CMAKE_CURRENT_BINARY_DIR}/venv/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
add_custom_target(${PROJECT_NAME}_shadow_guard ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/venv_site/_jedy_skrobot_shadow_guard.pth"
    "${CMAKE_CURRENT_SOURCE_DIR}/venv_site/_jedy_skrobot_shadow_guard.py"
    "${_venv_site_packages}/"
  VERBATIM
)
add_dependencies(${PROJECT_NAME}_shadow_guard ${PROJECT_NAME}_generate_virtualenv)

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
    scripts/enshu/teach_follow.py
    scripts/enshu/sample_servo.py
    scripts/enshu/sample_controllers.py
    scripts/enshu/checkerboard_pose.py
    scripts/enshu/color_clusters.py
    scripts/enshu/checkpoint_go_forward.py
    scripts/enshu/checkpoint_go_forward_imu.py
    scripts/enshu/track_box.py
    scripts/enshu/show_camera.py
    # Asks the course's VLM (ri.ask, through the AI TA server) which way to go.
    scripts/enshu/vlm_navigate.py
    # Nav2 goals from a program (docs/tips/slam.md; Gazebo lidar:=true).
    scripts/enshu/nav2_goal.py
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

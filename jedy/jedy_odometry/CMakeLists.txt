cmake_minimum_required(VERSION 3.16)
project(jedy_odometry)

find_package(ament_cmake REQUIRED)

# A plain importable package, not a virtualenv of its own: whatever interpreter
# consumes it is the one already running the rest of the stack, and a venv here
# would install it somewhere that interpreter cannot see. Same reasoning as
# jedy_safety.
install(
  DIRECTORY jedy_odometry
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/jedy_wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jedy_wheel_odometry
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/jedy_odometry.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_angle test/test_wheel_angle.py)
endif()

ament_package()

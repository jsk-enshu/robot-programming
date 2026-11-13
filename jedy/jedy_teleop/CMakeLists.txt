cmake_minimum_required(VERSION 3.16)
project(jedy_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY jedy_teleop DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/jedy_joy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jedy_joy
)
install(
  PROGRAMS scripts/jedy_joy_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jedy_joy_monitor
)
install(
  PROGRAMS scripts/jedy_joy_buttons.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jedy_joy_buttons
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/jedy_teleop.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_deadband test/test_deadband.py)
endif()

ament_package()

cmake_minimum_required(VERSION 3.16)
project(jedy_safety)

find_package(ament_cmake REQUIRED)

# Installed as a plain importable package, NOT into a virtualenv of its own:
# it is imported by whatever interpreter runs actuo_bus_node --real, which is
# actuo_ros' venv (actuo with the safety extra). A venv here would install it somewhere that interpreter
# cannot see, and the filter would be missing exactly where it is needed.
install(
  DIRECTORY jedy_safety
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/jedy_safety.dsv.in")

ament_package()

"""Remove a sourced ROS 2 environment before micro-ROS is cross-built.

micro_ros_platformio cross-compiles ROS 2 packages with colcon. If the shell
has sourced /opt/ros/<distro>/setup.bash, CMake finds the host's packages
(for example rosidl_typesupport_cpp) and the micro-ROS build fails with
"No 'rosidl_typesupport_cpp' found".
"""
import os

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

ROS_PREFIX = "/opt/ros"
PATH_LIKE = ("PATH", "PYTHONPATH", "LD_LIBRARY_PATH")
DROP = ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION")


def strip_ros():
    """Drop ROS entries from os.environ so child processes do not see them."""
    for key in PATH_LIKE:
        value = os.environ.get(key)
        if value is None:
            continue
        kept = [p for p in value.split(os.pathsep) if not p.startswith(ROS_PREFIX)]
        os.environ[key] = os.pathsep.join(kept)
    for key in DROP:
        os.environ.pop(key, None)


strip_ros()

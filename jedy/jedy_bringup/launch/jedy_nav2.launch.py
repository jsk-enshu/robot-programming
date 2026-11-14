"""Navigate on a saved map with Nav2 (AMCL + MPPI), in Gazebo.

    ros2 launch jedy_bringup jedy_gazebo.launch.py lidar:=true   # first
    ros2 launch jedy_bringup jedy_nav2.launch.py rviz:=true      # bundled map
    ros2 launch jedy_bringup jedy_nav2.launch.py map:=$HOME/room13bc_map.yaml rviz:=true

Then "Nav2 Goal" in RViz, or a program (jedy_skrobot's nav2_goal). The map
defaults to maps/room13bc.yaml, made with jedy_slam.launch.py in Gazebo's
Room 13BC from the spawn pose, so AMCL starts at (0, 0, 0) on it.

nav2_bringup's bringup_launch.py with config/nav2_params.yaml: map_server +
AMCL (localization) and the navigation servers. Like jedy_slam.launch.py it
needs lidar:=true (/scan, odom -> base_link); the real Jedy has neither.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


#: slam_toolbox and Nav2 are not dependencies of jedy_bringup (the tips page
#: is optional and they are large), so say how to get them.
INSTALL = ("sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup "
           "ros-jazzy-nav2-simple-commander")


def required_share(package):
    """Share directory of ``package``, or a RuntimeError saying how to install it.

    Parameters
    ----------
    package : str
        A ROS package name.

    Returns
    -------
    str
    """
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise RuntimeError(
            "{} is not installed. SLAM and Nav2 are optional "
            "(docs/tips/slam.md); install them first:\n  {}".format(
                package, INSTALL)) from None


def generate_launch_description():
    """Build the launch description.

    Returns
    -------
    launch.LaunchDescription
        Nav2 (map_server, AMCL, planner, controller, behaviours, BT
        navigator, ...) and optionally RViz.
    """
    bringup = get_package_share_directory("jedy_bringup")
    nav2_bringup = required_share("nav2_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": use_sim_time,
            "slam": "False",
            "autostart": "true",
            "use_composition": LaunchConfiguration("use_composition"),
        }.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", os.path.join(bringup, "rviz", "jedy_nav.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(bringup, "maps", "room13bc.yaml"),
            description="map YAML (map_saver_cli's output)"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup, "config", "nav2_params.yaml"),
            description="Nav2 parameters"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Gazebo's /clock"),
        # False: one process per Nav2 server. With nav2_bringup's default
        # (True, all servers in one component container) the launch hung
        # loading the composed nodes on a machine without
        # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.
        DeclareLaunchArgument(
            "use_composition", default_value="False",
            choices=["True", "False"],
            description="run the Nav2 servers in one component container"),
        DeclareLaunchArgument(
            "rviz", default_value="false", choices=["true", "false"],
            description="RViz with the map, costmaps, plan and goal tools"),
        nav2, rviz,
    ])

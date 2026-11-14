"""Build a 2-D map with slam_toolbox (online, async), in Gazebo.

    ros2 launch jedy_bringup jedy_gazebo.launch.py lidar:=true   # first
    ros2 launch jedy_bringup jedy_slam.launch.py rviz:=true
    ros2 run nav2_map_server map_saver_cli -f ~/room13bc_map     # when done

Needs /scan and the odom -> base_link TF, which jedy_gazebo.launch.py
publishes with lidar:=true only. The real Jedy has neither (no LiDAR, no
odom frame), so this is for the simulator. docs/tips/slam.md walks through it.

slam_toolbox is a lifecycle node on Jazzy (2.8.x): it does nothing until it
is configured and activated. slam_toolbox's own online_async_launch.py does
both as soon as the node is up (autostart:=true), so this includes it rather
than starting the node itself. Started from the spawn pose, the map's origin
is where the robot was spawned, which is what jedy_nav2.launch.py's AMCL
initial pose assumes.
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
        slam_toolbox, configured and activated, and optionally RViz.
    """
    bringup = get_package_share_directory("jedy_bringup")
    slam_toolbox = required_share("slam_toolbox")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "use_sim_time": use_sim_time,
            "autostart": "true",
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
            "use_sim_time", default_value="true",
            description="Gazebo's /clock; the simulator is the only "
                        "supported source of /scan"),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(bringup, "config",
                                       "slam_toolbox_params_simple.yaml"),
            description="slam_toolbox parameters"),
        DeclareLaunchArgument(
            "rviz", default_value="false", choices=["true", "false"],
            description="RViz with the map, the scan and the robot"),
        slam, rviz,
    ])

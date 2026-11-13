"""Jedy's whole body in Isaac Sim, with the real robot's ROS interface.

Isaac Sim itself runs on its own (it is a separate application, usually a
container):

    USD=.../jedy_fullbody.usdc OUT=/tmp/isaac \\
        $(ros2 pkg prefix jedy_sim)/share/jedy_sim/isaac/run_jedy_isaac.sh

and this launch brings up the robot side against it:

    ros2 launch jedy_bringup jedy_isaac.launch.py

jedy_isaac_servicer takes the bus servicer's place -- same shared memory, same
lock, same handshake -- so what runs above it is exactly the real robot's
stack: jedy_fullbody.launch.py with the actuo backend, the same
jedy_fullbody_controllers.yaml, the same five trajectory controllers and the
mecanum controller on /cmd_vel, all under /jedy_fullbody. jedy_skrobot's
connect() therefore works unchanged; it sees /clock and uses sim time.
jedy_sim_servo serves the real robot's servo switch
(/actuo_bus_servicer/servo/set) with Isaac releasing the joints, as in Gazebo.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from jedy_preflight import preflight_action  # noqa: E402

SHM_NAME = "jedy_fullbody"


def generate_launch_description():
    """Build the launch description.

    Returns
    -------
    launch.LaunchDescription
        The Isaac servicer, then the ros2_control stack on sim time.
    """
    wheels = LaunchConfiguration("wheels")
    servicer = Node(
        package="jedy_sim", executable="jedy_isaac_servicer",
        arguments=["--shm-name", SHM_NAME],
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(wheels),
        output="screen",
    )
    servicer_wheels = Node(
        package="jedy_sim", executable="jedy_isaac_servicer",
        arguments=["--shm-name", SHM_NAME, "--wheels",
                   "--base", LaunchConfiguration("base")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(wheels),
        output="screen",
    )
    servo = Node(
        package="jedy_sim", executable="jedy_sim_servo",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    stack = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("jedy_bringup"), "launch",
                 "jedy_fullbody.launch.py"])),
            launch_arguments={
                "wheels": wheels,
                "rviz": LaunchConfiguration("rviz"),
                "backend": "actuo",
                "use_sim": "true",
                "use_sim_time": "true",
            }.items()),
    ], scoped=True)
    return LaunchDescription([
        # Fail now, with the PIDs, if a robot or an old Gazebo is running.
        preflight_action(),
        DeclareLaunchArgument("wheels", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("base", default_value="wheels",
                              choices=["planar", "wheels"],
                              description="must match the USD Isaac runs "
                                          "(make_jedy_usd.py --base): wheels "
                                          "= the robot stands on its wheels"),
        # A leftover segment file from a previous run would be adopted by the
        # C++ side as this run's; jedy_fullbody_all.launch.py does the same.
        ExecuteProcess(cmd=["rm", "-f", os.path.join(
            "/tmp", "actuo_{}.shm".format(SHM_NAME))], output="log"),
        servicer_wheels, servicer,
        TimerAction(period=3.0, actions=[stack, servo]),
    ])

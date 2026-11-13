"""Speaker + listener (+ RViz with the robot and the speech bubbles).

    ros2 launch jedy_bringup jedy_gazebo.launch.py        # or the real robot
    ros2 launch jedy_speech speech_demo.launch.py         # this
    ros2 run jedy_speech voice_command                    # move by voice

    listen:=false   no microphone (then `ros2 run jedy_speech say "..."`)
    audio:=false    no speaker output, bubbles only
    rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("jedy_speech")
    audio = LaunchConfiguration("audio")
    return LaunchDescription([
        DeclareLaunchArgument("listen", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("audio", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        Node(package="jedy_speech", executable="speaker",
             condition=IfCondition(audio), output="screen"),
        Node(package="jedy_speech", executable="speaker", arguments=["--no-audio"],
             condition=UnlessCondition(audio), output="screen"),
        Node(package="jedy_speech", executable="listener",
             condition=IfCondition(LaunchConfiguration("listen")), output="screen"),
        Node(package="rviz2", executable="rviz2", name="speech_rviz",
             arguments=["-d", os.path.join(share, "rviz", "speech.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])

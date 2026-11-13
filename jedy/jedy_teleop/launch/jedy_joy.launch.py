#!/usr/bin/env python3
"""PS4 pad for Jedy: joy_node plus the teleop node, and nothing else.

    ros2 launch jedy_teleop jedy_joy.launch.py

Runs alongside jedy_four_dof.launch.py rather than inside it: a pad is
something you plug in, not part of the robot, and keeping it separate means it
can be restarted (or unplugged and replugged) without touching the controllers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dev", default_value="/dev/input/js0",
                              description="the joydev node. joy_linux reads "
                                          "this directly, so it is a path and "
                                          "not an SDL index."),
        DeclareLaunchArgument("namespace", default_value="/jedy_four_dof"),
        # Everything below defaults to jedy_four_dof's values, so this launch
        # keeps behaving exactly as it did for that robot. jedy_fullbody passes
        # its own, because it has a different second head joint, a different
        # poses file and a slightly different wheelbase.
        DeclareLaunchArgument(
            "head_joints", default_value="head_joint0,head_module1_joint1",
            description="pan joint, then tilt joint. The D-pad drives exactly "
                        "these two, so the order is the mapping."),
        DeclareLaunchArgument(
            "poses", default_value="",
            description="named-pose YAML. Empty means jedy_bringup's "
                        "jedy_four_dof_poses.yaml. jedy_joy also takes its "
                        "controller/joint grouping from this file, so it must "
                        "be the one matching the robot in `namespace`."),
        DeclareLaunchArgument("wheel_radius", default_value="0.03"),
        DeclareLaunchArgument("centre_projection", default_value="0.11975",
                              description="|x| + |y| of a wheel from the base "
                                          "centre"),
        DeclareLaunchArgument(
            "min_wheel_rad_s", default_value="2.83",
            description="slowest wheel speed the base honours (its deadband); "
                        "default: the ARM-H7 base's"),
        DeclareLaunchArgument(
            "max_wheel_rad_s", default_value="6.85",
            description="fastest wheel speed (saturation); default: the "
                        "ARM-H7 base's"),
        DeclareLaunchArgument(
            "deadzone", default_value="0.10",
            description="the driver's own deadzone. Kept modest because the "
                        "teleop node maps whatever gets through onto the "
                        "base's usable speed range anyway -- a large deadzone "
                        "here would just take travel away from that mapping."),
        # joy_linux, not joy. The `joy` package goes through SDL, and SDL drops
        # joystick events when the application does not have input focus -- so
        # with RViz focused, joy_node opened the pad, read its state ONCE at
        # open, and then published that same state forever. Measured while
        # chasing it: 3207 evdev events arriving from the kernel and not one
        # change in /joy. `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1` fixes it,
        # but an environment variable that must be set or the robot silently
        # stops listening is not a thing to rely on. joy_linux reads joydev
        # directly and has no such notion.
        Node(package="joy_linux", executable="joy_linux_node", output="screen",
             parameters=[{
                 "dev": LaunchConfiguration("dev"),
                 "deadzone": LaunchConfiguration("deadzone"),
                 # Repeat the last state rather than going silent, so releasing
                 # the deadman is seen as a release and not as a dropped pad --
                 # and so jedy_joy's watchdog can tell the two apart.
                 "autorepeat_rate": 20.0,
             }]),
        Node(package="jedy_teleop", executable="jedy_joy", output="screen",
             arguments=["--namespace", LaunchConfiguration("namespace"),
                        "--head-joints", LaunchConfiguration("head_joints"),
                        "--poses", LaunchConfiguration("poses"),
                        "--wheel-radius", LaunchConfiguration("wheel_radius"),
                        "--centre-projection",
                        LaunchConfiguration("centre_projection"),
                        "--min-wheel-rad-s",
                        LaunchConfiguration("min_wheel_rad_s"),
                        "--max-wheel-rad-s",
                        LaunchConfiguration("max_wheel_rad_s")]),
    ])

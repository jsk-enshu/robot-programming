"""Drive the base from the keyboard, in a terminal window of its own.

    ros2 launch jedy_bringup keyboard_teleop.launch.py

The same as running, in a terminal,

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

teleop_twist_keyboard reads keys from its terminal, and a node started by
launch has none, so it is opened in a new gnome-terminal. ``--wait`` keeps
gnome-terminal (and so this launch) running until that window is closed;
without it gnome-terminal returns at once and launch ends while the node runs
on, untracked.

It publishes /cmd_vel (geometry_msgs/TwistStamped), which both the real
robot's and the simulator's mecanum_drive_controller listen to. The 2025
bring-up listened to /mecanum_drive_controller/reference instead; in 2026
only jedy_gazebo.launch.py legacy:=true still serves that name, so it is not
used here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        output='screen',
        prefix='gnome-terminal --wait --',
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ],
        parameters=[
            {'stamped': True},
        ],
    )
    return LaunchDescription([
        keyboard_teleop_node,
    ])

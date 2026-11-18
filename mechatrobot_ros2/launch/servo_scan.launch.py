"""Sweep the servo with the ultrasonic sensor on it and show the points in rviz.

Start serial_bridge.py first, and stop anything else that sends
motor1/command (ros2_control, motor_command_by_hand.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run servo_scan.py and rviz2 with config/servo_scan.rviz."""
    rviz_config = os.path.join(
        get_package_share_directory('mechatrobot_ros2'), 'config', 'servo_scan.rviz')
    direction = DeclareLaunchArgument(
        'direction', default_value='1.0',
        description='1.0 if a positive servo angle turns the sensor counter-clockwise '
                    'seen from above, -1.0 if clockwise')
    step_deg = DeclareLaunchArgument(
        'step_deg', default_value='7.3',
        description='degrees the servo target moves every 50 ms; '
                    'a divisor of 180 measures at the same angles on every sweep')
    scan = Node(
        package='mechatrobot_ros2',
        executable='servo_scan.py',
        name='servo_scan',
        output='screen',
        parameters=[{'direction': LaunchConfiguration('direction'),
                     'step_deg': LaunchConfiguration('step_deg')}],
    )
    # rviz needs the fixed frame to exist in TF; the points are drawn in it.
    frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='servo_scan_frame',
        arguments=['--frame-id', 'world', '--child-frame-id', 'servo_scan'],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )
    return LaunchDescription([direction, step_deg, scan, frame, rviz])

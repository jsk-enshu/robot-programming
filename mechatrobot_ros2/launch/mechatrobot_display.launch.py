import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_mechatrobot_ros2 = get_package_share_directory('mechatrobot_ros2')

    # URDF file path
    urdf_file = os.path.join(pkg_mechatrobot_ros2, 'urdf', 'robot.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # RViz2 configuration
    rviz_config = os.path.join(pkg_mechatrobot_ros2, 'config', 'robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # LED controller node
    led_controller = Node(
        package='mechatrobot_ros2',
        executable='led_controller.py',
        name='led_controller',
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        led_controller,
    ])

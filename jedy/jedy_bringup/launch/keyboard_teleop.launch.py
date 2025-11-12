from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    keyboard_teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        output='screen',
        prefix='gnome-terminal --',
        remappings=[
            ('cmd_vel', '/mecanum_drive_controller/reference')
        ],
        parameters=[
            {'stamped': True}
        ]
    )
    return LaunchDescription([
        keyboard_teleop_node
    ])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    jedy_bringup_share = get_package_share_directory('jedy_bringup')
    config_filepath = os.path.join(jedy_bringup_share, 'config', 'ps3.config.yaml')
    teleop_twist_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_twist_joy'),
                'launch',
                'teleop-launch.py'
            ])
        ]),
        launch_arguments={
            'joy_config': 'ps3',
            'joy_vel': '/mecanum_drive_controller/reference',
            'publish_stamped_twist': 'true',
            'config_filepath': config_filepath
        }.items()
    )

    return LaunchDescription([
        teleop_twist_joy_launch
    ])

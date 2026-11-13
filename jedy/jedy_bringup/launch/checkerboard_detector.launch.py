from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/remote/color/image_rect_raw',
        description='Input image topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/remote/color/camera_info',
        description='Camera info topic'
    )

    checkerboard_size_arg = DeclareLaunchArgument(
        'checkerboard_size',
        default_value='[6, 4]',
        description='Checkerboard size (internal corners): [width, height]'
    )

    square_size_arg = DeclareLaunchArgument(
        'square_size',
        default_value='0.02',
        description='Size of one checkerboard square in meters'
    )

    checkerboard_frame_arg = DeclareLaunchArgument(
        'checkerboard_frame',
        default_value='checkerboard',
        description='Checkerboard frame ID'
    )

    # Node
    checkerboard_detector_node = Node(
        package='jedy_bringup',
        executable='checkerboard_detector.py',
        name='checkerboard_detector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_topic': LaunchConfiguration('input_topic')},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            {'checkerboard_size': LaunchConfiguration('checkerboard_size')},
            {'square_size': LaunchConfiguration('square_size')},
            {'checkerboard_frame': LaunchConfiguration('checkerboard_frame')},
        ]
    )

    # Image view node
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='checkerboard_image_view',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('image', '/checkerboard_detector/debug_image')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        input_topic_arg,
        camera_info_topic_arg,
        checkerboard_size_arg,
        square_size_arg,
        checkerboard_frame_arg,
        checkerboard_detector_node,
        image_view_node
    ])

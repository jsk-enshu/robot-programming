import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_namespace = LaunchConfiguration('camera', default='camera')
    remote_namespace = LaunchConfiguration('remote', default='remote')

    color_in_transport = PythonExpression(
        ["'raw' if '", use_sim_time, "' == 'true' else 'compressed'"]
    )
    depth_in_transport = PythonExpression(
        ["'raw' if '", use_sim_time, "' == 'true' else 'compressedDepth'"]
    )
    color_remap_key = PythonExpression(
        ["'in' if '", use_sim_time, "' == 'true' else 'in/compressed'"]
    )
    depth_remap_key = PythonExpression(
        ["'in' if '", use_sim_time, "' == 'true' else 'in/compressedDepth'"]
    )
    color_topic_suffix = PythonExpression(
        ["'' if '", use_sim_time, "' == 'true' else '/compressed'"]
    )
    depth_topic_suffix = PythonExpression(
        ["'' if '", use_sim_time, "' == 'true' else '/compressedDepth'"]
    )
    color_republish_or_relay = Node(
        package='image_transport',
        executable='republish',
        name='color_republish',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'in_transport': color_in_transport},
            {'out_transport': 'raw'},
        ],
        remappings=[
            (color_remap_key, [camera_namespace, TextSubstitution(text='/color/image_rect_raw'), color_topic_suffix]),
            ('out', [remote_namespace, TextSubstitution(text='/color/image_rect_raw')]),
        ]
    )

    aligned_depth_republish_or_relay = Node(
        package='image_transport',
        executable='republish',
        name='aligned_depth_republish',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'in_transport': depth_in_transport},
            {'out_transport': 'raw'},
        ],
        remappings=[
            (depth_remap_key, [camera_namespace, TextSubstitution(text='/aligned_depth_to_color/image_raw'), depth_topic_suffix]),
            ('out', [remote_namespace, TextSubstitution(text='/aligned_depth_to_color/image_raw')]),
        ]
    )

    color_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='color_info_relay',
        arguments=[
            [camera_namespace, TextSubstitution(text='/color/camera_info')],
            [remote_namespace, TextSubstitution(text='/color/camera_info')]
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    aligned_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='aligned_info_relay',
        arguments=[
            [camera_namespace, TextSubstitution(text='/aligned_depth_to_color/camera_info')],
            [remote_namespace, TextSubstitution(text='/aligned_depth_to_color/camera_info')]
        ],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    point_cloud_xyzrgb = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'queue_size': 100}],
        remappings=[
            ('rgb/image_rect_color', [remote_namespace, TextSubstitution(text='/color/image_rect_raw')]),
            ('rgb/camera_info', [remote_namespace, TextSubstitution(text='/color/camera_info')]),
            ('depth_registered/image_rect', [remote_namespace, TextSubstitution(text='/aligned_depth_to_color/image_raw')]),
            ('depth_registered/camera_info', [remote_namespace, TextSubstitution(text='/aligned_depth_to_color/camera_info')]),
            ('points', [remote_namespace, TextSubstitution(text='/depth/color/points')]),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'camera',
            default_value='camera',
            description='Namespace of input topics'
        ),
        DeclareLaunchArgument(
            'remote',
            default_value='remote',
            description='Namespace of output topics'
        ),
        color_republish_or_relay,
        aligned_depth_republish_or_relay,
        color_info_relay,
        aligned_info_relay,
        point_cloud_xyzrgb,
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_prefix = get_package_prefix('jedy_bringup')
    script_path = os.path.join(pkg_prefix, 'lib', 'jedy_bringup', 'bridge_marker.py')

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/HSI_color_filter/boxes_hsi',
        description='Input BoundingBoxArray topic name'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/HSI_color_filter/boxes_hsi_marker_array',
        description='Output MarkerArray topic name'
    )

    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    ros1_setup = '/opt/ros/one/setup.bash'

    # ROS1ノードを起動するコマンド
    # rosrunスタイルで起動し、パラメータを設定
    # LaunchConfigurationを適切に展開するためにリスト形式で構築
    bridge_marker_node = ExecuteProcess(
        cmd=['bash', '-c', [
             'source ', ros1_setup, ' && ',
             'python3 ', script_path, ' ',
             '_input_topic:=', input_topic, ' ',
             '_output_topic:=', output_topic
        ]],
        shell=False,
        output='screen',
        name='bridge_marker_node'
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        bridge_marker_node,
    ])

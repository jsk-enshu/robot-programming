"""The recognition nodes that ``ri.what_do_you_see()`` / ``ri.gesture()`` call.

    ros2 launch jedy_vision recognition.launch.py
    ros2 launch jedy_vision recognition.launch.py image_topic:=/image_raw   # a PC's web camera

Start it next to any Jedy bring-up (Gazebo, the real robot with
``camera:=true``, Isaac Sim); both nodes read the head camera's colour image.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image = LaunchConfiguration('image_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_rect_raw',
            description='colour image to recognise in (default: the head camera)'),
        DeclareLaunchArgument(
            'detection', default_value='true',
            description='object detection by words (/object_detection/what_do_you_see)'),
        DeclareLaunchArgument(
            'gesture', default_value='true',
            description='hand gestures (/gesture/read_gesture)'),
        Node(package='jedy_vision', executable='object_detection_node',
             output='screen', condition=IfCondition(LaunchConfiguration('detection')),
             parameters=[{'image_topic': image}]),
        Node(package='jedy_vision', executable='gesture_node',
             output='screen', condition=IfCondition(LaunchConfiguration('gesture')),
             parameters=[{'image_topic': image}]),
    ])

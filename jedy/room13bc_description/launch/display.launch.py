"""RViz2: the room (model:=scene, room or desk) with robot_state_publisher and joint sliders (gui:=true).

    ros2 launch room13bc_description display.launch.py [model:=scene|room|desk] [gui:=true|false] [mesh:=obj|glb]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "room13bc_description"
URDF = {"scene": "room13bc_scene.urdf", "room": "room13bc.urdf", "desk": "desk.urdf"}


def nodes(context):
    share = get_package_share_directory(PKG)
    model = LaunchConfiguration("model").perform(context)
    urdf = URDF[model]
    if LaunchConfiguration("mesh").perform(context) == "glb":
        urdf = urdf.replace(".urdf", "_glb.urdf")
    with open(os.path.join(share, "urdf", urdf), encoding="utf-8") as f:
        description = f.read()
    gui = LaunchConfiguration("gui")
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher", condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "room13bc.rviz")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="scene", description="scene (room + 48 desks), room or desk"),
        DeclareLaunchArgument("mesh", default_value="obj",
                              description="the visual meshes: obj (OBJ + MTL + PNG) or glb (glTF, textures embedded)"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="joint sliders (the scene has 48 hinges: one slider each)"),
        OpaqueFunction(function=nodes),
    ])

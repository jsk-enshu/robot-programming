"""Gazebo (gz sim, via ros_gz_sim): an empty world, the room spawned from robot_description, /clock bridged.

    ros2 launch room13bc_description gazebo.launch.py [model:=scene|room|desk] [mesh:=glb|obj]

The model is fixed to the world (its "world" link). Each desk top hangs on its hinge and rests on its upper
limit (flat) under gravity. package:// mesh URIs resolve through GZ_SIM_RESOURCE_PATH (and IGN_GAZEBO_RESOURCE_PATH
for Fortress), which this launch extends with this package's share directory's parent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "room13bc_" + model, "-z", "0.0"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
    ]


def generate_launch_description():
    resources = os.path.dirname(get_package_share_directory(PKG))
    gz = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="scene", description="scene (room + 48 desks), room or desk"),
        DeclareLaunchArgument("mesh", default_value="glb",
                              description="the visual meshes: obj (OBJ + MTL + PNG) or glb (glTF, textures embedded)"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resources),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resources),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz), launch_arguments={"gz_args": "-r empty.sdf"}.items()),
        OpaqueFunction(function=nodes),
    ])

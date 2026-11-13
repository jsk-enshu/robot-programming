"""Bring up Jedy under ros2_control, namespaced /jedy.

Simulation (unchanged):

    ros2 launch jedy_bringup jedy.launch.py
    ros2 run jedy_sim jedy_servicer

Real Kondo backend:

    ros2 launch jedy_bringup jedy.launch.py use_sim:=false
    ros2 run actuo_ros actuo_bus_node --real \
        --config <share>/jedy_bringup/config/jedy_kondo.yaml \
        --shm-name jedy --rate 100

--rate is the servicer's own argument, not a launch argument -- this launch does
not start the servicer, so a launch arg for it would be a knob that does
nothing. 100 Hz is a starting point derived from this stack's own
controller_manager update_rate (jedy_controllers.yaml), so one bus cycle maps to
one control period on a request/response board. Lower reduces bus load and
lengthens the setpoint hold; higher shortens it and costs bus/CPU. It pairs with
bus.move_time_ms in jedy_kondo.yaml, which must move with it. NEITHER NUMBER HAS
BEEN VALIDATED ON HARDWARE.

Publishes /jedy/joint_states (joint_state_broadcaster) and accepts arm goals on
/jedy/jedy_arm_controller/joint_trajectory. The C++ ActuoSystem creates the shared
memory segment "jedy"; run the matching backend servicer alongside.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution)


def generate_launch_description():
    pkg = FindPackageShare("jedy_bringup")
    urdf = PathJoinSubstitution([pkg, "urdf", "jedy_actuo.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "jedy_controllers.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "jedy.rviz"])

    # ParameterValue(..., value_type=str): a URDF is a string, but launch
    # otherwise tries to parse it as YAML and fails.
    robot_description = {
        "robot_description": ParameterValue(Command([
            "xacro ",
            urdf,
            " backend:=", LaunchConfiguration("backend"),
            " use_sim:=", LaunchConfiguration("use_sim"),
        ]), value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="jedy",
        parameters=[robot_description, controllers],
        output="screen",
    )
    # Only the 14 arm joints go through ros2_control, so joint_state_broadcaster
    # publishes only those. The head/wheel/gripper joints would then have no TF
    # and their links would collapse to the origin in RViz. joint_state_publisher
    # fills the gap: it takes the arm joints from the broadcaster (source_list) and
    # emits 0 for every other movable joint, publishing the merged set that
    # robot_state_publisher consumes for full TF.
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="jedy",
        parameters=[{"source_list": ["/jedy/joint_states"], "rate": 30}],
        remappings=[("joint_states", "/jedy/joint_states_full")],
        output="screen",
    )
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="jedy",
        parameters=[robot_description],
        remappings=[("joint_states", "/jedy/joint_states_full")],
        output="screen",
    )
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster",
                          "-c", "/jedy/controller_manager"])

    mode = LaunchConfiguration("mode")

    def spawner(name, mode_value):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "-c", "/jedy/controller_manager"],
                    condition=IfCondition(EqualsSubstitution(mode, mode_value)))

    # one controller per mode; the servicer must run with the matching --mode
    arm = spawner("jedy_arm_controller", "position")
    vel = spawner("jedy_velocity_controller", "velocity")
    eff = spawner("jedy_effort_controller", "effort")

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="launch RViz2 with the Jedy config"),
        DeclareLaunchArgument("mode", default_value="position",
                              description="position | velocity | effort"),
        DeclareLaunchArgument("backend", default_value="actuo",
                              choices=["actuo", "mock"],
                              description="actuo = ActuoSystem backend; "
                                          "mock = GenericSystem backend"),
        DeclareLaunchArgument("use_sim", default_value="true",
                              choices=["true", "false"],
                              description="passed to ActuoSystem. true = sim "
                                          "bus servicer, false = real bus "
                                          "servicer"),
        control_node, robot_state_pub, joint_state_pub, jsb, arm, vel, eff, rviz,
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_mechatrobot_ros2 = get_package_share_directory('mechatrobot_ros2')

    # Paths
    urdf_file = os.path.join(pkg_mechatrobot_ros2, 'urdf', 'robot.urdf')
    controllers_file = os.path.join(pkg_mechatrobot_ros2, 'config', 'controllers.yaml')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            controllers_file
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'mechatrobot_hardware_interface:=info']
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Position trajectory controller spawner
    position_trajectory_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_trajectory_controller'],
        output='screen'
    )

    # Delay position controller spawner after joint state broadcaster
    delay_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_trajectory_controller_spawner],
        )
    )

    # Static transform publisher (map to base_link)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base',
        arguments=['-0.2', '0', '0', '3.1415', '0', '0', '/map', '/base_link']
    )

    # rqt_joint_trajectory_controller
    rqt_joint_trajectory_controller = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        name='rqt_joint_trajectory_controller',
        output='screen'
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,
        static_tf_node,
        joint_state_broadcaster_spawner,
        delay_position_controller_spawner,
        rqt_joint_trajectory_controller,
    ])

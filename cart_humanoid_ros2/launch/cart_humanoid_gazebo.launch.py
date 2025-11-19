import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Set Gazebo resource path
    pkg_cart_humanoid_ros2 = get_package_share_directory('cart_humanoid_ros2')
    model_path = os.path.join(pkg_cart_humanoid_ros2, 'worlds', 'model')

    resource_paths = [model_path, os.path.dirname(pkg_cart_humanoid_ros2)]

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + ':'.join(resource_paths)
    else:
        gz_resource_path = ':'.join(resource_paths)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    fix_base_link = LaunchConfiguration('fix_base_link', default='true')

    model_file = os.path.join(pkg_cart_humanoid_ros2, 'urdf', 'robot_gz.xacro')
    controllers_config = os.path.join(pkg_cart_humanoid_ros2, 'config', 'cart_humanoid_controllers.ros2.yaml')

    # Gazebo with humanoid_workspace world
    world_file = os.path.join(pkg_cart_humanoid_ros2, 'worlds', 'humanoid_workspace.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Robot description
    robot_description = {'robot_description': Command([
        'xacro ', model_file,
        ' fix_base_link:=', fix_base_link,
        ' controllers_config:=', controllers_config,
    ])}

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-entity', 'cart_humanoid', '-z', '0.01'],
        output='screen'
    )

    # Wait for Gazebo to be ready before spawning controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    fullbody_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fullbody_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    l_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['l_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    r_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['r_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawners to ensure gz_ros2_control is ready
    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_fullbody_controller = TimerAction(
        period=7.0,
        actions=[fullbody_controller_spawner]
    )

    delayed_l_gripper_controller = TimerAction(
        period=9.0,
        actions=[l_gripper_controller_spawner]
    )

    delayed_r_gripper_controller = TimerAction(
        period=11.0,
        actions=[r_gripper_controller_spawner]
    )

    # Clock bridge - essential for use_sim_time nodes
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Camera bridge - bridges Gazebo camera topics to ROS2 (RGB and depth)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
        remappings=[
            ('/camera/image', '/camera/color/image_rect_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
            ('/camera/depth_image', '/camera/depth/image_rect_raw'),
            ('/camera/depth_camera_info', '/camera/depth/camera_info'),
        ]
    )

    # Relay depth topics to aligned_depth_to_color (same data in simulation)
    aligned_depth_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/camera/depth/image_rect_raw', '/camera/aligned_depth_to_color/image_raw'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    aligned_depth_info_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/camera/depth/camera_info', '/camera/aligned_depth_to_color/camera_info'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Point cloud generation from RGB + Depth (like ROS1 openni2.launch)
    point_cloud_xyzrgb = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image_rect_color', '/camera/color/image_rect_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth_registered/image_rect', '/camera/depth/image_rect_raw'),
            ('depth_registered/camera_info', '/camera/depth/camera_info'),
            ('points', '/camera/depth/color/points'),
        ]
    )

    # Depthimage to laserscan converter
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_height': 10,
            'range_min': 0.05,
            'output_frame': 'camera_depth_frame'
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/rgb/camera_info'),
            ('scan', '/scan'),
        ],
        output='screen'
    )

    # RViz2 node with config file - delayed to ensure topics exist
    rviz_config = os.path.join(pkg_cart_humanoid_ros2, 'config', 'cart_humanoid.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Delay RViz2 to ensure topics are ready
    delayed_rviz = TimerAction(
        period=8.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='GZ_FILE_PATH', value=model_path),
        SetEnvironmentVariable(name='DISPLAY', value=':1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fix_base_link', default_value='true'),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        camera_bridge,
        aligned_depth_relay,
        aligned_depth_info_relay,
        point_cloud_xyzrgb,
        depthimage_to_laserscan,
        delayed_joint_state_broadcaster,
        delayed_fullbody_controller,
        delayed_l_gripper_controller,
        delayed_r_gripper_controller,
        delayed_rviz,
    ])

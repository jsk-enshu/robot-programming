import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Set Gazebo resource path
    pkg_jedy_bringup = get_package_share_directory('jedy_bringup')
    pkg_jedy_description = get_package_share_directory('jedy_description')
    model_path = os.path.join(pkg_jedy_bringup, 'worlds', 'model')

    # ':' で区切られた複数のパスを設定
    resource_paths = [model_path, os.path.dirname(pkg_jedy_description)]

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + ':'.join(resource_paths)
    else:
        gz_resource_path = ':'.join(resource_paths)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    model_file = os.path.join(pkg_jedy_description, 'urdf', 'jedy_gz.xacro')
    controllers_config = os.path.join(pkg_jedy_bringup, 'config', 'jedy_controllers.ros2.yaml')

    # Gazebo with sensors enabled
    world_file = os.path.join(pkg_jedy_bringup, 'worlds', 'empty.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Robot description with initial joint positions
    # You can override these values by passing arguments to the launch file
    robot_description = {'robot_description': Command([
        'xacro ', model_file,
        ' head_j0_init:=0.0',              # 0 deg
        ' head_j1_init:=0.0',              # 0 deg
        ' rarm_j0_init:=1.5708',           # 90 deg
        ' rarm_j1_init:=0.0',              # 0 deg
        ' rarm_j2_init:=-0.5236',          # -30 deg
        ' rarm_j3_init:=-1.7453',          # -100 deg
        ' rarm_j4_init:=0.0',              # 0 deg
        ' rarm_j5_init:=-0.1745',          # -10 deg
        ' rarm_j6_init:=0.0',              # 0 deg
        ' rarm_gripper_init:=0.0',
        ' larm_j0_init:=-1.5708',          # -90 deg
        ' larm_j1_init:=0.0',              # 0 deg
        ' larm_j2_init:=0.5236',           # 30 deg
        ' larm_j3_init:=-1.7453',          # -100 deg
        ' larm_j4_init:=0.0',              # 0 deg
        ' larm_j5_init:=-0.1745',          # -10 deg
        ' larm_j6_init:=0.0',              # 0 deg
        ' larm_gripper_init:=0.0',
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
        arguments=['-topic', '/robot_description', '-entity', 'jedy', '-z', '0.2'],
        output='screen'
    )

    # Wait for Gazebo to be ready before spawning controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    head_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rarm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rarm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    larm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['larm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawners to ensure gz_ros2_control is ready
    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_diff_drive_controller = TimerAction(
        period=7.0,
        actions=[diff_drive_controller_spawner]
    )

    delayed_head_controller = TimerAction(
        period=9.0,
        actions=[head_controller_spawner]
    )

    delayed_rarm_controller = TimerAction(
        period=11.0,
        actions=[rarm_controller_spawner]
    )

    delayed_larm_controller = TimerAction(
        period=13.0,
        actions=[larm_controller_spawner]
    )

    # Clock bridge - essential for use_sim_time nodes
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Camera bridge - bridges Gazebo camera topics to ROS2 (RGB and depth only)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
        remappings=[
            ('/camera/image', '/camera/rgb/image_raw'),
            ('/camera/camera_info', '/camera/rgb/camera_info'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ]
    )

    # Point cloud generation from RGB + Depth (like ROS1 openni2.launch)
    point_cloud_xyzrgb = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image_rect_color', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth_registered/image_rect', '/camera/depth/image_raw'),
            ('points', '/camera/depth/points'),
        ]
    )

    # Static transform to connect Gazebo's camera frame to optical frame
    # RGB and depth images should be published in camera_*_optical_frame
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_depth_optical_frame', 'jedy/head_link1/camera'],
        output='screen'
    )

    # base_footprint frame - required by Nav2 collision monitor
    # This is the projection of base_link onto the ground plane
    base_footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # cmd_vel_relay removed - DiffDrive plugin accepts regular Twist on /cmd_vel
    # cmd_vel_relay = Node(
    #     package='jedy_bringup',
    #     executable='twist_stamper.py',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'frame_id': 'base_link'}
    #     ],
    #     remappings=[
    #         ('cmd_vel_in', '/cmd_vel'),
    #         ('cmd_vel_out', '/mecanum_drive_controller/reference'),
    #     ]
    # )

    # LiDAR bridge - bridges Gazebo LiDAR to ROS2 with BEST_EFFORT QoS for sensor data
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '--ros-args',
            '-p', 'qos_overrides./scan.subscription.reliability:=best_effort',
            '-p', 'qos_overrides./scan.publisher.reliability:=best_effort'
        ],
        output='screen'
    )

    # Note: diff_drive_controller publishes odom and TF directly, so no bridge needed

    # RViz2 node with config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_jedy_bringup, 'config', 'jedy.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='DISPLAY', value=':1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        camera_bridge,
        camera_tf_publisher,
        base_footprint_publisher,  # Add base_footprint frame for Nav2
        point_cloud_xyzrgb,
        lidar_bridge,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
        delayed_head_controller,
        delayed_rarm_controller,
        delayed_larm_controller,
        rviz_node,
    ])

#!/usr/bin/env python3
"""
Publish robot_description from xacro file to ROS 2 topic.
This allows rqt_joint_trajectory_controller to get joint limits.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import subprocess
import sys
import os

class RobotDescriptionPublisher(Node):
    def __init__(self, xacro_file):
        super().__init__('robot_description_publisher')

        # Convert xacro to URDF
        self.get_logger().info(f'Processing xacro file: {xacro_file}')
        try:
            result = subprocess.run(
                ['xacro', xacro_file],
                capture_output=True,
                text=True,
                check=True
            )
            robot_desc = result.stdout
            self.get_logger().info(f'Successfully generated URDF ({len(robot_desc)} characters)')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to process xacro: {e.stderr}')
            sys.exit(1)

        # Create publisher with TRANSIENT_LOCAL durability
        # This ensures late subscribers receive the message
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(String, '/robot_description', qos)

        # Publish the robot description
        msg = String()
        msg.data = robot_desc
        self.publisher.publish(msg)
        self.get_logger().info('Published robot_description to /robot_description topic')
        self.get_logger().info('Topic will remain available for late subscribers (TRANSIENT_LOCAL)')

def main():
    # Default xacro file path from jedy_description package
    try:
        jedy_description_path = get_package_share_directory('jedy_description')
        default_xacro = os.path.join(jedy_description_path, 'urdf', 'jedy_gz.xacro')
    except Exception as e:
        print(f'Error: Could not find jedy_description package: {e}', file=sys.stderr)
        sys.exit(1)

    xacro_file = sys.argv[1] if len(sys.argv) > 1 else default_xacro

    rclpy.init()
    node = RobotDescriptionPublisher(xacro_file)

    # Keep node alive to maintain the publisher
    print('\nrobot_description is now published and available.')
    print('Press Ctrl+C to stop...\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

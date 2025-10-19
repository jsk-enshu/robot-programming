#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameChanger(Node):
    def __init__(self):
        super().__init__('scan_frame_changer')

        # Parameters
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('target_frame', 'base_laser')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        # QoS profile for sensor data (BEST_EFFORT)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            sensor_qos)

        self.publisher = self.create_publisher(LaserScan, output_topic, sensor_qos)

        self.get_logger().info(f'Scan frame changer started: {input_topic} -> {output_topic}, frame: {self.target_frame}')

    def scan_callback(self, msg):
        # Change frame_id
        msg.header.frame_id = self.target_frame
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameChanger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ButtonSubscriber(Node):
    def __init__(self):
        super().__init__('checkpoint1_button_python')
        self.subscription = self.create_subscription(
            Int32,
            '/atom_s3_button_state',
            self.button_callback,
            10)
        self.get_logger().info('ready to subscribe !!!')

    def button_callback(self, msg):
        button_count = msg.data
        self.get_logger().info(f'subscribe msg [button-count: {button_count}]')


def main(args=None):
    rclpy.init(args=args)
    button_subscriber = ButtonSubscriber()
    rclpy.spin(button_subscriber)
    button_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

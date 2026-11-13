#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class DisplayPublisher(Node):
    def __init__(self):
        super().__init__('checkpoint1_string_python')
        self.publisher = self.create_publisher(String, '/atom_s3_additional_info', 10)
        time.sleep(1)
        self.publish_messages()

    def publish_messages(self):
        msg = String()
        for i in range(1, 4):
            msg.data = str(i)
            self.get_logger().info(f'publish msg [data: {msg.data}]')
            self.publisher.publish(msg)
            time.sleep(1)

        msg.data = 'Finish!'
        self.get_logger().info(f'publish [data: {msg.data}]')
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    display_publisher = DisplayPublisher()
    display_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

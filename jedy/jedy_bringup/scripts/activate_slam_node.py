#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time


class SlamActivator(Node):
    def __init__(self):
        super().__init__('slam_activator')
        self.get_logger().info('Waiting for SLAM Toolbox to be ready...')
        time.sleep(2)

        # Create service client for changing state
        self.client = self.create_client(ChangeState, '/slam_toolbox/change_state')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for slam_toolbox service...')

        # Configure
        self.get_logger().info('Configuring SLAM Toolbox...')
        self.call_change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(1)

        # Activate
        self.get_logger().info('Activating SLAM Toolbox...')
        self.call_change_state(Transition.TRANSITION_ACTIVATE)

        self.get_logger().info('SLAM Toolbox is now active!')

    def call_change_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Transition successful: {future.result().success}')
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)
    activator = SlamActivator()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

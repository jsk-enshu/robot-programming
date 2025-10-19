#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from opencv_apps.msg import RotatedRectStamped
from geometry_msgs.msg import TwistStamped

class TrackBoxToCmdVel(Node):
    def __init__(self):
        super().__init__('track_box_to_cmd_vel_0')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.subscription = self.create_subscription(
            RotatedRectStamped, '/camshift/track_box', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribing: {msg.rect.center}')
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        # 認識結果の領域の中心のx座標が320より小さければ（画像の左），右回転する
        if msg.rect.center.x < 320:
            cmd_vel.twist.angular.z = 0.1
        else:
            cmd_vel.twist.angular.z = -0.1
        self.get_logger().info(f'Publishing angular.z: {cmd_vel.twist.angular.z}')
        # 台車速度指令値をパブリッシュ
        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TrackBoxToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

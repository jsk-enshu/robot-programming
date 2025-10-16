#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from opencv_apps.msg import RotatedRectStamped
from geometry_msgs.msg import TwistStamped

class TrackBoxToCmdVel(Node):
    def __init__(self):
        super().__init__('track_box_to_cmd_vel_1')
        self.rect = RotatedRectStamped()  # メンバ変数として定義
        self.subscription = self.create_subscription(
            RotatedRectStamped,
            '/camshift/track_box',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hzのタイマー

    def listener_callback(self, msg):
        area = msg.rect.size.width * msg.rect.size.height
        self.get_logger().info(f'Area: {area}, Center: ({msg.rect.center.x}, {msg.rect.center.y})')
        if area > 100 * 100:
            self.rect = msg  # メンバ変数に結果を保存

    def timer_callback(self):
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        # ROS 2では get_clock() を使って現在の時間を取得
        now = self.get_clock().now()
        rect_arrived = now - rclpy.time.Time.from_msg(self.rect.header.stamp)

        if rect_arrived.nanoseconds / 1e9 < 1.0:  # 最大1秒前の認識結果を利用
            if self.rect.rect.center.x < 320:
                cmd_vel.twist.angular.z = 0.1
            else:
                cmd_vel.twist.angular.z = -0.1

        # 古いデータの場合、あるいは最初からデータがない場合は、ゼロのcmd_velがpublishされる
        # これにより、対象物がカメラから消えた場合はロボットは止まる
        self.get_logger().info(f'Publishing angular.z: {cmd_vel.twist.angular.z}')
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

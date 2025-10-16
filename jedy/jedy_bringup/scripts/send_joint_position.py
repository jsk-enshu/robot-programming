#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class HeadController(Node):
    def __init__(self):
        super().__init__('head_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        # Publisherが接続を確立するのを少し待つ
        time.sleep(1)

    def send_joint_position(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory.joint_names = ['head_joint0', 'head_joint1']

        # ループ内で新しいPointインスタンスを作成する
        for i in range(5):
            point = JointTrajectoryPoint()
            # head_joint0 (pan) と head_joint1 (tilt) を動かす
            point.positions = [math.pi/6*(i%3-1), math.pi/8*(i%3-1)]
            point.time_from_start = Duration(sec=1+i, nanosec=0)
            joint_trajectory.points.append(point)

        self.get_logger().info('Publishing joint trajectory...')
        self.publisher_.publish(joint_trajectory)
        # 動作終了を待つ
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
    node.send_joint_position()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
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
        # subscriberが現れるまで待つ最大時間[s]
        self.declare_parameter('wait_timeout', 10.0)
        # 同じ軌道を再送し続ける時間[s]と再送周期[s]
        self.declare_parameter('resend_duration', 1.0)
        self.declare_parameter('resend_period', 0.1)

    def wait_for_subscriber(self):
        timeout = self.get_parameter('wait_timeout').value
        start = time.monotonic()
        while rclpy.ok() and self.publisher_.get_subscription_count() == 0:
            if time.monotonic() - start > timeout:
                self.get_logger().error(
                    f'No subscriber on {self.publisher_.topic_name} after {timeout} s. '
                    'Is head_controller (or ros1_bridge) running?')
                return False
            self.get_logger().info('Waiting subscriber...', throttle_duration_sec=1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

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

        # ros1_bridge経由の場合，ROS 2側のsubscriberが見えた時点ではROS 1側の接続が
        # まだ確立しておらず最初のメッセージが落ちることがある．
        # header.stampが同じ軌道はコントローラ側で同一の軌道として扱われるので，
        # 同じメッセージをしばらく再送しても動作は変わらない．
        resend_duration = self.get_parameter('resend_duration').value
        resend_period = self.get_parameter('resend_period').value
        self.get_logger().info('Publishing joint trajectory...')
        start = time.monotonic()
        while rclpy.ok():
            self.publisher_.publish(joint_trajectory)
            if time.monotonic() - start >= resend_duration:
                break
            rclpy.spin_once(self, timeout_sec=resend_period)
        # 動作終了を待つ
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = HeadController()
    # Publisherが接続を確立するのを待つ
    if not node.wait_for_subscriber():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.send_joint_position()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

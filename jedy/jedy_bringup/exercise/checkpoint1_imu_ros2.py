#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math


class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_yaw_pitch_roll_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.get_logger().info('Ready to subscribe to IMU data!')

    def calc_yaw_pitch_roll(self, linear_acceleration):
        x = linear_acceleration.x
        y = linear_acceleration.y
        z = linear_acceleration.z

        # 各角度の計算 (radian単位)
        roll = math.atan2(-x, math.sqrt(y * y + z * z))
        pitch = math.atan2(y, z)
        yaw = 0  # Yawは加速度データだけでは計算できないため、仮に0とする

        return roll, pitch, yaw

    def imu_callback(self, msg):
        roll, pitch, yaw = self.calc_yaw_pitch_roll(msg.linear_acceleration)
        self.get_logger().info(f'roll: {roll:.3f} pitch: {pitch:.3f} yaw: {yaw:.3f}')


def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

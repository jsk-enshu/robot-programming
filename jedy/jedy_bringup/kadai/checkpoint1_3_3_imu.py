#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import math

def calc_yaw_pitch_roll(linear_acceleration):
    x = linear_acceleration.x
    y = linear_acceleration.y
    z = linear_acceleration.z

    # 各角度の計算 (radian単位)
    roll = math.atan2(-x, math.sqrt(y * y + z * z))
    pitch = math.atan2(y, z)
    yaw = 0  # Yawは加速度データだけでは計算できないため、仮に0とする

    return roll, pitch, yaw

def imu_callback(msg):
    roll, pitch, yaw = calc_yaw_pitch_roll(msg.linear_acceleration)
    rospy.loginfo("roll: %.3f pitch: %.3f yaw: %.3f", roll, pitch, yaw)

def main():
    rospy.init_node('imu_yaw_pitch_roll_node')
    rospy.Subscriber("/imu_publisher/imu", Imu, imu_callback)
    rospy.loginfo("Ready to subscribe to IMU data!")
    rospy.spin()

if __name__ == '__main__':
    main()

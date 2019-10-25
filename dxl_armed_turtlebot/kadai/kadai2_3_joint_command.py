#!/usr/bin/env python

import math
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def send_joint_position():
    rospy.init_node("kadai2_3_joint_command", anonymous=True)

    pub = rospy.Publisher("/fullbody_controller/command", JointTrajectory, queue_size=1)
    rospy.sleep(1)

    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory.joint_names = ["arm_joint1", "arm_joint2", "arm_joint3",
                                    "arm_joint4", "arm_joint5", "arm_joint6"]
    for i in range(5):
        point = JointTrajectoryPoint()
        point.positions = [math.pi/2, 0, math.pi/4*(i%2), 0, math.pi/2, math.pi/2]
        point.time_from_start = rospy.Duration(1.0+i)
        joint_trajectory.points.append(point)

    pub.publish(joint_trajectory)

    rospy.sleep(5)

if __name__ == '__main__':
    try:
        send_joint_position()
    except rospy.ROSInterruptException: pass

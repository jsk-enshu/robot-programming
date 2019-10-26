#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from opencv_apps.msg import RotatedRectStamped

pub = None

def cb(msg):
    global pub
    cmd_vel = Twist()
    size = msg.rect.size.width * msg.rect.size.height
    rospy.loginfo("rect size: %d\n"%size)
    if (size < 30000):
        cmd_vel.angular.z = 0.5
    cmd_vel.angular.z
    pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        rospy.init_node("kadai2_3_cmd_vel", anonymous=True)
        rospy.Subscriber("/camshift/track_box", RotatedRectStamped, cb)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException: pass

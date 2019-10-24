#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import Led

if __name__ == '__main__':
    rospy.init_node("kadai1_3_2_led_python", anonymous=True)

    pub = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1)
    rospy.sleep(1)

    msg = Led()
    for i in range(3):
        msg.value = i+1
        rospy.loginfo("publish msg [value: %d]"%(msg.value))
        pub.publish(msg)
        rospy.sleep(1)
    msg.value = 0
    rospy.loginfo("publish msg [value: %d]"%(msg.value))
    pub.publish(msg)

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Led

pub = rospy.Publisher('/mobile_base/commands/led1', Led)
def callback(data):
    print(rospy.get_caller_id()+"joy\n %s", data.buttons)
    if data.buttons[5] == 1:
        pub.publish(data.buttons[4]+1)
    return


def joy():
    rospy.init_node('myJoy', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)

    print("spin")
    rospy.spin()

if __name__ == "__main__":
    print("main")
    joy()

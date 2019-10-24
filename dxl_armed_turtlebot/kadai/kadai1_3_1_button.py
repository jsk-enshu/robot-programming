#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import ButtonEvent

def button_cb(msg):
    button = msg.button
    state = msg.state
    rospy.loginfo("subscribe msg [button: %d  state: %d]"%(button, state))
    
if __name__ == '__main__':
    rospy.init_node("kadai1_3_1_button_python", anonymous=True)

    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, button_cb)

    rospy.spin()

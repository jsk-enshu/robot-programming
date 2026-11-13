#!/usr/bin/env python


import rospy
from std_msgs.msg import Int32


def button_callback(msg):
    button_count = msg.data
    rospy.loginfo("subscribe msg [button-count: %d]", button_count)


def main():
    rospy.init_node("checkpoint1_3_1_button_python")
    rospy.Subscriber("/atom_s3_button_state", Int32, button_callback)
    rospy.loginfo("ready to subscribe !!!")
    rospy.spin()


if __name__ == "__main__":
    main()

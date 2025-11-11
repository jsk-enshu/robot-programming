#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time


def main():
    rospy.init_node("checkpoint1_3_2_string_python")
    pub = rospy.Publisher("/atom_s3_additional_info", String, queue_size=10)
    rospy.sleep(1)

    msg = String()
    for i in range(1, 4):
        msg.data = str(i)
        rospy.loginfo("publish msg [data: %s]", msg.data)
        pub.publish(msg)
        time.sleep(1)

    msg.data = "Finish!"
    rospy.loginfo("publish [data: %s]", msg.data)
    pub.publish(msg)


if __name__ == "__main__":
    main()

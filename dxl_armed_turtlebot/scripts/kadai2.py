#!/usr/bin/env python
import rospy, actionlib
from move_base_msgs.msg import *
from opencv_apps.msg import *
from image_view2.msg import *


def callback(msg):
    if msg.rect.center.x != 0.0:
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.pose.position.x = 1;
        goal.target_pose.pose.position.y = 1;
        goal.target_pose.pose.orientation.w = 1
        print goal
        client.send_goal(goal)
        print client.wait_for_result()

if __name__ == '__main__':
    try:
        rospy.init_node('track_box_listener', anonymous=True)
        rospy.Subscriber("/camshift/track_box", RotatedRectStamped, callback)
        rospy.spin()
    except rospy.ROSInterruptException: pass

#!/usr/bin/env python

import rospy, actionlib
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
from move_base_msgs.msg import *

def cb(msg):
    if not (msg.rect.center.x==0 and msg.rect.center.y==0):
        client =actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp =rospy.Time.now()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.pose.position.x = 3.3
        goal.target_pose.pose.position.y = 3.3
        goal.target_pose.pose.orientation.w = 1
        client.send_goal(goal)
        print client.wait_for_result()
        
if __name__ == '__main__':
    rospy.init_node('client', anonymous=True)
    rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
    rospy.spin()
        

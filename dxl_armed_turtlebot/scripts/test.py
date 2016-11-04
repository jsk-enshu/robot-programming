#!/usr/bin/env python

import rospy
#import Python client API for ROS
from opencv_apps.msg import RotatedRectStamped
#import RotatedRectStamped which is subscribed by ?
#Motion Analysis Nodes of opencv_apps.msg ?
#using camshift algorithms
from image_view2.msg import ImageMarker2
#message to draw on image_view2 canvas
from geometry_msgs.msg import Point
#Message type which contains the position of a point in free space

def cb(msg):
    print msg.rect #print msg.rect subscribed by RotatedRectStamped?
    #angle,center(x,y),size(width,height)
    marker = ImageMarker2() #set marker
    marker.type = 0 #draw circle?
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #at msg.rect position
    pub.publish(marker)#publish marker/draw marker on canvas?


rospy.init_node('client')
#initialize node

rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#set subscriber callback.
#rospy.Subscriber('topic_name',std_msgs.msg.String?,callback)
#the topic /camshift/track_box is opencv_apps.
#and String RotatedRectStamped has
#header:
#   seq
#   stamp:
#       secs
#       nsecs
#   frameid
#rect:
#    angle
#    center
#    size
#give msg to callback

pub = rospy.Publisher('/image_marker', ImageMarker2)
#rospy.Publisher initialization
#pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)

rospy.spin() #run

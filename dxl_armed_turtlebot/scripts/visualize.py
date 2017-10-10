#!/usr/bin/env python

import rospy # needed when using ROS Node
from opencv_apps.msg import RotatedRectStamped # import opencv_apps/msg/RotatedRectStamped.msg, and make rect (of RotatedRect type) and header (of Header type) available
from image_view2.msg import ImageMarker2 # import image_view2/ImageMarker2.msg
from geometry_msgs.msg import Point # import geometry_msgs/Point.msg

def cb(msg):
    print msg.rect #print out msg's rect
    marker = ImageMarker2() #construct and init marker of ImageMarker2()
    marker.type = 0 #define marker's type as circle
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #define marker's position 
    pub.publish(marker) #publish the marker

rospy.init_node('client') #initialize node with the name 'client'
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #subscribe messages of RotatedRectStamped type from '/camshift/track_box' topic. Pass the messages subscribed from the topic as an argument to a callback function 'cb'.
pub = rospy.Publisher('/image_marker', ImageMarker2) #publish a message of ImageMarker2 type to a topic '/image_marker'.
rospy.spin() #keeps python from exiting until this node is stopped

#!/usr/bin/env python

# import required libraries.
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect # print out the rect property of msg object.
    marker = ImageMarker2() # make ImageMarker2 class object.
    marker.type = 0 # set type to 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) # set marker's position.
    pub.publish(marker) # publish object to ros server.

rospy.init_node('client') # initialize ros node.
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) # subscribe '/camshif/track_box' topic (type RotatedRectStamped) and set cb as callback function.
pub = rospy.Publisher('/image_makrer', ImageMarker2) # declare pub which will be used in cb function.
rospy.spin() # start infinite loop to keep this node alive.

#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped #  for message of RotaredRect
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #  Cartesian coordinate
    pub.publish(marker)

rospy.init_node('client') #  initialize node
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #  receive rectangle to mark
pub = rospy.Publisher('/image_marker', ImageMarker2) #  send
rospy.spin() #  main loop

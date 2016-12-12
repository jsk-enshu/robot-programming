#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point #import libraries

def cb(msg):
    print msg.rect
    marker = ImageMarker2() #Create a marker
    marker.type = 0 #initialize marker
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #Bound marker to the center of the selected area
    pub.publish(marker)

rospy.init_node('client')
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
pub = rospy.Publisher('/image_marker',ImageMarker2)
rospy.spin()

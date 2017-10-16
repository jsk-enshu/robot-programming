#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    # print message
    print msg.rect
    # substitute marker
    marker = ImageMarker2()
    # set marker type
    marker.type = 0
    # set marker position
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # publish marker
    pub.publish(marker)
# init client node
rospy.init_node('client')
# send data to track_box
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# send data to image_marker
pub = rospy.Publisher('/image_marker', ImageMarker2)
rospy.spin()

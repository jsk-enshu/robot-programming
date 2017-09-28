#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    # print info of moving rectangle
    print msg.rect
    marker = ImageMarker2()
    # circle marker
    marker.type = 0
    # put marker pos
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # publish the marker
    pub.publish(marker)
rospy.init_node('client')
# send the data to visualizer of camshift
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# send the data to visualizer of image_marker
pub = rospy.Publisher('/image_marker', ImageMarker2)
rospy.spin()

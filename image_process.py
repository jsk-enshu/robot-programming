#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    # msgの内容を標準出力する
    print msg.rect
    
    # markerを作成する
    # 型はImageMarker2
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    pub.publish(marker)

    
rospy.init_node('client')

#track_boxの内容を受け取る
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

#image_markerを送出
pub = rospy.Publisher('/image_marker', ImageMarker2)
rospy.spin()
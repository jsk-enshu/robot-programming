#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect
    marker = ImageMarker2() #markerオブジェクトの作成
    marker.type = 0 #markerの形を指定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #markerの位置を指定
    pub.publish(marker) #markerを送信

rospy.init_node('client') #nodeの名前を指定
rospy.Subscriber('/camshift/track_box' RotatedRectStamped, cb) #/camshift/track_boxからメッセージを受信
pub = rospy.Publisher('/image_marker', ImageMarker2) #/image_markerにメッセージを送信
rospy.spin()
#!/usr/bin/env python
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect # msgの短形情報を表示 
    marker = ImageMarker2() # markerのインスタンスを生成
    marker.type = 0 #markerを円形にする
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #markerの中心を短形の中心にする
    pub.publish(marker) #markerを送信

rospy.init_node('client') #通信開始
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)  #(topic, 型, callback)
pub = rospy.Publisher('/image_marker', ImageMarker2) #(topic, 型, size)
rospy.spin()


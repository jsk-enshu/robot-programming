#!/usr/bin/env python

#必要なpackageをimport
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect #矩形の情報を表示
    marker = ImageMarker2() #markerの定義
    marker.type = 0 #円形のmarker
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #矩形の中心にmarkerの中心を設定
    pub.publish(marker) #markerを送信
    
rospy.init_node('client') #ノードを宣言
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #(topic名, 型, callback関数)
pub = rospy.Publisher('/image_marker', ImageMarker2) #(topic名, 型, size)
rospy.spin() #nodeが終了するまで

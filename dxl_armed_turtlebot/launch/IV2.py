#!/usr/bin/env python

import rospy
#pythonでROSを書くためにimport
from opencv_apps.mag import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
#messageファイルからモジュールをimport

def cb(msg):
    print msg.rect
    #引数のrectをprint
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #markerの宣言(importしたImageMarker2) その内部の値に代入
    pub.publish(marker)
    #publish "marker"に配信
#

rospy.init_node('client')
#init "client"という名前に定義する
rospy.Subscriber('/camshift/track_box' , RotatedRectStamped, cb)
#Subscriber RotatedRectStamped型の"/camshift/track_box"からを読み取り、cbに渡す
#メッセージが"/camshift/track_box"に来るたびにcbが実行される
pub = rospy.Publisher('/image_marker', ImageMarker2)
#Publisher 
rospy.spin()
#spin ノードが終了するまでループさせる

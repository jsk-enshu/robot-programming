#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#コールバック関数
def cb(msg):
    #矩形領域の表示
    print msg.rect
    #送信するマーカーを宣言
    marker = ImageMarker2()
    #markerのタイプ（円形）
    marker.type = 0
    #取得データをもとにmarkerを位置決め
    marker.position = Point(msg.rect.center.x, msg.rect.center.y,0)
    #送信
    pub.publish(marker)

#ノードの初期化
rospy.init_node('client')
#camshiftからRotatedRectStamped型を受け取る
#受け取るたびにcbを呼ぶ
rospy.Subscriber('/camshift/track_box',RotatedRectStamped, cb)
#image_markerにImageMarker2型を送信
pub = rospy.Publisher('/image_marker', ImageMarker2)
#このノードを終了させない
rospy.spin()

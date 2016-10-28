#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#/camshift/track_boxから受け取ったメッセージに対する処理
def cb(msg):
    #msg内のRotatedRect型のrectを表示
    print msg.rect
    #/image_markerに送出するmarkerはImageMarker2型
    marker = ImageMarker2()
    #markerのタイプと位置を指定
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y,0)
    #markerを送出
    pub.publish(marker)

#rospyに'client'というノード名を通知
rospy.init_node('client')
#RotatedRectStamped型のメッセージを/camshift/track_boxから受け取ることを宣言
#データを受け取った時呼び出す関数にcbを設定
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#/image_markerにImageMarker2型のメッセージを送出
pub = rospy.Publisher('/image_marker',ImageMarker2)
#ノード終了まで続ける
rospy.spin()

#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):  # コールバック関数
    print msg.rect  #  受信メッセージを表示
    marker = ImageMarker2()  # ImageMarker2型のmarkerを用意する
    marker.type = 0  # マーカーを円に設定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # 座標値をmsg.rect.xenterに設定する
    pub.publish(marker)  # markerを送信する

rospy.init_node('client')
# rospyにノード名を通知する
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# /camshift/track_boxというトピックからRotatedRectStamped型のメッセージを受け取る
# 受信時のコールバック関数をcbに設定する
pub = rospy.Publisher('/image_marker', ImageMarker2)
# /image_markerというトピックにImageMarker2型のメッセージを送ることを宣言
rospy.spin()
# ノードが停止するまでプログラムの実行を続ける

#!/usr/bin/env python
# -*- coding: utf-8 -*-

#rospyのモジュールをimport
import rospy

from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msg import Point

#コールバック関数の定義
def cd(msg):
    #マーカ情報をprint
    print msg.rect
    #markerをインスタンス化
    marker = ImageMarker2()
    #markerの種類を指定
    marker.type = 0
    #markerの位置情報を指定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #markerの送信
    pub.publish(marker)

#'client'という名前でnodeを立ち上げる
rospy.init_node('client')
#'/camshift/track_box'というトピックでメッセージを受信するとコールバック関数を呼び出す
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#'/image_marker'というトピックでメッセージを送信
pub = rospy.Publisher('/image_marker', ImageMarker2)
#nodeが終了するまで実行し続ける
rospy.spin()

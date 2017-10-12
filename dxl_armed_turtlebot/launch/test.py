#!/usr/bin/env python
# -*- coding: utf-8 -*-
#参考 : https://kazuyamashi.github.io/ros_lecture/ros_study_py.html 

#SubscriberとPublisherが両方共あるため、送ることも受け取ることもできるプログラム

#pythonでROSのソフトウェアを記述するときにimportするモジュールらしいですね
import rospy
#自分で定義したmessageファイルから生成されたモジュール
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
#Subscribeする対象のとpっくが更新されたら呼び出されるコールバック関数
#引数の型はRotatedRectStamped
def cb(msg):
    #座標を表示
    print msg.rect
    #ImageMarker2型のmarkerに情報を与える
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #iage_markerにmarkerを送る
    pub.publish(marker)

#初期化宣言,このソフトウェアはclientという名前
rospy.init_node('client')
#/camshift/track_boxというトピックからRotatedRectStampedを受け取り,トピックが更新された時はcb(コールバック関数)を実行
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#image_markerというtopicにImageMarker2の型を送るPublisher(?)
pub = rospy.Publisher('/image_marker', ImageMarker2)
#トピック更新の待受を行う関数
rospy.spin()

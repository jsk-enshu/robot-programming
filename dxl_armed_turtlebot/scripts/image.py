#!/usr/bin/env python
# coding: utf-8

#Python クライアントライブラリ
import rospy

#各メッセージ型を利用できるようにする
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point


#購読時のコールバック関数
def cb(msg):
    #受け取ったメッセージの情報を標準出力する
    print msg.rect

    #出版用のメッセージ(marker)を作成する
    #メッセージ型はImageMarker2
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    #作成したメッセージ(marker)を出版する
    pub.publish(marker)


#ノードを立ち上げるために、rospyにノード名を通知する
rospy.init_node('client')

#第一引数('/camshift/track_box')のトピックから
#第二引数(RotatedRectStamped)の型を持ったメッセージを購読する
#新しいメッセージを受け取るたびにそのメッセージを引数に第三引数のコールバック関数(cb)を実行する
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

#第一引数('/image_marker')のトピックに
#第二引数(ImageMaker2)の型を持ったメッセージを出版する
pub = rospy.Publisher('/image_marker', ImageMarker2)

#ノードが終了するまで処理を続ける
rospy.spin()

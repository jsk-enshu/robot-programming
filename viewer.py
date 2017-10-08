# -*- coding: utf-8 -*-
## 文字コードをutf-8とする という宣言
#! /usr/bin/env python
## 環境変数 PATH に含まれている python を探して実行するという宣言

import rospy
# rosの関数をpythonで使用可能にするモジュールをインポート
# ROS Node を書くために必要

# 以下、ROSのメッセージの型をpythonのクラスとしてインポート
from opencv_apps.msg import RotatedRectStamped
# rospy.Subscriber()で購読するメッセージのクラス
# http://docs.ros.org/kinetic/api/opencv_apps/html/msg/RotatedRectStamped.html
from image_view2.msg import ImageMarker2
# rospy.Publisher()で送信するメッセージのクラス
# http://docs.ros.org/kinetic/api/image_view2/html/msg/ImageMarker2.html
from geometry_msgs.msg import Point
# x,y,zをインスタンス変数にもち、空間内の点を表すクラス
# http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html

# rospy.Subscriber()で呼ばれるコールバック関数
def cb(msg):
    print msg.rect
    # 受け取ったメッセージを表示
    marker = ImageMarker2()
    # ImageMarker2クラスのオブジェクトを宣言
    # これをトピック「/image_marker」に送信する
    marker.type = 0
    # ImageMarker2クラスのインスタンス変数typeに0を代入
    # これによりマーカーを円に設定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # 追跡している長方形の中心（受け取ったメッセージ）をマーカーの中心として設定
    # ２次元なのでz座標は0で固定
    pub.publish(marker)
    # markerを送信

rospy.init_node('client')
# 新規ノードを'client'という名前で作成
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# 新しいメッセージがトピック'/camshift/track_box'に到着するたびに、そのメッセージを第1引数としてコールバック関数cbを呼ぶ
pub = rospy.Publisher('/image_marker', ImageMarker2)
# ノードが'/image_marker'というトピックにImageMarker2クラスのメッセージを送っていることを宣言
rospy.spin()
# このノードが停止するまでの間、pythonが終了するのを防ぐ



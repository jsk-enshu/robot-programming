#!/usr/bin/env python
# coding: utf-8

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
# rospyモジュールのインポート、及びにこのプログラムで利用する各種のメッセージ型のインポート

# /camshift/track_boxからメッセージを購読する際のコールバック関数
def cb(msg):
    print msg.rect
# RotatedRectStampedタイプのメッセージmsgのrectフィールドを出力する。

    marker = ImageMarker2()
# /image_markerトピックに配信するためのimage_view2.msg.ImageMarker2タイプのメッセージ 

    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
# 配信するメッセージの編集。

    pub.publish(marker)
# /image_markerトピックに編集したmarkerメッセージを配信

rospy.init_node('client') 
# rospy を'client'というノード名で初期化

rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #
# ノードが/camshift/track_boxというトピックを購読することを宣言
# また、トピックに配信されるメッセージの型がopencv_apps.msg.Rotatedrectstampedであること
# トピックに新しいメッセージが配信された時に呼び出される関数がcbであることを指定している

pub = rospy.Publisher('/image_marker', ImageMarker2, queue_size=10)
# ノードが/image_markerというトピックに配信することを宣言
# 配信するメッセージの型はImagemarker2であること、
# 購読する側が十分な速度で購読しない場合の、メッセージキューのサイズを10にすることを指定している

rospy.spin()
# ノードが停止するまでpythonプロセスを維持し続ける。

#!/usr/bin/env python
#pythonとして実行する

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#使用モジュールのインポート
#RotatedRectStamped ...　角度のついた短形の成分を送るメッセージの形式
#ImageMarker2 ...　線や円、そのIDや色の成分を送るメッセージの形式
#Point ... 点の性質を送るメッセージの形式

#
def cb(msg):
    print msg.rect
    #メッセージから受け取る短形の成分をprintする
    
    marker = ImageMarker2()
    marker.type = 0 #円
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # marker ... 中心が短形の中心の円
    
    pub.publish(marker)
    # /image_marker に marker というメッセージを送る

rospy.init_node('client')
#'client'という名前のnodeをrospyに通知する

rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#/camshift/track_box からRotatedRectStamped というタイプのメッセージを受け取る
#メッセージを受け取る度に cb(/camshift/track_box) を行う

pub = rospy.Publisher('/image_marker', ImageMarker2)
#/image_marker に ImageMarker2 というタイプのメッセージを送る

rospy.spin()
#nodeが終了するまで終了しない

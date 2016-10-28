#!/usr/bin/env python

#pythonとして実行

import rospy
#rospyをインポート
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
#A(親)の下にあるBをインポート

def cb(msg):
#コールバック関数
    print msg.rect
#rectを出力
    marker = ImageMarker2()
    marker.type = 0
#typeの設定
    marker.position = point(msg.rect.center.x, msg.rect.center.y, 0)
#positionを設定
    pub.publish(marker)
#chatterトピックに配信
rospy.init_node('client')
#clientというノードを配信
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#/cabshift/track_boxというトピックにRotatedRectStampedというタイプでcbを呼んでいる
pub = rospy.Publisher('/image_marker',ImageMarker2)
#/image_markerというトピックにImageMarker2というメッセージのタイプで送る
rospy.spin()
#自ノードを終了させない


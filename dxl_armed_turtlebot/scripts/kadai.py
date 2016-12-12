#!/usr/bin env python
# -*- coding: utf-8 -*-

#諸々インポート
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    #RotatedRect型のmessage(angle, center, sizeの3つの情報)をprint
    print msg.rect
    #ImageMarker2の変数markerを宣言 
    marker = ImageMarker2()
    #markerのtypeは円
    marker.type = 0
    #marker円の中心座標を設定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y,0)
    #markerを配信
    pub.publish(marker)
    

#rospyにノード名を通知
rospy.init_node('client')
#RotatedRectStampedタイプのcamshift/track_boxからデータを受け取り、受け取ったデータを第1引数として関数cbに送る
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#image_markerにImageMarker2という形式のデータを送る
pub = rospy.Publisher('/image_marker', ImageMarker2)
#ノードが終了するまでpythonを実行し続ける
rospy.spin()

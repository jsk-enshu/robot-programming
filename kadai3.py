#!/usr/bin/env python
# -*- coding: utf-8 -*-

#各種関数をimport
import rospy
from opencv_apps.msg import RotatedRectStamped
from image view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
	print msg.rect #受け取ったmsgを出力
	marker = ImageMarker2()#ImageMarker2のクラスのインスタンスを作成
	marker.type = 0#インスタンスのtypeを0に初期化
	marker.position=Point(msg.rect.center.x,msg.rect.center.y,0)
	pub.publish(marker)#markerを配信

rospy.init_node('client')#rospyにノード名を通知
rospy.Subscriber('/camshift/trace_box',RotateRecentStamped,cb)#RotateRecentStamped型の/camshift/trace_boxトピックから購買することの宣言。新しいメッセージがトピックに到着する度に、cbが起動される
pub=rospy.Publish('/image_marker',ImageMarker2)#ImageMarker2型のデータを/image_markerへ送る
rospy.spin()#自ノード終了させないようにする

#!/usr/bin/env python

#必要とする変数の型をインポートしている
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# /camshift/track_boxからメッセージが来るたび呼び出されるcallback関数
def cb(msg):
	print msg.rect    #RotatedRectStamped型の中にあるrectの情報を出力する
	marker = ImageMarker2() #配信するための変数の型をimage_view2.msg.ImgaeMarker2として定義する
	marker.type = 0 #円の場合は0を代入する
	marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #RotateRectstampedのなかの座標(x, y)をmarker.positionに代入する
	pub.publish(marker) #markerを配信する

rospy.init_node('client') #node名を'client'にする　
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #/camshift/track_boxというトピックからRotateRectStamped型の変数が来たら、コールバック関数が呼び出される
pub = rospy.Publisher('/image_marker', ImageMarker2) #pubが/image_markerというトピックからのImageMarker2という型の変数であることを宣言する
rospy.spin() #ノードが終了するまでの間、pythonプログラムを終了させないようにする

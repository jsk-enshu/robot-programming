#!/usr/bin/env python

import rospy #rosにおけるpythonのクライアントライブラリ
from opencv_apps.msg import RotatedRectStamped #opencv_apps/RotatedRectStampedのメッセージタイプを配信に再利用できる
from image_view2.msg import ImageMarker2 #image_view2/ImageMarker2のメッセージタイプを配信に再利用できる
from geometry_msgs.msg import Point #geometry_msgs/Pointのメッセージタイプを配信に再利用できる

def cb(msg):
	print msg.rect #受け取ったメッセージの矩形情報を垂れ流す
	marker = ImageMarker2() #小さい丸のマーカーの型を作る
	marker.type = 0 #マーカーの種類を決定
	marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #マーカーの位置を決定
	pub.publish(marker) #作成されたメッセージ(marker)を現在のトピック(/Image_marker)に送信する

rospy.init_node('client') #rospyにノード名をclientとして通知する
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #ノードがopencv_apps.RotatedRectStampedの方のcbトピックから購読することを宣言している
pub = rospy.Publisher('/image_marker', ImageMarker2) #ノードが/image_markerのトピックにImageMarker2の型でメッセージを送っている
rospy.spin() #プロセスが終了するまで、ずっとノードを保持する

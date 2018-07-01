#!/usr/bin/env python

#publish->送り手側、subscribe->受け手側

#必要なpackageのインポート
import rospy #ROS.orgより
from opencv_apps.msg import RotatedRectStamped　#ROS.orgより
from image_view2.msg import ImageMarker2 #ROS.orgより
from geometry_msgs.msg import Point #ROS.orgより

#関数の定義
def cb(msg):
    print msg.rect #矩形情報の表示
    marker = ImageMarker2() #markerの定義
    marker.type = 0 #markerを円形に設定 #CIRCLE/LINE_STRIP/etc.
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #markerの中心座標を矩形の中心に設定
    pub.publish(marker) #markerを送信

rospy.init_node('client') #ノード名の宣言(ROS Masterとの通信の開始)
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #(topic名, 型, callback関数) #ここではtopicはechoで出力したもの
pub = rospy.Publisher('/image_marker', ImageMarker2) #(topic名, 型, size)
rospy.spin() #node処理が終了するまで繰り返す

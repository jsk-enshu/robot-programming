#!/usr/bin/env python #pythonとして正確に実行

import rospy #Nodeを書くためrospyをimport
from opencv_apps.msg import RotatedRectStamped #RotatedRectStampedの使用のためimport
from image_view2.msg import ImageMarker2 #ImageMarker2の使用のためimport
from geometry_msgs.msg import Point #Pointの使用のためimport

def cb(msg): #ImageMarker2のメッセージ内容の配信とmarker付けした物体の追跡時の位置情報とサイズをプリント
    print msg.rect
    marker = ImageMarker2()
    marker.type = 0 #markerのタイプは円
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    pub.publish(marker) #markerのメッセージを配信

rospy.init_node('client') #rospyにノード名clientを通知,Masterに接続
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #/camshift/track_boxをRotatedRectStampedのメッセージタイプで受け取り,更新時に関数cbを呼ぶ
pub = rospy.Publisher('/image_marker', ImageMarker2) #/image_markerをImageMarker2のメッセージタイプで送信
rospy.spin() #プロセス終了まで繰り返す

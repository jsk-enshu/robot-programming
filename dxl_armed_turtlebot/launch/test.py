#!/usr/bin/env python
import rospy   #rospyをimport
from opencv_apps.msg import RotatedRectStamped #RotatedRectstampedをimport
from image_view2.msg import ImageMarker2 #ImageMarker2をimport
from geometry_msgs.msg import Point #Pointをimport

def cb(msg):
    print msg.rect
    marker =  ImageMarker2()　#ImageMarker２クラスからmarkerオブジェクトの作成
    marker.type = 0 #markerの種類は0（円）
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)#markerの位置をmsgで指定
    pub.publish(marker) #markerを送る

rospy.init_node('client') #clientという名前のnodeを立ち上げる
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)#RotatedRectStampedのタイプの/camshift/track_boxトピックから購読し、新しいメッセージが到達するたびにコールバック関数cbを実行する
pub = rospy.Publisher('/image_marker', ImageMarker2)#/image_markerノードにImageMarker2のメッセージタイプで送ることを宣言
rospy.spin() #nodeが終了するまでプログラムを実行し続ける

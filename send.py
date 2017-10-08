#!/usr/bin/env python 

#pythonでrosのプログラムを書く際に必要な基礎的なAPI
import rospy 

#使用するメッセージ型のimport
from opencv_apps.msg import RotatedRectStamped 
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#コールバック関数の定義
def cb(msg):
    #受信したメッセージを表示
    print msg.rect

    #ImageMarker2()型のインスタンスを生成
    marker = ImageMarker2()

    #markerの種類を円に設定
    marker.type = 0

    #markerの位置をmsg.rect.centerに設定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    #markerを送信
    pub.publish(marker)

    
#ノード名をclientに設定
rospy.init_node('client')

#トピック/camshift/track_boxからRotatedRectStamped型のメッセージを受け取ることを
#宣言
#トピックが更新された場合、コールバック関数cbを実行する
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

#トピック/image_markerにImageMarker2型のメッセージを送ることを宣言
pub = rospy.Publisher('/image_marker', ImageMarker2)

#ノードが停止するまでプログラムを回し続ける
rospy.spin()

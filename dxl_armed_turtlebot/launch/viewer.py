#!/usr/bin/env python

#必要なモジュールのインポート
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#以下コールバック関数の定義
def cb(msg):
    #受信したメッセージの表示
    print msg.rect
    #ImageMarker2クラスのインスタンスの生成
    marker = ImageMarker2()
    #markerの種類を円として指定
    marker.type = 0
    #markerの位置の指定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #markerの送信
    pub.publish(marker)
    
#rospyにノード名を通知
rospy.init_node('client')
#/camshift/track_box というトピックからRotatedRectStamped という型のメッセージを受信
#コールバック関数をcbと指定
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#/image_markerというトピックにImageMarker2という型のメッセージを送信
pub = rospy.Publisher('/image_marker', ImageMarker2)
#このノードが停止するまでの間プログラムが終了しないことを保証
rospy.spin()


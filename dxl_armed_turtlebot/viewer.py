#!/usr/bin/env python
#実行可能権を付与している

import rospy #rospyをimportしている。rospyはpythonとrosのツールをつなぐ役割がある。
from opencv_apps.msg import RotatedRectStamped #open_apps.msgからRotatedRecrStampedをインポート。
from image_view2.msg import ImageMarker2 #image_view2.msgからImageMarker2をインポート。
from geometry_msgs.msg import Point #geometry_msgs.msgからPointをインポート。

#コールバック関数
def cb(msg):   
    print msg.rect #msg.rectを出力。角度、中心、サイズの情報を持つ。
    marker = ImageMarker2() #変数の宣言
    marker.type = 0 #
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #変数の位置を決定。
    pub.publish(marker) #マーカーをパブリッシュ。

rospy.init_node('client') #clientというノードを立ち上げる。
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #/camshift/tarack_boxからRotatedRectStampedを受け取り、cbを実行する。
pub = rospy.Publisher('/image_marker', ImageMarker2) #/image_markerにImageMarker2の情報を渡す。
rospy.spin() #ノードが終了するまで処理を続行


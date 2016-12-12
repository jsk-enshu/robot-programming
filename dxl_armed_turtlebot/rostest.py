#! /user/bin/env/python

#rospyの使用を宣言
import rospy

#必要なファイルをインポート
from opencv_apps.msg import RotateRectStamped
from image_view2.msg import ImageMaker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect  #msgクラスの領域を表示
    marker = ImageMarker2()  #マーカーの型を設定
    marker.type = 0  #マーカーのタイプを円に設定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) 
    #マーカーのポジションを領域の中央の座標に設定

    pub.publish(marker)  #マーカーをセット

rospy.init_node('client')  #rospyにノード名を通知して、ノードを開始する
rospy.Subscriber('/camshift/track_box', RotateRectStamped, cb)
#track_boxからRotateRectStamped.msgを受け取り関数cbに渡す

pub = rospy.Publisher('/image_marker', ImageMarker2)
#'/image_marker'にImageMarker2という型でマーカーを送ることを宣言

rospy.spin()  #ノードが終了するまで自ノードを維持

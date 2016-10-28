#!/usr/bin/env python
#この一行でこのスクリプトファイルがpythonスクリプトとして確実に実行される。

#ROSの実行に必要なインポート
import rospy
#opencvの処理のライブラリ
from opencv_apps.msg import RotatedRectStamped
#描画ライブラリ
from image_view2.msg import ImageMarker2
#幾何学的な基本要素のライブラリ
from geometry_msgs.msg import Point

def cb(msg):
#RotatedRectStamped型の変数の内容を表示
    print msg.rect
#ImageMarker2型の変数markerを定義
    marker = ImageMarker2()
#円を作る
    marker.type = 0
#重心を代入
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
#生成したメッセージを配信
    pub.publish(marker)

#rospyにノード名を通知
rospy.init_node('client')

#ノードがopencv_apps.msgs.RotatedRectStamped型の/camshift/track_boxというトピックから受信する。トピックが更新された時に関数cbを呼ぶ
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

#ノードが/image_markerというトピックにImageMarker2というメッセージの型で送っていることを宣言
pub = rospy.Publisher('/image_marker', ImageMarker2)

#ノードが勝手に終了しないようにする
rospy.spin()

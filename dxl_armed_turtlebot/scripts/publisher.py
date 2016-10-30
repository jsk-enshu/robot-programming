#!/usr/bin/env python


# 必要なライブラリをインストール
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point



def cb(msg):                   # msgクラスを引数にとる関数cbを宣言

    print msg.rect             # msgクラスのrect(領域の中心座標と大きさ)を表示
    marker = ImageMarker2()    # ImageMarker2型の変数makerを定義　
    marker.type = 0            # 領域のタイプを指定(0は円)
                               # markerのpositionに領域の中心座標を代入
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    pub.publish(marker)        # image_markerに情報を送る

rospy.init_node('client')      # rospyでclientノードを開始
                               # /camshift/track_boxからRotatedRectStampedメッセージを受信
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
                               # image_markerに情報を送る
pub = rospy.Publisher('/image_marker', ImageMarker2)
rospy.spin()                   # ノードが終了するまで終了しない

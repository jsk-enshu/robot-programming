#!/usr/bin/env python

import rospy #python用のrosをインポート
from opencv_apps.msg import RotatedRectStamped #opencvの長方形に関するパッケージ
from image_view2.msg import ImageMarker2 #マーカーのパッケージ
from geometry_msgs.msg import Point #座標の指示に用いるパッケージ

def cb(msg): #msgを引数とする関数cb(call back)を定義する。
    print msg.rect #msg.rectをターミナルに表示する。
    marker = ImageMarker2() #関数ImageMarker2()を読み込みmarkerに代入し初期化する。
    marker.type = 0 #markerの種類を指定する。この場合は円。
    marker.position = Point(msg.rect.center.x,msg.rect.center.y,0) #markerの位置をmsgに基づき指定する。
    pub.publish(marker) #rosoutへmarkerを書き込む。

rospy.init_node('client') #ノードの名前を決めてroscoreに接続する。
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #"/camshift/trackbox"というトピックが更新された時に関数cbを呼ぶ。
pub = rospy.Publisher('/image_marker',ImageMarker2) #"/image_marker"という名前でString型のトピックを発行することを宣言する。
rospy.spin() #プロセス終了までアイドリングさせる。

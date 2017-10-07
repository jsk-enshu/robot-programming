#! /usr/bin/env python
# スクリプトがpythonで実行されるようにする


import rospy #モジュールからrospyをインクルードしrosの関数をpythonでも使用可能にする
from opencv_apps.msg import RotatedRectStamped　#opencv_appsからRotatedRectStampedというメッセージの型を受け取る(RotatedRectは回転した長方形)
from image_view2.msg import ImageMarker2　#image_view2からImageMarker2というメッセージの型を受け取る(ImageMarkerはイメージを送受信するための型)
from geometry_msgs.msg import Point　#geometry_msgsからPointというメッセージの型を受け取る(geometry_msgsはイメージの原型のためのメッセージ、Pointは座標を送受信するための型)

def cb(msg):　#callback関数の定義
    print msg.rect　#msgからrectについての情報を得てそれを出力する（rectの情報は原点のx,y座標、高さ、幅）
    marker = ImageMarker2()　#markerの初期化
    marker.type = 0　#markerの種類の定義　０は円　（参照　http://docs.ros.org/fuerte/api/image_view2/html/msg/ImageMarker2.html）
    marker.position = Point(msg.rect.center.x, msg.rect.center.y,0)　#markerの位置の定義　x,y座標はメッセージから得られた長方形の中心、ｚ座標は０
    pub.publish(marker)　#パブリッシャ関数を呼び出し、markerを送信する。pub.publishは３種類の使い方があり、Explicit Styleを今回用いている。Explicit Styleでは自身で定義したメッセージインスタンスを引数とすることで、そのインスタンスをそのまま送信できる。

rospy.init_node('client')　#rosのノードを起動する。名前はclienとなる
rospy.Subscriber('/camshift/track_box',RotatedRectStamped, cb)　#Subscriberを起動し'/camshift/track_box'からデータを受け取る。データの型はRotatedRectStamped,コールバック関数（呼びだされた時に処理を行う関数）は先程定義したcb
pub = rospy.Publisher('/image_marker',ImageMarker2) #Publisherを起動する。'image_marker'にImageMarker2という型のデータを送信する
rospy.spin()　#ノードが停止するまでpythonが終了しないことを保証する

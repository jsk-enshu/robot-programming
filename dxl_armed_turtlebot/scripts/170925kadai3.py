#!/usr/bin/env python
# coding: UTF-8

#pythonでROSのノードを書く場合に先頭に#!の1行を書くことで、スクリプトファイルがpythonスクリプトとして確実に実行される。rosrun [パッケージ] ***.py のコマンドで実行する場合には必要になるだろうが、python ***.pyのコマンドで実行する場合はこの1行は必要ないのではないかと考える。

#ROSのコードを書くために一般に必要な関数・型をインポート
import rospy
from opencv_apps.msg import RotatedRectStamped
#RotatedRectStamped.msgは、
# Header header
# RotatedRect rect
#からなる
#RotatedRect.msgは、
# float64 angle
# Point2D center
# Size size
#からなる
from image_view2.msg import ImageMarker2
#ImageMarker2.msgは、多くのメンバを持ち、その一部が
# byte CIRCLE=0
# int32 type
# geometry_msgs/Point position
from geometry_msgs.msg import Point
#Point.msgは、
# float64 x
# float64 y
# float64 z
#からなる

def cb(msg):
    #angle center sizeを出力
    print msg.rect
    #Imagemarker2型のmarkerの初期化
    marker = ImageMarker2()
    #タイプをCIRCLEに設定
    marker.type = 0
    #位置を受信したrectのcenterの位置に設定
    marker.position = Point(msg.rect.center.x,msg.rect.center.y,0)
    #markerをメッセージとして送る
    pub.publish(marker)

#ノード名をclientに設定、同名のノードは同時に存在できないことに注意
rospy.init_node('client')
#/camshift/track_boxにメッセージが送られた時に呼び出すコールバック関数をcbに設定。RotatedRectStamped型として処理する。
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#/image_markerにImageMaeker2型のメッセージを送ることを宣言
pub = rospy.Publisher('/image_marker',ImageMarker2)
#以下、コールバック関数を必要に応じて呼び出す無限ループ
rospy.spin()


#参考http://wiki.ros.org/ja/ROS

#!/usr/bin/env python

# 直接実行可能にする

# 各種インポート
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#コールバック関数
def cb(msg):
    #RotatedRectStamped型のRotatedRect型の変数の内容をを表示
    print msg.rect
    #ImageMarker2型の変数を作成
    marker = ImageMarker2()
    #図形は円
    marker.type = 0
    #msg.rectが表す図形の重心位置を代入
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #メッセージとしてmarkerをもつトピック/image_markerを送出する
    pub.publish(marker)

#clientというノードを登録する
rospy.init_node('client')
#/camshift/track_boxというトピックが更新された時RotatedRectStamped型のメッセージを受け取りcbを呼び出す
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#ImageMarker2型のメッセージを持つ/image_markerというトピックを作成
pub = rospy.Publisher('/image_marker', ImageMarker2)
#ノードが終了命令を受けるまでスリープ
rospy.spin()

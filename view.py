#!/usr/bin/env python

#各種インポート
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point


def cb(msg):
    #msgのrectフィールドをプリント
    print msg.rect
    #ImageMarker2型の変数を生成
    marker=ImageMarker2()
    #図形を円とする
    marker.type=0
    #msg.rectの中心を指定
    marker.position=Point(msg.rect.center.x,msg.rect.center.y,0)
    #/image_markerトピックにmarkerを送る
    pub.publish(marker)

#initというノード名でrospyを初期化する
rospy.init_node('client')
#/camshift/track_boxというトピックに送られるRotatedRectStamped型のメッセージを購読し、更新された時にそれを引数として関数cbを呼び出す
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#/image_markerというトピックにImageMarker2型のメッセージを送る
pub=rospy.Publisher('/image_marker',ImageMarker2)
#ノード終了までプロセスを維持
rospy.spin()

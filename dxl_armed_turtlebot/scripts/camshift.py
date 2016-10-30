#!/usr/bin/env python

#ライブラリからファイルをインポートする.
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import  ImageMarker2
from geometry_msgs.msg import Point

#コールバック関数を定義する.
def cb(msg):
    print msg.rect
    #RotatedRectStamped型の変数内容を表示する.
    marker = ImageMarker2()
    #ImageMarker2型の変数markerを作る.
    marker.type = 0
    #円に指定する.
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #重心を代入する.
    pub.publish(marker)
    #メッセージ'marker'をトピックに配信する.

rospy.init_node('client')
#rospyに'client'というノード名を通知する.
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#ノードがRotatedRectStamped型の/camshift/track_boxトピックから購読することを宣言する.新しいメッセージがトピックに到着するたびにメッセージを第一引数としてcbが起動する.
pub = rospy.Publisher('/image_marker', ImageMarker2)
#ノードが'image_marker'というトピックにImaegMarker2というメッセージのタイプで送る.
rospy.spin()
#ノードが終了するまでの間,自ノードが終了しないようにする.

#!/usr/bin/env python
# 必要なpackageのimport
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect#msgのrectの表示
    marker=ImageMarker2()#markerの定義
    marker.type=0#markerを円にする。　ImageMarker2においてint32 type # CIRCLE/LINE_STRIP/etc.,byte CIRCLE=0
    marker.position=Point(msg.rect.center.x,msg.rect.center.y,0) #ImageMarker2においてgeometry_msgs/Point position # used for CIRCLE/TEXT, 2D in pixel-coords.マーカーの位置を、msg.rectの中心に設定する。
    pub.publish(marker)#markerを送信する

    rospy.init_node('client')#nodeの名前（これによりMasterと通信できるようになる）
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)#RotatedRectStamped型のメッセージを、'/camshift/track_box'というtopicからうけとり、cbを受け取ったメッセージを第一引数とするコールバック関数として用いる。
pub=rospy.Publisher('/image_marker',ImageMarker2)#'/image_marker'というtopicにImageMarker2型のメッセージを送信する
rospy.spin()#シャットダウンされるまでノードをexitさせない

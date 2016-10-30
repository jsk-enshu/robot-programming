#!/usr/bin/env python

#必要なモジュールをインポート
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#メッセージを購読し配信するコールバック関数
def cb(msg):
    print msg.rect  #受け取ったメッセージのrectプロパティを表示

    marker = ImageMarker2()
    marker.type = 0

    #受け取ったmsgを格納
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    pub.publish(marker)　#markerを配信

rospy.init_node('client')　#clientという名前でノードを初期化

#'/camshift/track_box'というトピックからメッセージを購読する
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) 

#'image/marker'というトピックにメッセージを配信
pub = rospy.Publisher('/image_marker', ImageMarker2)

rospy.spin() #ノードが終了するまでプロセスを終了しないようにする

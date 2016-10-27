#!/usr/bin/env python

# 必要なファイルをインポートする
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# メッセージを購読し配信するコールバック関数
def cb(msg):

    # 受け取ったメッセージのrectプロパティを表示
    print msg.rect

    # markerの型を設定
    marker = ImageMarker2()
    marker.type = 0

    # marker.positionに受け取ったmsgを格納
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    # markerを使って配信
    pub.publish(marker)

# 'client'という名前のノードを立ち上げる
rospy.init_node('client')

# '/camshift/track_box'というトピックからRotatedRectStamped型をメッセージを購読し、
# コールバック関数を呼ぶ
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

# '/image_marker'というトピックにImageMarker2型のメッセージを配信する宣言
pub = rospy.Publisher('/image_marker', ImageMarker2, queue_size = 10)

# ノードが終了するまで自ノードを終了させないようにする
rospy.spin()

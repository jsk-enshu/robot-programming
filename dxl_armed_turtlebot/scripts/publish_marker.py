#!/usr/bin/env python

# パブリッシャ・サブスクライバに関するチュートリアル
# http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# ROSノードを書く際はrospyをインポートする必要がある.
import rospy
# 各種型のインポート
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# メッセージがトピックに到着した際のcallback
def cb(msg):
	# 受け取ったメッセージ(引数)をprintして確認
	print msg.rect
	# ImageMarker2型のインスタンスmarkerの宣言
	marker = ImageMarker2()
	# markerのtypeに0をセット
	marker.type = 0
	# markerのpositionに重心座標をセット
	marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
	# markerを送出
	pub.publish(marker)

# ノード名'client'で初期化
rospy.init_node('client')
# 新しいメッセージがトピックに到着するたびにcbを呼び出す
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# ノードが'/image_marker'というトピックにImageMarker2型でメッセージを送ることを宣言
pub = rospy.Publisher('/image_marker', ImageMarker2)
# 自ノードを終了させない
rospy.spin()


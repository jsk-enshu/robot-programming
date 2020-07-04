#!/usr/bin/env python

# 必要なモジュールのインポート
import rospy
from opencv_apps.msg import RotatedRectStamped #領域の切り出しに関して
from image_view2.msg import ImageMarker2 #画像表示に関して
from geometry_msgs.msg import Point #座標に関して

# 購読時に起動する関数、画像処理結果から表示に必要な情報をまとめたmarkerを作成して配信
def cb(msg):
  print msg.rect
  marker = ImageMarker2()
  marker.type = 0
  marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
  pub.publish(marker)

# 他ノードとの通信
rospy.init_node('client') #ノード名をclientとして通知
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #camshiftによる画像処理結果を購読
pub = rospy.Publisher('/image_marker', ImageMarker2) #画像上に表示するマーカーを配信
rospy.spin() #ノードが止まるまでpythonの実行をキープ

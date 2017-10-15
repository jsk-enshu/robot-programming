#!/usr/bin/env python
#rospyのモジュールを使うことを宣言している
import rospy
# RotatedRectStamped,ImageMarker2,Pointは以下のプログラムでパッケージ名をつけなくても良いようにしている
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
  #受け取ったメッセージを送信する
  print msg.rect
  marker=ImageMarker2()
  marker.type=0
  marker.position=Point(msg.rect.center.x,msg.rect.center.y,0)
  pub.publish(marker)
#ノードの名前を決める
rospy.init_node('client')
#メッセージを受信したらcb関数を呼び出す
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#メッセージの送信を定義する
pub=rospy.Publisher('/image_marker',ImageMarker2)
rospy.spin()


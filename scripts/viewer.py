#!/usr/bin/env python

# ROS pythonと利用する各メッセージ型をインポートする　
import rospy 
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# /camshift/track_boxからメッセージを購読する際のコールバック関数
def cb(msg):
    # RotatedRectStampedからrectフィールドを出力する。
    print msg.rect
    
    # 配信するため、marker変数を作って、image_view2.msg.ImageMarker2タイプのメッセージをmarkerに代入する
    marker = ImageMarker2()
    
    # markerの中身を編集
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    
    # markerを配信する
    pub.publish(marker)

# rospyに'client'というノード名を初期化する
rospy.init_node('client')
# '/camshift/track_box'というトピックから購読することを宣言する。
# 新しいメッセージがトピックに到着するたびに、メッセージを第一引数として
# コールバック関数cbが起動される
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# /camshift/trackboxというトピックにRotatedRectStampedという型のメッセージを送っていることを宣言する
pub = rospy.Publisher('/image_marker', ImageMarker2)
# ノードが終了するまでの間、pythonプロセス終了させないようにしています
rospy.spin()

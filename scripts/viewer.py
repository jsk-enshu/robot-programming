#!/usr/bin/env python

# rospyと利用する各メッセージ型をインポートする　
import rospy 
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# /camshift/track_boxからメッセージを購読する際のコールバック関数
def cb(msg):
    # RotatedRectStamped型のmsgのrectに含まれる情報を出力する。
    print msg.rect
    
    # 配信するため、marker変数を作って、image_view2.msg.ImageMarker2型のメッセージをmarkerに代入する
    marker = ImageMarker2()
    
    # markerの中身を編集
    ## 円の場合は０を指定
    marker.type = 0　
    ## 中心座標(x,y,z)をmarker.positionに代入する
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    
    # markerを配信する
    pub.publish(marker)

# rospyに'client'というノード名を初期化する
rospy.init_node('client')
# '/camshift/track_box'というトピックからRotatedRectStamped型のメッセージを購読することを宣言する。
# 新しいメッセージがトピックに到着するたびに、メッセージを第一引数としてコールバック関数cbが起動される
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# ノードが"/image_marker"トピックから"ImageMarker2"型のメッセージを送っていることを宣言する
pub = rospy.Publisher('/image_marker', ImageMarker2)
# ノードが終了するまでの間、pythonプロセス終了させないようにしています
rospy.spin()

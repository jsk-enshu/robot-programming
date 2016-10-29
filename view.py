# -*- coding: utf-8 -*-
#スクリプトファイルがPythonスクリプトとして実行される
#!/usr/bin/env python

#rospyをインポートする。ROS Nodeを書く際に必要。
import rospy
#様々な変数の型をインポートする。
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#RotatedRectStamped型のmsgのrectに含まれている内容を表示する。
def cb(msg):
    print msg.rect

#購読したトピックmsgを使ってmarkerに値を代入する。
    marker = ImageMarker2()

#配信するメッセージの編集する。
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

#/ImageMarkerトピックに編集したmarkerメッセージを追加する。
    pub.publish(marker) 

#'client'というノード名でrospyを初期化する。
rospy.init_node('client')

#"RotatedRectStamped"型の/"/camshift/track_box"というメッセージを購読する。新しいメッセージが到達するたびに、それを第一引数として関数"cd"が起動する。
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

#ノードが"ImageMarker2"型のメッセージを"/image_marker"トピックに配信する。
pub = rospy.Publisher('/image_marker',ImageMarker2)

#ノードが終了するまで、単純に自ノード終了させないようにする。
rospy.spin()


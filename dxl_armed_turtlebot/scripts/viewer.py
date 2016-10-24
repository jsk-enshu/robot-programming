#!/usr/bin/env python
# -*- coding: utf-8 -*-

#ROSに必要な様々な関数を読み込む。
import rospy

#様々な変数の型を読み込む。異なる言語間で通信するために必要となる。
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
#RotatedRectStamped型のメッセージmsgにはHeader型のheaderとRotatedRect型のrectが含まれているが、そのrectに含まれている内容をprintで表示している。rectに含まれている内容とは、float64型のangleとPrint2D型のcenterとSize型のsizeである。
    print msg.rect

#購読したトピックmsgを使って配信するトピックmarkerに値を代入する。
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

#トピックを配信する。
    pub.publish(marker)

#rospyにノード名を通知する。
rospy.init_node('client')

#ノードがRotatedRectStampedという型の/camshift/track_boxというメッセージを購読することを宣言している。また、メッセージが到着するたびに、メッセージを引数としてcdという関数を呼び出す。
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

#/camshift/trackboxというトピックにRotatedRectStampedという型のメッセージを送っていることを宣言している。
pub = rospy.Publisher('/image_marker', ImageMarker2)

#ノードが終了するまでrospy.spin()がノードを終了させないようにしている。
rospy.spin()

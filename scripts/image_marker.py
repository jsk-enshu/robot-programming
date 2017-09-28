#/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect
    #markerオブジェクトを初期化する
    marker = ImageMarker2()
    marker.type = 0
    #長方形の座標をpositionに設定する
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #メッセージを送る
    pub.publish(marker)

#"client"というノードを作成
rospy.init_node('client')
#Subscriberを作成
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#Publisherを作成
pub = rospy.Publisher('/image_marker', ImageMarker2)
#プロセスが終了するまで回す
rospy.spin()

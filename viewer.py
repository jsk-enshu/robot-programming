#!/usr/bin/env python

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    #呼び出されるコールバック関数
    print msg.rect#angle,center,size情報を出力
    marker = ImageMarker2()#markerの初期化
    marker.type = 0#markerのタイプを円に指定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y,0)#markerの位置をrectの中心に指定
    pub.publish(marker)#markerを送出

rospy.init_node('client')
#'client'という名のROSノードを初期化、roscoreに接続される
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#'/camshift/track_box'というトピックが更新されたらコールバック関数を呼ぶ
pub = rospy.Publisher('/image_marker',ImageMarker2)
#'/image_marker'という名のトピックの発行を宣言
rospy.spin()
#プロセス終了までアイドリング

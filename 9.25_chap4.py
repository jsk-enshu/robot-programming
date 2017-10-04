#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):#msgを引数としたコールバック関数
    print msg.rect #msgのrect(center.x,center.y,width,height)
    marker = ImageMarker2()
    marker.type =0#マーカーの形を円にする
    marker.position = Point(msg.rect.center.x,msg.rect.center.y,0)#位置指定
    pub.publish(marker)#トピックにmarkerを配信する

rospy.init_node('client')#ノードの初期化
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
pub = rospy.Publisher('/image_marker', ImageMarker2)
#トピックにメッセージを公開するためのハンドルを作成
#('topic　name',Message class, queue_size)

rospy.spin()#終わらない

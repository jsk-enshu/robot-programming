#!/usr/bin/env python

import rospy #
from opencv_apps.msg import RotatedRectStamped #
from image_view2.msg import ImageMarker2 #
from geometry_msgs.msg import Point #

def cb(msg): #
	print msg.rect #
	marker=ImageMarker2() #marker は　imagemarker2型
	marker.type=0 #CIRCLE=0
	marker.position=Point(msg.rect.center.x, msg.rect.center.y,0) #マーカーの位置
	pub.publish(marker) #'client'というノードに’marker'を送出

rospy.init_node('client') #init_node(ノードの名前)
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#Subscriber(データを取ってくるノード、ノードの型、受け取ったデータをもとにデータを送出する関数)
pub = rospy.Publisher('/image_marker', ImageMarker2)
#Ｐｕｂｌｉｓｈｅｒ(送出するノード、ノードの型)
rospy.spin()#ノードがとまるまで存在し続ける。

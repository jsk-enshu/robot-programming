#!/usr/bin/env python

import rospy #rospyというライブラリをインポート
from opencv_apps.msg import RotatedRectStamped #前者のファイルから後者のライブラリにインポート
from image_view2.msg import ImageMarker2 #上に同じ
from geometry_msgs.msg import Point #上に同じ

def cb(msg): #cbという関数を定義
    print msg.rect
    marker = ImageMarker2() #インポートしたライブラリを利用してmarkerを指定
    marker.type = 0 #markerのtype成分を指定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #markerのposition成分を指定
    pub.publish(marker) #

rospy.init_node('client') #
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #/camshift/track_boxを受け取る
pub = rospy.Publisher('/image_marker', ImageMarker2) #/image_markerを送出する
rospy.spin() #


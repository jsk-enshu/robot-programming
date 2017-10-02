# -*- coding: utf-8 -*-
#!/usr/bin/env python
#Pythonスクリプトとして実行するための宣言

import rospy #rospyはROS Nodeを書くために必要
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
#使用するメッセージのタイプをimportする

#callback関数の作成
def cb(msg):
    print msg.rect #引数となっているメッセージのrectを表示
    marker = ImageMarker2() #markerの宣言
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0 ) #markerの位置指定
    pub.publish(marker) #下で宣言したトピックにmarkerを配信

rospy.init_node('client')
#rospyにノード名を通知する ノード名はbase nameを使用する

rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#/camshift/track_box というトピックから RotatedRectStamped というメッセージのタイプを受け取ることを宣言し、メッセージを第一引数としてcallback関数cbを起動する

pub = rospy.Publisher('/image_marker',ImageMarker2)
#/image_marker というトピックに ImageMarker2 というメッセージのタイプを送信するということを宣言する

rospy.spin()
#rospy.spin()によって自ノードの終了を防ぐ

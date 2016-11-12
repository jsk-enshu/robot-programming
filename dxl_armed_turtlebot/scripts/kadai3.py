#-*- coding:utf-8 -*-
#!/usr/bin/env python

#ROSで扱う関数をimportする
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#pythonで中身を記述して、msgに入ったものをROSの共通言語に翻訳しサーバーに送信する
def cb(msg):
    print msg.rect
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x,msg.rect.center.y, 0)
    pub.publish(marker)

#ノードの追加
rospy.init_node('client')
#第一引数（第二引数という型の）をcbで読み取る
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#第一引数へ第二引数の型のデータを送る
pub = rospy.Publisher('/image_marker',ImageMarker2)
#ノードの終了まで続ける
rospy.spin()

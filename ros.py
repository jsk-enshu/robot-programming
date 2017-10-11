#!/usr/bin/env python

import rospy#rospyをインポートする
from opencv_apps.msg import RotatedRectStamped#回転するためにインポートする
from image_view2.msg import ImageMarker2#markerを作るためにインポートする
from geometry_msgs.msg import Point#markerを座標指定するためにインポートする

def cb(msg):#コールバック関数を定義
    print msg.rect#msgのrectの情報を表示する
    marker =ImageMarker2()#markerオブジェクトを作る
    marker.type=0#markerの種類を指定（円）
    marker.position=Point(msg.rect.center.x, msg.rect.center.y, 0)#markerの位置を指定
    pub.publish(marker)#markerを送信する

rospy.init_node('client')#名前がclientのnodeを立ち上げる
rospy.Subscriber('/camshift/tracj_box',RotatedRectStamped,cb)#メッセージが/camshift/tracj_boxに到達したらコールバック関数cbを呼び出す
pub=rospy.Publisher('/image_marker',ImageMarker2)#メッセージを/image_markerにImageMarker2の型で送る
rospy.spin()#nodeが終了するまでプログラミングを実行し続ける

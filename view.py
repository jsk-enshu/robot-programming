#!/usr/bin/env python　　　スクリプトファイルをpythonファイルとして扱えう

import rospy #rospyライブラリをインポート
from opencv_apps.msg import RotatedRectStamped#from A import BでAからBにインポート
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#コールバック関数を定義
def cb(msg):
    print msg.rect#変数内容を表示
    marker = ImageMarker2()#ImageMarker2型の変数を生成
    marker.type = 0#円に指定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)#重心
    pub.publish(marker)#新しいメッセージをimage_markerトピックに配信
    
rospy.init_node('client')#rospyにノード名clientを通知。マスターと通信するのに必要
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)#RotatedRectStamped型の/camshift/track_boxトピックから購読することを宣言
pub = rospy.Publisher('/image_marker', ImageMarker2) #nodeがimage_markerにImageMarker2というメッセージのタイプで送信していることを宣言
rospy.spin()#ノードが終了するまで自ノードが終了するのを防ぐ

#!/usr/bin/env python

import rospy #ROS用のpythonライブラリ
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):#関数の定義
    print msg.rect#msgのcenter.x,center.y,width,heightの表示
    marker = ImageMarker2()#ImageMarker2という型のインスタンスを作成
    marker.type = 0#markerの種類を円に指定
    marker.position = Point(msg.revt.venter.x, msg.rect.center.y, 0)#markerの位置を指定
    pub.publish(marker)#markerを送信

rospy.init_node('client')#rospyにノード名を通知
rospy.Subscriber('/camshift/track_box' , RotatedRectStamped,cb)#メッセージを受け取る度に cb(/camshift/track_box) を行う
+
pub = rospy.Publisher('/image_marker',ImageMarker2)#'image_marker'にImageMarker2という型のデータを送る
rospy.spin()#node終了までプログラムの実行を続ける

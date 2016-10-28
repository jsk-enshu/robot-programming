#!/usr/bin/env python  #このスクリプトファイルがPythonスクリプトとして確実に実行する
# -*- coding: utf-8 -*-  #日本語でコメントアウトするために追加

import rospy  #ROSノード、つまりROSパッケージ内の実行ファイルを作成する際にインポートする必要がある

#トピックを購読するために必要な変数の型RotatedRectStamped,ImageMarker2,Pointをそれぞれopencv_apps.msg,image_view2.msg,geometry_msgs.msgから読み込む
from opencv_apps.msg import RotatedRectStamped  
from image_view2.msg import ImageMarker2  
from geometry_msgs.msg import Point

def cb(msg):

    #購読したメッセージmsgを表示し、配信するトピックmarkerにその値を代入する
    print msg.rect  
    marker = ImageMarker2()
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    pub.publish(marker)  #トピックを配信する

rospy.init_node('client')  #rospyにノード名を通知する rospyがこの情報を得ない限り、ROSのMasterと通信を始めることができないので重要
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
pub = rospy.Publisher('/image_marker', ImageMarker2)  #ノードが/imagemarkerというトピックにImageMarker2というメッセージのタイプで送っていることを宣言している
rospy.spin()

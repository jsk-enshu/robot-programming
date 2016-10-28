#!/usr/bin/env python

# rospyモジュールのインポート
import rospy

# メッセージ型のインポート
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# メッセージを購読して配信するコールバック関数
def cb(msg):
    
    # 受信したメッセージのrectフィールドを作る
    print msg.rect

    # /image_markerトピックに配信するimage_view2.msg.ImageMarker2タイプのメッセージ
    marker = ImageMarker2()

    # 円のモード
    marker.type = 0

    # 重心位置
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    # トピックを配信する
    pub.publish(marker)

# rospyのノード名をclientで初期化する
rospy.init_node('client')

# /camshift/track_boxというトピックが更新された時に、RotatedRestStamped型のメッセージを受け取りcbを呼び出す
rospy.Subscriber('/camshift/track_box', RotatedRectStamped,cb)

# ImageMarker2型のメッセージを持つ/image_markerというトピックを作成
pub = rospy.Publisher('/image_marker', ImageMarker2)

# ノードが終了命令を受けるまでスリープ
rospy.spin()

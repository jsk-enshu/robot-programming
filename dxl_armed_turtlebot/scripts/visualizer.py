#!/usr/bin/env python

# ros及び各メッセージの定義をimport。fromなのでglobal scope
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# message受信時のcallback
def cb(msg):
    # 検出された四角形の情報を標準出力にprint
    print msg.rect

    # 画像処理結果ビジュアライザに送るための構造体を準備
    marker = ImageMarker2()
    
    # type指定。0はCIRCLE
    marker.type = 0
    
    # 座標指定。四角形の中心を中心とする。二次元画像に出すのでzは0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    
    # 配信
    pub.publish(marker)

# ノード名clientで作成
rospy.init_node('client')

# 画像認識の結果を購読。RotatedRectStampedのmessageが/camshift/track_boxという名前のnodeから来たらcbを実行。
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

# 円を描画するためのmarkerを送るpublisherを作成。配信自体は上記のcallback内から。
pub = rospy.Publisher('/image_marker', ImageMarker2)

# 無限ループで待つ(ログを表示)
rospy.spin()

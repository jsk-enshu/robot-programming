#!/usr/bin/env python
# coding: UTF-8

import rospy # ros及び各メッセージの定義をimportしている
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# messageを受信したときにcallbackされる関数を定義する
def cb(msg):
    # 検出されたrectangleを標準出力にprintする
    print msg.rect

    # 画像処理結果の構造体を準備している
    marker = ImageMarker2()

    # type指定。ドキュメントによると0がCIRCLEに対応しているので、ここでは0でよい
    marker.type = 0

    # 座標を指定。四角形の中心を中心とする。二次元画像に出すのでzは0でよい
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    # 配信を行う
    pub.publish(marker)

# 配信ノード名をclientで作成する
rospy.init_node('client')

# 画像認識の結果を読む。RotatedRectStampedのmessageが/camshift/track_boxという名前のnodeから来たらcbを実行。
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

# 円を描画するためのmarkerを送るpublisherを作成。配信自体は上記のcallback内から行われる。
pub = rospy.Publisher('/image_marker', ImageMarker2)

# 無限ループで待つ(ログを表示)
rospy.spin()

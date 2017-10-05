#!/usr/bin/env python
# coding: utf-8

# 必要なモジュールのimport
# ROSの中枢となるモジュール
import rospy
# ROSのメッセージ型をpython上でのクラスとしてimport
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

# callback関数の定義
def cb(msg):
    print msg.rect
    # ImageMarkerクラスのインスタンス生成
    marker = ImageMarker2()
    # 生成したインスタンスのメンバ変数を書き込む
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # インスタンスデータを発行
    pub.publish(marker)

# ノードの初期化
rospy.init_node('client')

# 購読者インスタンスの生成
# /camshift/track_boxトピックから
# RotatedRectStamped型のメッセージが発行された時それを受け取り
# 関数cbをコールバックする
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

# 発行者インスタンスの生成
# メソッドpublishによって/image_marker2トピックに
# ImageMarker2型のメッセージを発行する
pub = rospy.Publisher('/image_marker', ImageMarker2, queue_size=1)

# 無限ループ
rospy.spin()

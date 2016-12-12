#!/usr/bin/env python

#Nodeを書く際はrospyをインポートする
import rospy
#画像処理ライブラリ
from opencv_apps.msg import RotatedRectStamped
#描画ライブラリ
from image_view2.msg import  ImageMarker2
#点や線や姿勢など幾何学的な特徴を扱うmsg
from geometry_msgs.msg import Point

#msg：メッセージのソースコードを異なる言語で作成する
def cb(msg):
    # RotatedRectStamped型の変数の内容をを表示
    print msg.rect
    # ImageMarker2型変数作成
    marker = ImageMarker2()
    # 描画タイプ
    marker.type = 0
    # 重心の座標をマーカーに代入
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    # markerを配信
    pub.publish(marker)

# rospyにノード名通知。ノード'client'の作成、初期化
rospy.init_node('client')
# ノードがRotatedRectStamped型のトピック'/camshift/track_box'から読み込むことを宣言。
# 新しいメッセージがトピックに到達するたびにコールバック関数cbを実行
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
# /image_markerというトピックにImageMarker2型のトピックを送信
pub = rospy.Publisher('/image_marker', ImageMarker2)
# 自ノード終了させないため
rospy.spin()

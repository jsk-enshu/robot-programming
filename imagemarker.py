# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy #rosのpython実装のインストール
from opencv_apps.msg import RotatedRectStamped #opencvでの回転長方形を表現するクラス
from image_view2.msg import ImageMarker2 #図の描画に使う（二次元？）　jskレポジトリ内にある
from geometry_msgs.msg import Point #座標を取り出すのに使う

def cb(msg): 
    print msg.rect #座標情報を表示 
    marker = ImageMarker2() #マーカーの定義
    marker.type = 0 #マーカーの種類　=0はcircle?
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #マーカーの座標
    pub.publish(marker) #マーカーを出版

rospy.init_node('client') #初期化　'client'という名前でroscoreに接続
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #購読側の関数　（名前、型、実行関数）　更新されるたびにcbが呼ばれる
pub = rospy.Publisher('/image_marker', ImageMarker2) #出版側の関数　（名前、型）
rospy.spin() #ノードの一連の処理が終わるまで終了しない

#!/usr/bin/env python

import rospy #rospyというライブラリをインポートしている
from opencv_apps.msg import RotatedRectStamped　#opencv_appsというmsgファイル中のRotatedRectStampedというライブラリをインポートしている
from image_view2.msg import ImageMarker2　#同上
from geometry_msgs.msg import Point　#同上

def cb(msg): #msgというクラスを引数に取るcbという関数を定義
print msg.rect　#msgの中のrectという変数を表示
    marker = ImageMarker2()　#markerというクラスに上でインポートしたImageMarker2という関数の返り値を代入
    marker.type = 0 #marker内のtypeという変数に0を代入(おそらく初期化を行っていると考えられる)
    marker.position = Point(msg.rect.center.x,msg.rect.center.y,0)　#markerのpositionという変数に上でインポートしたPointという関数の戻り値を代入している。また、msg.rect自体もクラスであり、クラスの中に三重にクラスが入れ子になっていることもわかる
    pub.publish(marker)　#上の三行で値を代入したmarkerというクラスを、このコードの場合は後述されている'/image_marker'というトピックに配信する

rospy.init_node('client')　#'client'というトピックをイニシャライズする
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb) #画像処理をした結果の'/camshift/track_box'を受け取る
pub = rospy.Publisher('/image_marker',ImageMarker2) #'/image_marker'というトピックにImageMarker2という形でメッセージを送ることを宣言
rospy.spin()#描画している

import rospy
/#pythonでROSを記述するときのモジュール
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point
#自分で定義したメッセージファイルから生成されたモジュール

def cb(msg):
#関数の定義
    print msg.rect
#メッセージの表示
    marker=ImageMarker2()
#ImageMarker2クラスのインスタンスを作成
    marker.type=0
#markerの種類を円に指定
    marker.position=Point(msg.rect.center.x,msg.rect.center.y,0)
#markerの位置の明記
    pub.publish(marker)
#送出する関数

rospy.init_node('client')
#clientという名前のソフトウェアの初期化宣言
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
#/camshift/track_boxを受け取る
pub=rospy.Publisher('/image_marker',ImageMarker2)
#/image_markerを送出
rospy.spin()
#トピック更新の待受

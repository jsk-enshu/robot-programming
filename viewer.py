import rospy #pythonでROSのソフトウェアを記述するためのモジュールのインポート
#以下は必要なメッセージ型のモジュールのインポート
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2    
from geometry_msgs.msg import Point

#コールバック関数の定義
def cb(msg):
    print msg.rect　#メッセージの表示
    marker = ImageMarker2() #markerのメッセージの型をImageMarker2に設定
    marker.type = 0 #markerの種類の設定（0は円に設定されている）
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #markerの位置の設定（rectは長方形の意味）
    pub.publish(marker) #markerの送信

rospy.init_node('client') #ノードの名前をclientに設定
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #/camshift/track_boxというトピックを介してメッセージを受信し、コールバック関数を実行
pub = rospy.Publisher('/image_marker', ImageMarker2) #/image_markerというトピックを介してメッセージを送信
rospy.spin() #コールバック関数を呼び続けてプログラムをループさせる

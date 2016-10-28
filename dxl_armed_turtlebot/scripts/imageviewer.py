#ライブラリのインストール
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#マーカーを作る関数
def cb(msg):
    print msg.rect
    marker = ImageMarker2()
    marker.type = 0 #マーカータイプを円に設定
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #円の中心を指定
    pub.publish(marker)

rospy.init_node('client') #ノードを設定
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb) #/cmshift/track_boxを受け取り、cbへ渡す
pub = rospy.Publisher('/image_marker', ImageMarker2) #/image_markerを送出する
rospy.spin() #繰り返し

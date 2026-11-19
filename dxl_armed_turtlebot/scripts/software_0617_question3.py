#必要なものを読み込む
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect          #xyz軸方向の長さと角度を変数に持つ楕円。
    marker=ImageMarker2()   #変数markerにImageMarker2の内容を代入する。
    marker.type=0           #typeをcircleに設定。（円を表示するようにする。）
    marker.position=Point(msg.rect.center.x,msg.rect.center.y,0)    #楕円の中心をマークする。
    pub.publish(marker)     #markerの内容を発信し、他の機器から受け取れるようにする。

rospy.init_node('client')
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)   #/camshift/track_boxを受け取った時にRotatedRectStampedという型のコールバック関数を呼ぶ。
pub=rospy.Publisher('/image_marker',ImageMarker2)   #/image_markerを送出する。
rospy.spin()    #シャットダウンするまでコールバック関数を呼び続ける。

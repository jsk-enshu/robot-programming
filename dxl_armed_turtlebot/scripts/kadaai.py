import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect #矩形領域の座標表示
    marker = ImageMarker2()
    marker.type = 0　#マーカーのセット(imagemarker2.msgによるとCIRCLE = 0)
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) #マーカーをうつ場所(2次元)
    pub.publish(marker) #マーカー情報の送出

rospy.init_node('client')　#clientというノード名を通知
rospy.Subscriber('/camshift/track_box',RotatedRectStamped,cb)
 #RotatedRectStamped型のtrace_boxというトピックから購読して、新しいメッセージについてcbを呼び出す
pub = rospy.Publisher('/image_marker',ImageMarker2)#image_markerというトピック、ImageMarker2というメッセージ型
rospy.spin() #終了しないようにアイドリング

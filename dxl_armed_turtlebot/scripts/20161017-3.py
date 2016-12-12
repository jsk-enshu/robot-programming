### 必要な各モジュールをインポート
import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point


### コールバック関数
def cb(msg):

### RotatedRectStamped型のメッセージmsgのrectの情報、すなわち
### 長方形領域の角度angleと中心座標(x, y)、大きさ(width, height)を表示
    print msg.rect

### 出版するためのImageMarker2型の変数markerを設定
    marker = ImageMarker2()

### 円の場合、0を指定
    marker.type = 0

### 長方形領域の中心座標(x, y, z)をmarker.positionに代入
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

### /image_markerにmarkerの情報を配信
    pub.publish(marker)


### rospyにclientというノード名を通知
rospy.init_node('client')

### /camshift/track_boxトピックからRotatedRectStamped型メッセージを購読
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

### /image_markerトピックにImagemarker2型メッセージを配信
pub = rospy.Publisher('/image_marker', ImageMarker2)

### ノードが終了するまでの間は単純に自ノード終了しないようにする
rospy.spin()


### 参考HP

### http://docs.ros.org/indigo/api/jsk_recognition_msgs/html/msg/RotatedRectStamped.html
### http://docs.ros.org/fuerte/api/image_view2/html/msg/ImageMarker2.html
### http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html
### http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
### http://wiki.ros.org/ja/rospy_tutorials/Tutorials/WritingPublisherSubscriber

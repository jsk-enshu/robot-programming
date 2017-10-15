#!/user/bin/env pyhton
#pythonのインタプリタを使って実行することを明示

#各ライブラリをimport
import rospy
from  opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#ImageMarker2のメッセージの配信とmarkerを付けたオブジェクトの位置とサイズを表示
def cb(msg):
    print msg.rect #メッセージをプリント
    marker = ImageMarker2() #インスタンスを生成
    marker.type = 0 #markerの種類を円に
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    #marker.positionにPointのインスタンスを代入
    pub.publish(marker)#rospy.Publisher.publish()を呼び出し

rospy.init_node('client')
#rosのノードを初期化。ノードをclientと命名
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
#/camshift/track_boxからメッセージを受け取る。メッセージのデータタイプはRotatedRectStampedである。データを受け取った時にcb
pub = rospy.Publisher('/image_marker', ImageMarker2)
#トピック名は/image_marker。データのクラスはImageMarker2
rospy.spin()
#ノードが停止するまでpythonが終了しないようにする

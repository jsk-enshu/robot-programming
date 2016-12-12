 import rospy
 from opencv_apps.msg import RotatedRectStamped
 from image_view2.msg import ImageMarker2
 from geometry_msgs.msg import Point
 

def cb(msg):

    
    print msg.rect

    # markerのセット
    marker = ImageMarker2()
    marker.type = 0

    # marker.positionに受け取ったmsgを格納
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    # markerを使って配信
    pub.publish(marker)
    
# 'client'という名前のノードを立ち上げる
rospy.init_node('client')

# '/camshift/track_box'からRotatedRectStamped型のメッセージを購読し、cbコールバック関数を呼ぶ
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

# '/image_marker'にImageMarker2型のメッセージを送信
pub = rospy.Publisher('/image_marker', ImageMarker2, queue_size = 10)

# ノードが終了するまで続ける
rospy.spin()

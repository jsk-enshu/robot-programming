#! /usr/bin/env python
import rospy    #python client library for ROS
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2    #image_view2 allows for drawing markers on image
# ImageMarker2 is a message to draw on the image_view2 canvas
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect  #print data received
    marker=ImageMarker2()   #create instance of ImageMarker2
    marker.type=0
    marker.position=Point(msg.rect.center.x, msg.rect.center.y, 0)  #define position
    pub.publish(marker)

rospy.init_node("client")
rospy.Subscriber("/camshift/track_box", RotatedRectStamped, cb)
pub=rospy.Publisher("/image_marker", ImageMarker2)  #handle to publish messages to a topic
rospy.spin()    #keeps the node existing, until node is shutdown

#!/usr/bin/env python
import rospy
from opencv_apps.msg import RotatedRectStamped #This is to use "RotatedRect".
from image_view2.msg import ImageMarker2 #This is to designate the range of interest(ROI) using a marker.
from geometry_msgs.msg import Point #This is to designate the position of a point in a three-dimensional space.
def cb(msg):
    print msg.rect #print received data
    marker=ImageMarker2() 
    marker.type=0 #initialize
    marker.position=Point(msg.rect.center.x,msg.rect.center.y,0) #define the position as the center of the rectangle.
    pub.publish(marker)#send messages.

    rospy.init_node('client') #create a new node and name it "client"
    rospy.Subscirber('/camshift/track_box',RotatedRectStamped,cb)
    pub=roapy.Publisher('/image_marker',ImageMarker2) #define subscriber and publisher
    rospy.spin() #continue this until the process is cancelled

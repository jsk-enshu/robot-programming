#!/usr/bin/env python

# import rospy module (ROS client library)
import rospy

# import opencv_apps.msg.RotatedRectStamped class
from opencv_apps.msg import RotatedRectStamped

# import image_view2.msg.ImageMarker2 class
from image_view2.msg import ImageMarker2

# import geometry_msgs.msg.Point class
from geometry_msgs.msg import Point


def cb(msg):
    '''
    Callback function called when received message from topic '/camshift/track_box'

    parameters:
        msg: ROS message
    returns:
        None
    '''

    # print information of marker
    print msg.rect

    # instantiate ImageMarker2 object
    marker = ImageMarker2()

    # set markers value
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)

    # publish marker message to ROS node
    pub.publish(marker)


# register client node with name 'client'
rospy.init_node('client')

# register as subscriber to topic '/camshift/track_box' with data class of message and callback function
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)

# register as a publisher of ROS topic with data class of message
pub = rospy.Publisher('/image_marker', ImageMarker2)

# blocks until ROS node is shutdown
rospy.spin()

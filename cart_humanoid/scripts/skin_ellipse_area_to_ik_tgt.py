#!/usr/bin/env python
# coding=utf-8
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from opencv_apps.msg import *

class skin_ellipse_area_to_ik_tgt:

    def __init__(self):
        rospy.init_node("skin_ellipse_area_to_ik_tgt")
        rospy.Subscriber("/head_arm/general_contours/ellipses", RotatedRectArrayStamped, self.cb)
        self.head_pub = rospy.Publisher("/ik_head_tgt", PoseStamped, queue_size=1)
        self.larm_pub = rospy.Publisher("/ik_larm_tgt", PoseStamped, queue_size=1)
        self.rarm_pub = rospy.Publisher("/ik_rarm_tgt", PoseStamped, queue_size=1)
        # self.lleg_pub = rospy.Publisher("/ik_lleg_tgt", PoseStamped, queue_size=1)
        # self.rleg_pub = rospy.Publisher("/ik_rleg_tgt", PoseStamped, queue_size=1)
        rospy.spin()

    def cb(self, msg):
        # Structure of RotatedRectArrayStamped (rostopic echo)
        # rects: 
        #   - 
        #     angle: 0.0
        #     center: 
        #       x: 0.0
        #       y: 0.0
        #     size: 
        #       width: 0.0
        #       height: 0.0
        #   - 
        #     angle: 114.636924744
        #     center: 
        #       x: 376.963684082
        #       y: 404.353729248
        #     size: 
        #       width: 3.78912639618
        #       height: 4.34580373764

        unsorted_list = [ (rs.size.width * rs.size.height, rs.center.x, rs.center.y) for rs in msg.rects]
        top3_area_list = sorted([(area, x, y) for (area, x, y) in unsorted_list], reverse=True)[:3]
        pos_sorted_list = sorted([(x, y, area) for (area, x, y) in top3_area_list])
        # # sorted by X pos in image coords, and simple labeling
        # # left-most  ellipse in the image = right hand
        # # middle     ellipse in the image = face
        # # right-most ellipse in the image = left hand
        
        valid_area_min = 10000
        if len([area for (x, y, area) in pos_sorted_list if area > valid_area_min]) < 3:
            rospy.logwarn_throttle(1, "valid skin area < 3, keep waiting...")
            return
        else:
            rarm_pos = pos_sorted_list[0]
            head_pos = pos_sorted_list[1]
            larm_pos = pos_sorted_list[2]
        
        rospy.loginfo("larm: " + str(larm_pos) + " head: " + str(head_pos) + " rarm: " + str(rarm_pos))

        # 2D image coords [0~640, 0~480] -> 3D robot coords [CONST_DEPTH, -0.5~0.5, 0~1]
        IMAGE_W = 640.0 # [pixel]
        IMAGE_H = 480.0 # [pixel]
        CONST_DEPTH = 0.5 # for 2D -> 3D []
        val = PoseStamped()
        
        val.header.frame_id = "BODY"
        val.pose.position.x = CONST_DEPTH
        val.pose.position.y =  (head_pos[0] - (IMAGE_W/2)) / (IMAGE_W/2) * 0.5
        val.pose.position.z = -(head_pos[1] - (IMAGE_H/2)) / (IMAGE_H/2) * 0.5 + 0.5
        self.head_pub.publish(val)

        val.header.frame_id = "BODY"
        val.pose.position.x = CONST_DEPTH
        val.pose.position.y =  (larm_pos[0] - (IMAGE_W/2)) / (IMAGE_W/2) * 0.5
        val.pose.position.z = -(larm_pos[1] - (IMAGE_H/2)) / (IMAGE_H/2) * 0.5 + 0.5
        self.larm_pub.publish(val)

        val.header.frame_id = "BODY"
        val.pose.position.x = CONST_DEPTH
        val.pose.position.y =  (rarm_pos[0] - (IMAGE_W/2)) / (IMAGE_W/2) * 0.5
        val.pose.position.z = -(rarm_pos[1] - (IMAGE_H/2)) / (IMAGE_H/2) * 0.5 + 0.5
        self.rarm_pub.publish(val)

        
if __name__ == '__main__':
    instance = skin_ellipse_area_to_ik_tgt()

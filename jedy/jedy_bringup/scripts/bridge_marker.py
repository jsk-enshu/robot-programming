#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import colorsys

g_marker_array_pub = None

def generate_color_by_label(label):
    hue = (label * 0.6180339887) % 1.0
    saturation = 1.0
    value = 1.0
    
    r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
    return ColorRGBA(r=r, g=g, b=b, a=0.5)

def bbox_array_callback(bbox_array_msg):
    global g_marker_array_pub
    if g_marker_array_pub is None:
        return

    marker_array = MarkerArray()
    
    for i, bbox in enumerate(bbox_array_msg.boxes):
        marker = Marker()
        
        marker.header = bbox_array_msg.header
        
        marker.ns = "jsk_bounding_boxes"
        marker.id = i  # 配列内のインデックスをIDとして使用
        
        # 形状はCUBE (立方体)
        marker.type = Marker.CUBE
        
        # アクションは追加/修正
        marker.action = Marker.ADD
        
        # 座標と向き (BoundingBoxからそのままコピー)
        marker.pose = bbox.pose
        
        # 大きさ (BoundingBoxのdimensionsをscaleにマッピング)
        marker.scale = bbox.dimensions
        
        # 色 (ラベルに基づいて生成)
        marker.color = generate_color_by_label(bbox.label)
        
        # マーカーの寿命 (2.0秒。新しいメッセージが来ないと自動的に消える)
        # これにより、検出が消えたときにRvizからも消えます。
        marker.lifetime = rospy.Duration(2.0)
        
        marker_array.markers.append(marker)

    g_marker_array_pub.publish(marker_array)

def main():
    global g_marker_array_pub

    rospy.init_node('bbox_to_marker_converter_node')

    input_topic = rospy.get_param('~input_topic', '/HSI_color_filter/boxes')
    output_topic = rospy.get_param('~output_topic', 'output_marker_array')

    rospy.Subscriber(input_topic, BoundingBoxArray, bbox_array_callback, queue_size=1)
    g_marker_array_pub = rospy.Publisher(output_topic, MarkerArray, queue_size=1)

    rospy.loginfo("BoundingBoxArray -> MarkerArray 変換ノードを開始しました。")
    rospy.loginfo("  入力トピック: %s", input_topic)
    rospy.loginfo("  出力トピック: %s", output_topic)
    rospy.spin()

if __name__ == '__main__':
    main()

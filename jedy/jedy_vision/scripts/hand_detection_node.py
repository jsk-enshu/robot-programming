#!/usr/bin/env python3
"""Find hands in the camera image (MediaPipe Hand Landmarker).

Reads every new image, ``rate`` times a second at most, and publishes the
hands on ``~/hands`` (jedy_vision_msgs/DetectionArray): one Detection a hand,
labelled ``右手`` or ``左手`` (the person's own side, for a camera facing
them), with the box around its 21 points [px]. ``~/image`` is the image with
the hands drawn, for RViz or ``rqt_image_view``.

Parameters
----------
image_topic : str
    The colour image (default: the head camera, as ``ri.image()``).
rate : float
    Images read a second at most (about 20 ms each on a laptop CPU).
max_hands : int
    Hands looked for in one image.
min_score : float
    Hands found with less confidence are dropped.
"""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from jedy_vision import HAND_BONES
from jedy_vision import HandEstimator
from jedy_vision.recognition import decode_rgb
from jedy_vision.recognition import encode_rgb
from jedy_vision_msgs.msg import Detection
from jedy_vision_msgs.msg import DetectionArray

IMAGE_TOPIC = '/camera/color/image_rect_raw'
SIDE_JA = {'Right': '右手', 'Left': '左手'}
BONE_COLOR = (255, 80, 80)
BOX_COLOR = (0, 255, 0)
LINE_THICKNESS = 2


class HandDetectionNode(Node):
    """The node; see the module docstring."""

    def __init__(self):
        super(HandDetectionNode, self).__init__('hand_detection')
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_hands', 2)
        self.declare_parameter('min_score', 0.5)
        p = self.get_parameter
        self.estimator = HandEstimator(max_hands=p('max_hands').value,
                                       min_score=p('min_score').value)
        self._lock = threading.Lock()
        self._image = None
        self._done = None            # the last image read
        topic = p('image_topic').value
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.hands_pub = self.create_publisher(DetectionArray, '~/hands', 10)
        self.image_pub = self.create_publisher(Image, '~/image', 1)
        self.create_timer(1.0 / p('rate').value, self._on_timer)
        self.get_logger().info('ready: looking for hands in {}'.format(topic))

    def _on_image(self, msg):
        with self._lock:
            self._image = msg

    def _on_timer(self):
        with self._lock:
            msg = self._image
        if msg is None or msg is self._done:
            return
        self._done = msg
        rgb = decode_rgb(msg)
        result = DetectionArray(header=msg.header)
        drawn = rgb.copy()
        for hand in self.estimator.estimate(rgb):
            x1, y1 = hand.landmarks.min(axis=0)
            x2, y2 = hand.landmarks.max(axis=0)
            result.detections.append(Detection(
                label=SIDE_JA.get(hand.side, hand.side), score=hand.score,
                box=[float(x1), float(y1), float(x2), float(y2)]))
            points = np.round(hand.landmarks).astype(int)
            for a, b in HAND_BONES:
                cv2.line(drawn, tuple(points[a]), tuple(points[b]), BONE_COLOR, LINE_THICKNESS)
            cv2.rectangle(drawn, (int(x1), int(y1)), (int(x2), int(y2)), BOX_COLOR, LINE_THICKNESS)
        self.hands_pub.publish(result)
        self.image_pub.publish(encode_rgb(drawn, msg.header))

    def destroy_node(self):
        self.estimator.close()
        super(HandDetectionNode, self).destroy_node()


def main():
    rclpy.init()
    node = HandDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Find objects named by words in the head camera's image.

Service ``/object_detection/what_do_you_see`` (jedy_vision_msgs/WhatDoYouSee):
the words to look for in, the objects found in the latest image out. This is
what ``ri.what_do_you_see()`` and ``ri.sees()`` call.

The detector runs only when asked (about 0.15 s an image on a laptop CPU), not
on every frame, so the node costs almost nothing while nobody asks. With
``stream:=true`` it also runs ``rate`` times a second on the current words and
publishes ``~/detections``. Either way every run publishes ``~/image``, the
image with the boxes drawn, for RViz or ``rqt_image_view``.

Each detection also says where the object is (``position``, in the camera's
optical frame), from the depth image: the box centre at the median depth of
the box's middle third. Every run publishes ``~/markers``
(visualization_msgs/MarkerArray): a sphere and the name at each object, for
RViz. Without a depth image (or one of another size than the colour image)
the detections still come, with ``has_position`` false.

Parameters
----------
image_topic : str
    The colour image (default: the head camera, as ``ri.image()``).
depth_topic : str
    The depth image registered to it, ``32FC1`` [m] or ``16UC1`` [mm].
camera_info_topic : str
    The colour image's camera_info.
words : list of str
    The words to look for until a request says otherwise.
min_score : float
    Detections below this confidence are dropped.
max_age : float
    A request fails when the latest image is older than this [s] (wall
    clock): an answer about an old image would look like a current one.
stream : bool
    Also detect continuously and publish ``~/detections``.
rate : float
    Detections a second with ``stream``.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np

from jedy_vision.recognition import box_position
from jedy_vision.recognition import decode_depth
from jedy_vision.recognition import decode_rgb
from jedy_vision.recognition import draw_detections
from jedy_vision.recognition import encode_rgb
from jedy_vision.recognition import is_ascii
from jedy_vision.recognition import WordDetector
from jedy_vision_msgs.msg import Detection
from jedy_vision_msgs.msg import DetectionArray
from jedy_vision_msgs.srv import WhatDoYouSee

IMAGE_TOPIC = '/camera/color/image_rect_raw'
DEPTH_TOPIC = '/camera/depth/image_rect_raw'
CAMERA_INFO_TOPIC = '/camera/color/camera_info'
#: How long RViz keeps a marker after the run that drew it [s].
MARKER_LIFETIME = 2.0


class ObjectDetectionNode(Node):
    """The node; see the module docstring."""

    def __init__(self):
        super(ObjectDetectionNode, self).__init__('object_detection')
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('depth_topic', DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', CAMERA_INFO_TOPIC)
        self.declare_parameter('words', ['人', 'ペットボトル', 'コップ', '椅子'])
        self.declare_parameter('min_score', 0.25)
        self.declare_parameter('max_age', 2.0)
        self.declare_parameter('stream', False)
        self.declare_parameter('rate', 5.0)
        p = self.get_parameter
        self.max_age = p('max_age').value
        self.get_logger().info('loading YOLOE ...')
        self.detector = WordDetector(min_score=p('min_score').value)
        self.detector.set_words(list(p('words').value))
        # One detection at a time: the service and the stream timer would
        # otherwise share the model from two threads.
        self._model_lock = threading.Lock()
        self._lock = threading.Lock()
        self._image = None
        self._received = 0.0
        self._depth = None
        self._K = None
        topic = p('image_topic').value
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(Image, p('depth_topic').value, self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('camera_info_topic').value,
                                 self._on_info, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, '~/image', 1)
        self.detections_pub = self.create_publisher(DetectionArray, '~/detections', 1)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.create_service(WhatDoYouSee, '~/what_do_you_see', self._on_request)
        if p('stream').value:
            self.create_timer(1.0 / p('rate').value, self._on_timer)
        self.get_logger().info('ready: looking for {} in {}'.format(
            self.detector.words, topic))

    def _on_image(self, msg):
        with self._lock:
            self._image = msg
            self._received = time.monotonic()

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def _on_info(self, msg):
        with self._lock:
            self._K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _positions(self, msg, found):
        """``(x, y, z)`` or None for each detection, from the latest depth.

        None for all when there is no depth image or camera_info yet, or the
        depth image is not the colour image's size (then its pixels are not
        the colour image's).
        """
        with self._lock:
            depth_msg, K = self._depth, self._K
        if depth_msg is None or K is None:
            return [None] * len(found)
        if (depth_msg.height, depth_msg.width) != (msg.height, msg.width):
            self.get_logger().warn(
                'depth image {}x{} is not the colour image {}x{}: no positions'.format(
                    depth_msg.width, depth_msg.height, msg.width, msg.height),
                throttle_duration_sec=10.0)
            return [None] * len(found)
        depth = decode_depth(depth_msg)
        return [box_position(depth, box, K) for _, _, box in found]

    def _markers(self, header, found, positions):
        """A sphere and the name at each positioned detection, for RViz."""
        markers = MarkerArray()
        clear = Marker(header=header, action=Marker.DELETEALL)
        markers.markers.append(clear)
        vocabulary = self.detector.vocabulary
        for i, ((word, score, _), pos) in enumerate(zip(found, positions)):
            if pos is None:
                continue
            name = word if is_ascii(word) else vocabulary.get(word, word)
            for kind, marker_type in (('sphere', Marker.SPHERE),
                                      ('label', Marker.TEXT_VIEW_FACING)):
                m = Marker(header=header, ns='detections_' + kind, id=i,
                           type=marker_type, action=Marker.ADD)
                m.pose.position = Point(x=pos[0], y=pos[1] - (0.08 if kind == 'label' else 0.0),
                                        z=pos[2])
                m.pose.orientation.w = 1.0
                if kind == 'sphere':
                    m.scale.x = m.scale.y = m.scale.z = 0.06
                    m.color.g, m.color.a = 1.0, 0.8
                else:
                    m.scale.z = 0.05
                    m.text = '{} {:.2f} ({:.2f} m)'.format(name, score, pos[2])
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                m.lifetime.sec = int(MARKER_LIFETIME)
                markers.markers.append(m)
        return markers

    def _latest(self):
        """The latest image message, or None with the reason."""
        with self._lock:
            msg, received = self._image, self._received
        if msg is None:
            return None, 'no image on {} yet (publishers: {})'.format(
                self.get_parameter('image_topic').value,
                self.count_publishers(self.get_parameter('image_topic').value))
        age = time.monotonic() - received
        if age > self.max_age:
            return None, 'the latest image is {:.1f} s old (max_age {} s): is the camera running?'.format(
                age, self.max_age)
        return msg, ''

    def _detect(self, msg, words=None):
        """Detect in ``msg`` and publish the drawn image; the result message."""
        rgb = decode_rgb(msg)
        with self._model_lock:
            if words:
                self.detector.set_words(words)
            found = self.detector.detect(rgb)
        positions = self._positions(msg, found)
        self.image_pub.publish(encode_rgb(
            draw_detections(rgb, found, self.detector.vocabulary, positions), msg.header))
        self.markers_pub.publish(self._markers(msg.header, found, positions))
        result = DetectionArray(header=msg.header)
        for (word, score, box), pos in zip(found, positions):
            d = Detection(label=word, score=score, box=list(box))
            if pos is not None:
                d.position = Point(x=pos[0], y=pos[1], z=pos[2])
                d.has_position = True
            result.detections.append(d)
        return result

    def _on_request(self, request, response):
        msg, why = self._latest()
        if msg is None:
            response.success, response.message = False, why
            return response
        try:
            response.result = self._detect(msg, list(request.words))
        except ValueError as e:
            response.success, response.message = False, str(e)
            return response
        response.success = True
        response.message = 'looked for {}'.format(self.detector.words)
        return response

    def _on_timer(self):
        msg, why = self._latest()
        if msg is None:
            self.get_logger().warn(why, throttle_duration_sec=5.0)
            return
        self.detections_pub.publish(self._detect(msg))


def main():
    rclpy.init()
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

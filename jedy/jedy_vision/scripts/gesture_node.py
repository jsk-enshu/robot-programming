#!/usr/bin/env python3
"""Read hand gestures (グー・チョキ・パー, いいね, ...) in the camera image.

Reads every new image, ``rate`` times a second at most, and publishes the
gesture on ``~/gesture`` (jedy_vision_msgs/Gesture). A gesture counts only
once it has been the same in ``hold`` images in a row: a hand caught moving
from one shape to the next is often read as a third one for an image or two.
Service ``/gesture/read_gesture`` answers with the current one; that is what
``ri.gesture()`` and ``ri.janken()`` call.

Parameters
----------
image_topic : str
    The colour image (default: the head camera, as ``ri.image()``).
rate : float
    Images read a second at most (about 25 ms each on a laptop CPU).
hold : int
    Images in a row a gesture must be seen in.
min_score : float
    Readings below this confidence count as no gesture.
max_age : float
    The service fails when the latest image is older than this [s].
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from jedy_vision.recognition import decode_rgb
from jedy_vision.recognition import GESTURE_JA
from jedy_vision.recognition import GestureReader
from jedy_vision.recognition import JANKEN
from jedy_vision_msgs.msg import Gesture
from jedy_vision_msgs.srv import ReadGesture

IMAGE_TOPIC = '/camera/color/image_rect_raw'


class GestureNode(Node):
    """The node; see the module docstring."""

    def __init__(self):
        super(GestureNode, self).__init__('gesture')
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('hold', 3)
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('max_age', 2.0)
        p = self.get_parameter
        self.hold = p('hold').value
        self.min_score = p('min_score').value
        self.max_age = p('max_age').value
        self.reader = GestureReader()
        self._lock = threading.Lock()
        self._image = None
        self._received = 0.0
        self._done = None            # the last image read
        self._run_name = None        # the reading of the last images ...
        self._run_length = 0         # ... and in how many in a row
        self._current = Gesture()    # what is published / answered
        topic = p('image_topic').value
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(Gesture, '~/gesture', 10)
        self.create_service(ReadGesture, '~/read_gesture', self._on_request)
        self.create_timer(1.0 / p('rate').value, self._on_timer)
        self.get_logger().info('ready: reading gestures in {}'.format(topic))

    def _on_image(self, msg):
        with self._lock:
            self._image = msg
            self._received = time.monotonic()

    def _on_timer(self):
        with self._lock:
            msg = self._image
        if msg is None or msg is self._done:
            return
        self._done = msg
        reading = self.reader.read(decode_rgb(msg))
        if reading is not None and reading[1] < self.min_score:
            reading = None
        name = None if reading is None else reading[0]
        if name == self._run_name:
            self._run_length += 1
        else:
            self._run_name, self._run_length = name, 1
        g = Gesture(header=msg.header)
        if name is not None and self._run_length >= self.hold:
            g.name, g.score, g.handedness = reading
            g.label = GESTURE_JA[name]
            g.janken = g.label if g.label in JANKEN else ''
        with self._lock:
            self._current = g
        self.pub.publish(g)

    def _on_request(self, request, response):
        with self._lock:
            received, current = self._received, self._current
        if self._image is None:
            response.success = False
            response.message = 'no image on {} yet'.format(
                self.get_parameter('image_topic').value)
            return response
        age = time.monotonic() - received
        if age > self.max_age:
            response.success = False
            response.message = 'the latest image is {:.1f} s old (max_age {} s): is the camera running?'.format(
                age, self.max_age)
            return response
        response.success = True
        response.gesture = current
        return response

    def destroy_node(self):
        self.reader.close()
        super(GestureNode, self).destroy_node()


def main():
    rclpy.init()
    node = GestureNode()
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

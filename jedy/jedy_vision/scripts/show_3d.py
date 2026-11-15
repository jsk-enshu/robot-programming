#!/usr/bin/env python3
"""Show the AprilTags and objects the head camera sees, in 3D, in RViz.

    ros2 launch jedy_bringup jedy_gazebo.launch.py tags:=true box:=true pointcloud:=true
    ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true pointcloud:=true   # real robot
    ros2 launch jedy_vision recognition.launch.py                          # only for --words
    ros2 run jedy_vision show_3d --tag-size 0.064 --words "red object"
    rviz2 -d $(ros2 pkg prefix jedy_vision)/share/jedy_vision/rviz/show_3d.rviz

Every ``--period`` seconds (plus about 0.7 s when looking for objects) it
reads one colour image and publishes:

- ``/show_3d/markers`` (visualization_msgs/MarkerArray): each AprilTag as a
  thin plate of its size with an arrow out of its face and its id (base
  frame, from the colour image only), and each object found for ``--words``
  as a translucent box with its name, score and distance (optical frame).
- ``/show_3d/image`` (sensor_msgs/Image): the colour image with the tags'
  outlines and ids and the objects' boxes.

The point cloud in show_3d.rviz is not made here: it is the camera's own,
``/camera/depth/color/points``, published with ``pointcloud:=true`` by the
RealSense driver on the real robot and by depth_image_proc in Gazebo.

The object's box: the detector gives a box in the image and the detection
node the distance ``z`` to the object's front; the box is ``(x2 - x1) * z /
fx`` wide and ``(y2 - y1) * z / fy`` tall, as deep as the smaller of the two
(one image cannot tell the depth), and its centre is half that behind the
front.

Tags need their size: the black square's edge [m] (``--tag-size``). A tag of
another size is still found, but its distance comes out scaled by the
ratio of the sizes.
"""

import argparse

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from skrobot.coordinates.math import matrix2quaternion
from visualization_msgs.msg import Marker, MarkerArray

from jedy_skrobot import connect
from jedy_vision import TagDetector, set_head

#: The colour camera's optical frame in the robot's URDF (x right, y down, z forward).
CAMERA_FRAME = 'camera_color_optical_frame'


def marker(header, ns, mid, kind, rgba):
    m = Marker(header=header, ns=ns, id=mid, type=kind, action=Marker.ADD)
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.pose.orientation.w = 1.0
    return m


def tag_markers(tags, size, header):
    """A plate, an arrow out of the face and the id for each tag (base frame)."""
    out = []
    for i, tag in enumerate(tags):
        plate = marker(header, 'tag plate', i, Marker.CUBE, (1.0, 1.0, 1.0, 0.9))
        plate.pose.position = Point(x=float(tag.position[0]), y=float(tag.position[1]),
                                    z=float(tag.position[2]))
        q = rotation_to_quaternion(tag.rotation)
        plate.pose.orientation.x, plate.pose.orientation.y = q[0], q[1]
        plate.pose.orientation.z, plate.pose.orientation.w = q[2], q[3]
        plate.scale.x, plate.scale.y, plate.scale.z = size, size, 0.003
        arrow = marker(header, 'tag normal', i, Marker.ARROW, (0.1, 0.4, 1.0, 1.0))
        end = tag.position + 0.08 * tag.normal
        arrow.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in (tag.position, end)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.02
        text = marker(header, 'tag id', i, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position = Point(x=float(end[0]), y=float(end[1]), z=float(end[2]) + 0.03)
        text.scale.z = 0.04
        text.text = 'tag#{}'.format(tag.id)
        out += [plate, arrow, text]
    return out


def object_boxes(detections, K, header):
    """A translucent box and a label for each positioned detection (optical frame)."""
    out = []
    for i, d in enumerate(detections):
        if not d.has_position:
            continue
        x1, y1, x2, y2 = d.box
        z = d.position.z
        w = (x2 - x1) * z / K[0, 0]
        h = (y2 - y1) * z / K[1, 1]
        depth = min(w, h)
        box = marker(header, 'object box', i, Marker.CUBE, (0.1, 1.0, 0.2, 0.45))
        box.pose.position = Point(x=d.position.x, y=d.position.y, z=z + depth / 2.0)
        box.scale.x, box.scale.y, box.scale.z = w, h, depth
        text = marker(header, 'object label', i, Marker.TEXT_VIEW_FACING, (0.1, 1.0, 0.2, 1.0))
        text.pose.position = Point(x=d.position.x, y=d.position.y - h / 2.0 - 0.07, z=z)
        text.scale.z = 0.025
        # One word per line: RViz draws a space in a text marker as a wide
        # gap, which splits the label across the view.
        text.text = '\n'.join(d.label.split() + ['{:.2f}'.format(d.score), '{:.2f}m'.format(z)])
        out += [box, text]
    return out


def rotation_to_quaternion(R):
    """``(x, y, z, w)`` (ROS order) of a rotation matrix.

    scikit-robot's ``matrix2quaternion`` returns ``[w, x, y, z]``.
    """
    w, x, y, z = matrix2quaternion(R)
    return float(x), float(y), float(z), float(w)


def annotated(rgb, tags, detections):
    """The colour image with tag outlines and ids, and object boxes."""
    import cv2
    out = rgb.copy()
    for tag in tags:
        pts = tag.corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (255, 0, 255), 2)
        cx, cy = tag.corners.mean(axis=0).astype(int)
        cv2.putText(out, 'tag {} {:.2f} m'.format(tag.id, np.linalg.norm(tag.position[:2])),
                    (int(cx) - 40, int(tag.corners[:, 1].min()) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA)
    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d.box)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = '{} {:.2f}'.format(d.label if d.label.isascii() else '?', d.score)
        if d.has_position:
            label += ' {:.2f} m'.format(d.position.z)
        cv2.putText(out, label, (x1, max(y1 - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1, cv2.LINE_AA)
    return out


def image_msg(rgb, header):
    return Image(header=header, height=rgb.shape[0], width=rgb.shape[1], encoding='rgb8',
                 is_bigendian=0, step=rgb.shape[1] * 3, data=rgb.tobytes())


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--tag-size', type=float, default=0.064,
                   help="edge of the tags' black square [m]")
    p.add_argument('--words', nargs='*', default=[],
                   help='objects to look for (needs recognition.launch.py), e.g. "red object"')
    p.add_argument('--tilt', type=float, default=20.0, help='head tilt, down [deg]')
    p.add_argument('--period', type=float, default=1.0, help='[s] between updates')
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on(joints=['head_joint0', 'head_joint1'])   # the real robot starts with servos off
    set_head(robot, ri, 0.0, args.tilt)
    K, _ = ri.camera_info()
    detector = TagDetector(args.tag_size, upscale=2)
    markers_pub = ri.create_publisher(MarkerArray, '/show_3d/markers', 1)
    image_pub = ri.create_publisher(Image, '/show_3d/image', 1)
    warned = False
    while True:
        rgb, stamp = ri.image(newer_than=ri.now())
        robot.angle_vector(ri.angle_vector())
        tags = detector.detect(rgb, K, robot)
        detections = []
        if args.words:
            try:
                detections = ri.what_do_you_see(args.words)
            except Exception as e:  # recognition.launch.py not running, unknown word, ...
                if not warned:
                    print('objects skipped: {}'.format(e))
                    warned = True
        # The object boxes and the image carry the image's stamp, so RViz
        # places them with the head where it was when the image was taken
        # (as it does the point cloud), not where it is now.
        sec = int(stamp)
        cam = Header(frame_id=CAMERA_FRAME,
                     stamp=Time(sec=sec, nanosec=int(round((stamp - sec) * 1e9))))
        base = Header(frame_id='base_link', stamp=ri.get_clock().now().to_msg())
        markers = MarkerArray(markers=[Marker(header=base, action=Marker.DELETEALL)])
        markers.markers += tag_markers(tags, args.tag_size, base)
        markers.markers += object_boxes(detections, K, cam)
        markers_pub.publish(markers)
        image_pub.publish(image_msg(annotated(rgb, tags, detections), cam))
        print('tags: {}  objects: {}'.format(
            ', '.join('{} at {}'.format(t.id, np.round(t.position, 2)) for t in tags) or '-',
            ', '.join('{} {:.2f}'.format(d.label, d.score) for d in detections) or '-'), flush=True)
        ri.sleep(args.period)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('stopped by Ctrl-C')
        raise SystemExit(130)

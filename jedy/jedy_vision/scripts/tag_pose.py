#!/usr/bin/env python3
"""Where the AprilTags are, in the robot's base frame (and in RViz).

    ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true camera:=d405
    ros2 run jedy_vision tag_pose            # print + /demo/markers for RViz
    ros2 run jedy_vision tag_pose --once

Only the colour image is used: the four corners of the tag's black square and
its known size give the pose (cv2.solvePnP, IPPE). The demo stage's tags
differ in size (tag 0 on the stand 8 cm, 1-6 on the cube 3.6 cm), so the
size is looked up by id (DEMO_TAG_SIZES); with ``--size`` every tag is
taken to be that size. If any cube face is in view, the cube's centre is
printed too (cube_pose). That works at any range,
also where the D405 has no depth. The camera's own pose comes from the robot
model with the measured head angles (``robot.camera_color_optical_frame``).
"""

import argparse

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from jedy_skrobot import connect
from jedy_vision import DEMO_TAG_SIZES, TagDetector, cube_pose, set_head


def markers(tags, stamp):
    out = MarkerArray()
    for tag in tags:
        for k, (axis, rgb) in enumerate(((0, (1.0, 0.0, 0.0)), (1, (0.0, 1.0, 0.0)),
                                         (2, (0.0, 0.0, 1.0)))):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'base_link', stamp
            m.ns, m.id = 'tag{}'.format(tag.id), k
            m.type, m.action = Marker.ARROW, Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = 0.01, 0.02, 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
            end = tag.position + 0.1 * (tag.normal if axis == 2 else tag.rotation[:, axis])
            m.points = [Point(x=float(a), y=float(b), z=float(c))
                        for a, b, c in (tag.position, end)]
            m.lifetime.sec = 1
            out.markers.append(m)
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--size', type=float, default=None,
                   help='tag edge [m] for every tag (default: per id, the demo stage)')
    p.add_argument('--tilt', type=float, default=30.0, help='head tilt, down [deg]')
    p.add_argument('--once', action='store_true')
    args = p.parse_args()

    robot, ri = connect()
    K, _ = ri.camera_info()
    set_head(robot, ri, 0.0, args.tilt)
    pub = ri.create_publisher(MarkerArray, '/demo/markers', 10)
    detector = TagDetector(DEMO_TAG_SIZES if args.size is None else args.size, upscale=2)
    while True:
        rgb, _ = ri.image(newer_than=ri.now())
        robot.angle_vector(ri.angle_vector())
        tags = detector.detect(rgb, K, robot)
        if not tags:
            print('no tag in view')
        for tag in tags:
            yaw = np.degrees(np.arctan2(-tag.normal[1], -tag.normal[0]))
            print('tag {}: at {} m, faces the robot {:+.1f} deg off-axis'.format(
                tag.id, np.round(tag.position, 3), yaw))
        cube = cube_pose(tags) if args.size is None else None
        if cube is not None:
            print('cube: centre at {} m (from faces {})'.format(
                np.round(cube.position, 3), sorted(t.id for t in cube.tags)))
        pub.publish(markers(tags, ri.get_clock().now().to_msg()))
        if args.once:
            return
        ri.sleep(0.5)


if __name__ == '__main__':
    main()

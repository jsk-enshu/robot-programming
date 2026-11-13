#!/usr/bin/env python3
"""Look around for an object, drive up to it and point at it.

    ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true camera:=d405
    ros2 run jedy_vision find_object banana      # or cup, chair, bottle, ... (YOLO's COCO names)
    ros2 run jedy_vision find_object cup --distance 0.5

1. search: turn on the spot until YOLO sees the object.
2. approach: turn towards it and drive. How far it is comes from the image
   alone -- the ray through the bottom of its box hits the floor at a point
   the robot model can compute (the camera pose is known from the joint
   angles). That works at any range; the D405's depth only reaches 0.5 m and
   is printed next to it once close enough.
3. point: the arm on its side points at it (inverse kinematics), and Jedy
   says what it found (jedy_speech's speaker, if running).
"""

import argparse
import math

import numpy as np

from jedy_skrobot import connect
from jedy_vision import FLOOR_Z, ObjectDetector, pixel_on_floor, say, set_head, solve_ik

#: How Jedy calls the COCO classes it may be asked for.
JAPANESE = {'cup': 'コップ', 'banana': 'バナナ', 'chair': 'いす', 'bottle': 'ボトル',
            'person': '人', 'teddy bear': 'くまのぬいぐるみ', 'sports ball': 'ボール',
            'apple': 'りんご', 'book': '本', 'laptop': 'パソコン', 'potted plant': '植木鉢'}
#: Pointing: hand this far from the shoulder, along the line to the object [m].
POINT_REACH = 0.2


def point_at(robot, ri, target, seconds=2.0):
    """Point the arm on the target's side at ``target`` (base frame)."""
    side = 'larm' if target[1] > 0 else 'rarm'
    arm = getattr(robot, side)
    home = robot.angle_vector().copy()
    shoulder = arm.link_list[0].worldpos()
    direction = target - shoulder
    direction /= np.linalg.norm(direction)
    hand = shoulder + POINT_REACH * direction
    hand[2] = max(hand[2], FLOOR_Z + 0.15)  # never towards the floor itself
    if not solve_ik(robot, arm, hand):
        print('cannot point there (IK)')
        robot.angle_vector(home)
        return None
    ri.angle_vector(robot.angle_vector(), seconds)
    ri.wait_interpolation()
    return home


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('name', help="YOLO class, e.g. cup, banana, chair, bottle, person")
    p.add_argument('--distance', type=float, default=0.6, help='stop this far away [m]')
    p.add_argument('--score', type=float, default=0.3, help='min YOLO confidence')
    p.add_argument('--tilt', type=float, default=10.0, help='head tilt, down [deg]')
    p.add_argument('--min-speed', type=float, default=0.08, help='[m/s] (real base)')
    p.add_argument('--timeout', type=float, default=120.0, help='[s], robot clock')
    args = p.parse_args()

    detector = ObjectDetector(min_score=args.score)
    detector.class_id(args.name)  # fail now on a typo, with the list of names
    label = JAPANESE.get(args.name, args.name)
    robot, ri = connect()
    ri.servo_on()
    K, _ = ri.camera_info()
    ri.tuck_arms()  # arms up before driving: hanging, they drag on the floor
    set_head(robot, ri, 0.0, args.tilt)
    width = None
    start, reason = ri.now(), 'timeout'
    try:
        while ri.now() - start < args.timeout:
            rgb, stamp = ri.image(newer_than=ri.now() - 0.05)
            width = rgb.shape[1]
            robot.angle_vector(ri.angle_vector())
            found = detector.detect(rgb, args.name)
            if not found:
                print('no {} in view: turning'.format(args.name))
                ri.go_velocity(0.0, 0.0, 0.35)
                ri.sleep(0.1)
                continue
            _, score, (x1, y1, x2, y2) = found[0]
            u = 0.5 * (x1 + x2)
            error = (u - width / 2.0) / (width / 2.0)
            ground = pixel_on_floor(robot, K, u, y2, FLOOR_Z)
            dist = None if ground is None else math.hypot(ground[0], ground[1])
            print('{} {:.2f} at x={:.0f} ({:+.2f}), {} m away (from the floor)'.format(
                args.name, score, u, error, '?' if dist is None else '{:.2f}'.format(dist)))
            if dist is not None and dist <= args.distance and abs(error) < 0.15:
                reason = 'reached'
                break
            w = -0.8 * error
            v = 0.0
            if dist is not None and abs(error) < 0.3:
                v = min(0.12, max(args.min_speed, 0.4 * (dist - args.distance)))
            ri.go_velocity(v, 0.0, w)
            ri.sleep(0.05)
    finally:
        ri.stop()
    print('stopped by {} after {:.1f} s'.format(reason, ri.now() - start))
    if reason != 'reached':
        say(ri, '{}が見つからないのだ'.format(label))
        raise SystemExit(1)

    # Where it is, in 3-D: the floor point under the middle of its box.
    rgb, _ = ri.image(newer_than=ri.now())
    robot.angle_vector(ri.angle_vector())
    found = detector.detect(rgb, args.name)
    if found:
        _, _, (x1, y1, x2, y2) = found[0]
        target = pixel_on_floor(robot, K, 0.5 * (x1 + x2), y2, FLOOR_Z)
        if ri.count_publishers('/camera/depth/image_rect_raw') > 0:
            depth, _ = ri.depth_image()
            d = depth[int(0.5 * (y1 + y2)), int(0.5 * (x1 + x2))]
            print('depth at its centre: {}'.format(
                '{:.2f} m'.format(d) if np.isfinite(d) else 'no reading (beyond 0.5 m on a D405)'))
        say(ri, '{}を見つけたのだ'.format(label))
        if target is not None:
            home = point_at(robot, ri, target + np.array([0.0, 0.0, 0.05]))
            if home is not None:
                ri.sleep(2.0)
                ri.angle_vector(home, 2.0)
                ri.wait_interpolation()


if __name__ == '__main__':
    main()

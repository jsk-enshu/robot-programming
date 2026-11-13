#!/usr/bin/env python3
"""Find people in one head-camera image and save the picture with the result.

    ros2 run jedy_vision detect_people                      # boxes (YOLO)
    ros2 run jedy_vision detect_people --pose               # + body pose (MediaPipe)
    ros2 run jedy_vision detect_people --pose --face --hands  # + face (box, mesh), hands
    ros2 run jedy_vision detect_people --image photo.jpg    # a file, no robot

In Gazebo, start with a person to look at, and look up at them (the head
camera is 0.4 m above the floor):

    ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true person_x:=1.6 person_y:=0.0
    ros2 run jedy_vision detect_people --pose --face --hands --tilt -25

The first thing to try when a demo "sees nobody": does this see anyone?
"""

import argparse
import time

import cv2
import numpy as np

from jedy_vision import (FaceEstimator, HAND_BONES, HandEstimator, PersonDetector,
                         PoseEstimator, box_distance, set_head)

#: Landmark pairs drawn as the skeleton (MediaPipe Pose indices).
SKELETON = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
            (11, 23), (12, 24), (23, 24), (23, 25), (25, 27), (24, 26), (26, 28)]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pose', action='store_true', help='also estimate body pose')
    p.add_argument('--face', action='store_true',
                   help='also find faces (box and 478-point mesh; needs the pose, '
                        'which is then estimated even without --pose)')
    p.add_argument('--hands', action='store_true', help='also find hands (21 points)')
    p.add_argument('--tilt', type=float, default=None,
                   help='first tilt the head to this [deg] (negative: up)')
    p.add_argument('--image', help='use this image file instead of the camera')
    p.add_argument('--save', default='people.png', help='annotated output image')
    args = p.parse_args()

    # Before connect(): connected, the model's first (loading) run is ~8x slower.
    detector = PersonDetector()
    depth = None
    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit('cannot read {}'.format(args.image))
        rgb = bgr[:, :, ::-1].copy()
    else:
        from jedy_skrobot import connect
        robot, ri = connect()
        if not ri.has_camera():
            raise SystemExit('no head camera image. Gazebo: launch with '
                             'render_sensors:=true (the default).')
        if args.tilt is not None:
            ri.servo_on(joints=['head_joint0', 'head_joint1'])
            set_head(robot, ri, 0.0, args.tilt)
            ri.sleep(0.5)
        rgb, _ = ri.image(newer_than=ri.now() - 0.05)
        if ri.count_publishers('/camera/depth/image_rect_raw') > 0:
            depth, _ = ri.depth_image()

    start = time.perf_counter()
    people = detector.detect(rgb)
    print('{} person(s), {:.0f} ms'.format(len(people), 1000 * (time.perf_counter() - start)))
    out = np.ascontiguousarray(rgb[:, :, ::-1])
    for i, person in enumerate(people):
        x1, y1, x2, y2 = (int(v) for v in person.box)
        dist = box_distance(depth, person.box) if depth is not None else None
        label = '{} {:.2f}{}'.format(i, person.score,
                                     '' if dist is None else ' {:.2f} m'.format(dist))
        print('  person {}: box ({}, {})-({}, {}) score {:.2f}{}'.format(
            i, x1, y1, x2, y2, person.score,
            '' if dist is None else ', {:.2f} m away'.format(dist)))
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 120, 0), 2)
        cv2.putText(out, label, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 120, 0), 2)
    poses = []
    if args.pose or args.face:
        with PoseEstimator(max_people=2) as estimator:
            poses = estimator.estimate(rgb)
        print('{} pose(s)'.format(len(poses)))
    if args.pose:
        for pose in poses:
            for a, b in SKELETON:
                pa, pb = pose.point(a), pose.point(b)
                if pa is not None and pb is not None:
                    cv2.line(out, tuple(int(v) for v in pa), tuple(int(v) for v in pb),
                             (0, 220, 0), 2)
    if args.face:
        with FaceEstimator() as estimator:
            faces = estimator.estimate(rgb, poses)
        print('{} face(s)'.format(len(faces)))
        for face in faces:
            x1, y1, x2, y2 = (int(v) for v in face.box)
            print('  face: box ({}, {})-({}, {}) score {:.2f}, mesh {}'.format(
                x1, y1, x2, y2, face.score,
                'none' if face.landmarks is None else '{} points'.format(len(face.landmarks))))
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 1)
            if face.landmarks is not None:
                for x, y in face.landmarks[::3]:
                    cv2.circle(out, (int(x), int(y)), 0, (255, 255, 0), -1)
    if args.hands:
        with HandEstimator() as estimator:
            hands = estimator.estimate(rgb)
        print('{} hand(s)'.format(len(hands)))
        for hand in hands:
            print('  hand: {} ({:.2f}), wrist at ({:.0f}, {:.0f})'.format(
                hand.side, hand.score, *hand.landmarks[0]))
            for a, b in HAND_BONES:
                cv2.line(out, tuple(int(v) for v in hand.landmarks[a]),
                         tuple(int(v) for v in hand.landmarks[b]), (0, 0, 255), 1)
    cv2.imwrite(args.save, out)
    print('saved', args.save)


if __name__ == '__main__':
    main()

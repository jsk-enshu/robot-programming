#!/usr/bin/env python3
"""Copy a person's arms like a mirror, by inverse kinematics.

MediaPipe gives the person's shoulder, elbow and wrist in the image. For each
arm, the direction shoulder -> wrist and how far the arm is stretched become a
hand target next to the robot's own shoulder, and ``inverse_kinematics`` moves
the robot's hand there (position only). Like a mirror: the person's LEFT arm
moves the robot's RIGHT arm, the one on the same side of the picture.

    image right  ->  robot -y (its right)
    image down   ->  robot -z
    (always a little forward, so the hands stay in front of the body)

    ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true   # terminal 1
    ros2 run jedy_vision mimic_arms --once                        # terminal 2
    ros2 run jedy_vision mimic_arms --image photo.jpg --once      # from a photo

The real robot moves its arms: keep clear of them.
"""

import argparse

import numpy as np
from skrobot.coordinates import Coordinates

from jedy_skrobot import connect
from jedy_vision import (LEFT_ELBOW, LEFT_SHOULDER, LEFT_WRIST, RIGHT_ELBOW,
                         RIGHT_SHOULDER, RIGHT_WRIST, PoseEstimator)

#: Distance from the robot's shoulder to the hand target at full stretch [m]
#: (the arm is about 0.28 m from shoulder to hand).
REACH = 0.24
#: Forward share of the target direction, before normalising: keeps the
#: hands in front of the body instead of through it.
FORWARD = 0.6
#: The hand target never goes below this height or behind this x in the base
#: frame [m]: lower, the hand hits the base (an "arms down" person put it at
#: z = -0.13 and the hand stopped 44 mm short, against the body).
MIN_Z = 0.0
MIN_X = 0.10


def human_arm(pose, shoulder, elbow, wrist):
    """Image direction and stretch of one human arm, or None if not visible.

    Returns
    -------
    tuple or None
        ``(dx, dy, stretch)``: unit direction shoulder -> wrist in the image
        (x right, y down) and |shoulder-wrist| / (upper + lower arm), 0..1.
    """
    s, e, w = pose.point(shoulder), pose.point(elbow), pose.point(wrist)
    if s is None or e is None or w is None:
        return None
    reach = np.linalg.norm(w - s)
    length = np.linalg.norm(e - s) + np.linalg.norm(w - e)
    if reach < 1e-6 or length < 1e-6:
        return None
    d = (w - s) / reach
    return float(d[0]), float(d[1]), float(np.clip(reach / length, 0.0, 1.0))


def hand_target(robot_arm, dx, dy, stretch):
    """Hand position for the robot arm in the base frame [m]."""
    direction = np.array([FORWARD, -dx, -dy])
    direction /= np.linalg.norm(direction)
    shoulder = robot_arm.link_list[0].worldpos()
    target = shoulder + REACH * max(stretch, 0.3) * direction
    target[0] = max(target[0], MIN_X)
    target[2] = max(target[2], MIN_Z)
    return target


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--once', action='store_true', help='copy one pose and stop')
    p.add_argument('--image', help='take the pose from this image file')
    p.add_argument('--time', type=float, default=1.5, help='seconds per motion')
    p.add_argument('--timeout', type=float, default=60.0, help='[s], robot clock')
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on()
    with PoseEstimator() as estimator:
        mimic(robot, ri, estimator, args)


def mimic(robot, ri, estimator, args):
    """The copy loop (see the module docstring)."""
    still = None
    if args.image:
        import cv2
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit('cannot read {}'.format(args.image))
        still = bgr[:, :, ::-1].copy()
    elif not ri.has_camera():
        raise SystemExit('no head camera image (render_sensors:=true in Gazebo).')

    # Mirror: the person's left arm drives the robot's right arm.
    pairs = [('rarm', robot.rarm, (LEFT_SHOULDER, LEFT_ELBOW, LEFT_WRIST)),
             ('larm', robot.larm, (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST))]
    start = ri.now()
    while ri.now() - start < args.timeout:
        rgb = still if still is not None else ri.image()[0]
        poses = estimator.estimate(rgb)
        if not poses:
            print('no person pose in view')
            if args.once:
                raise SystemExit(1)
            ri.sleep(0.5)
            continue
        robot.angle_vector(ri.angle_vector())
        targets = {}
        for name, arm, landmarks in pairs:
            measured = human_arm(poses[0], *landmarks)
            if measured is None:
                print('{}: that arm of the person is not visible, keep it'.format(name))
                continue
            target = hand_target(arm, *measured)
            solved = arm.inverse_kinematics(Coordinates(pos=target), rotation_axis=False,
                                            warn_if_fail=False)
            if solved is False or solved is None:
                print('{}: IK did not converge for {} m, keep it'.format(
                    name, np.round(target, 3)))
                continue
            targets[name] = target
            print('{}: person arm dir ({:+.2f}, {:+.2f}) stretch {:.2f} -> hand {} m'.format(
                name, measured[0], measured[1], measured[2], np.round(target, 3)))
        if targets:
            ri.angle_vector(robot.angle_vector(), args.time)
            ri.wait_interpolation()
            robot.angle_vector(ri.angle_vector())
            for name, target in targets.items():
                reached = getattr(robot, name).end_coords.worldpos()
                print('{}: hand position error {:.1f} mm'.format(
                    name, 1000 * np.linalg.norm(reached - target)))
        if args.once:
            return


if __name__ == '__main__':
    main()

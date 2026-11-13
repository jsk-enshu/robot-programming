#!/usr/bin/env python3
"""Wave back at a person who raises a hand, and say hello.

    ros2 run jedy_vision wave_back            # watch the camera
    ros2 run jedy_vision wave_back --test     # just wave once (check the motion)

MediaPipe Pose gives the person's shoulders and wrists. A hand counts as
raised when its wrist is above its shoulder in the image by more than
``--margin`` of the shoulder width, for ``--frames`` frames in a row (one
lucky frame is not a wave). Then Jedy says こんにちは (jedy_speech's speaker,
if running) and waves its arm on the same side as the person's raised hand,
mirror-like.
"""

import argparse

import numpy as np
from skrobot.coordinates import Coordinates

from jedy_skrobot import connect
from jedy_vision import (LEFT_SHOULDER, LEFT_WRIST, RIGHT_SHOULDER, RIGHT_WRIST,
                         PoseEstimator, say)

#: Hand positions relative to the shoulder for the wave [m] (reachable from
#: the reset pose, checked with the Jedy model); y mirrored for larm.
WAVE_POINTS = [(0.0, -0.10, 0.20), (0.0, 0.0, 0.17)]


def raised_hand(pose, margin):
    """``'left'`` / ``'right'`` (the person's side) or None."""
    s_l, s_r = pose.point(LEFT_SHOULDER), pose.point(RIGHT_SHOULDER)
    if s_l is None or s_r is None:
        return None
    width = np.linalg.norm(s_l - s_r)
    for side, shoulder, wrist in (('left', s_l, pose.point(LEFT_WRIST)),
                                  ('right', s_r, pose.point(RIGHT_WRIST))):
        if wrist is not None and shoulder[1] - wrist[1] > margin * width:
            return side
    return None


def wave(robot, ri, side, times=3):
    """Wave the arm ``side`` (rarm/larm) a few times, then put it back."""
    arm = getattr(robot, side)
    home = robot.angle_vector().copy()
    robot.reset_pose()
    seed = {j.name: j.joint_angle() for j in arm.joint_list}
    robot.angle_vector(home)
    for j in arm.joint_list:
        j.joint_angle(seed[j.name])
    poses = []
    for dx, dy, dz in WAVE_POINTS:
        if side == 'larm':
            dy = -dy
        for j in arm.joint_list:  # every point from the reset pose (IK is local)
            j.joint_angle(seed[j.name])
        target = arm.link_list[0].worldpos() + np.array([dx, dy, dz])
        ok = arm.inverse_kinematics(Coordinates(pos=target), rotation_axis=False)
        if ok is False or ok is None:
            print('cannot wave there (IK)')
            robot.angle_vector(home)
            return
        poses.append(robot.angle_vector().copy())
    ri.angle_vector(poses[0], 1.5)
    ri.wait_interpolation()
    for k in range(2 * times):
        ri.angle_vector(poses[(k + 1) % 2], 0.4)
        ri.wait_interpolation()
        robot.angle_vector(ri.angle_vector())
        print('  wave {}: hand at {} m'.format(k, np.round(arm.end_coords.worldpos(), 3)))
    ri.angle_vector(home, 1.5)
    ri.wait_interpolation()
    robot.angle_vector(ri.angle_vector())


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--margin', type=float, default=0.2,
                   help='wrist above shoulder by this x shoulder width')
    p.add_argument('--frames', type=int, default=3, help='frames in a row')
    p.add_argument('--cooldown', type=float, default=5.0, help='[s] between waves')
    p.add_argument('--test', action='store_true', help='wave once and exit')
    p.add_argument('--timeout', type=float, default=300.0, help='[s], robot clock')
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on()
    robot.angle_vector(ri.angle_vector())
    if args.test:
        say(ri, 'こんにちは')
        wave(robot, ri, 'rarm')
        return
    if not ri.has_camera():
        raise SystemExit('no head camera image')
    start, streak, last_wave = ri.now(), 0, -1e9
    with PoseEstimator() as estimator:
        while ri.now() - start < args.timeout:
            rgb, _ = ri.image(newer_than=ri.now() - 0.05)
            poses = estimator.estimate(rgb)
            side = raised_hand(poses[0], args.margin) if poses else None
            streak = streak + 1 if side else 0
            if side and streak >= args.frames and ri.now() - last_wave > args.cooldown:
                print('{} hand raised'.format(side))
                say(ri, 'こんにちは')
                # Mirror: the person's left hand is on the robot's right.
                robot.angle_vector(ri.angle_vector())
                wave(robot, ri, 'rarm' if side == 'left' else 'larm')
                last_wave, streak = ri.now(), 0
            ri.sleep(0.05)


if __name__ == '__main__':
    main()

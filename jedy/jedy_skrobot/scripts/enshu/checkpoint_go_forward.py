#!/usr/bin/env python3
"""Drive forward until a sensor says stop.

The scikit-robot version of jedyeus/exercise/checkpoint2-1-go-forward.l (which
stopped on the Atom S3 button, not used any more) and of the "head pitch
moved 30 degrees" example from the 2025 text. Pick the sensor:

    ros2 run jedy_skrobot checkpoint_go_forward --sensor head    # tilt the head by hand
    ros2 run jedy_skrobot checkpoint_go_forward --sensor depth   # something ahead
    ros2 run jedy_skrobot checkpoint_go_forward --sensor time --timeout 3

``head`` turns the head's servos OFF for the run (and on again at the end),
so the head can be tilted by hand, and stops once its pitch has changed by
``--threshold`` degrees. The joint angle is a sensor even with the torque off.
``depth`` stops once the head camera's depth in the middle of the image is
nearer than ``--distance`` (Gazebo ``box:=true`` puts a box
in front). ``time`` uses no sensor at all and is there to check the base.
Times are on the robot's clock (``ri.now()``), so in a simulator that runs
slower than real time ``--timeout 3`` is still three seconds of driving.
"""

import argparse

import numpy as np

from jedy_skrobot import connect

HEAD = ['head_joint0', 'head_joint1']


def centre_depth(ri, size=40):
    """Median depth [m] of the middle ``size`` x ``size`` pixels (NaN: none)."""
    depth, _ = ri.depth_image()
    h, w = depth.shape
    patch = depth[(h - size) // 2:(h + size) // 2,
                  (w - size) // 2:(w + size) // 2]
    patch = patch[np.isfinite(patch)]
    return float(np.median(patch)) if patch.size else float('nan')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sensor', choices=['head', 'depth', 'time'],
                   default='head')
    p.add_argument('--speed', type=float, default=0.1, help='[m/s]')
    p.add_argument('--threshold', type=float, default=30.0,
                   help='head: degrees of pitch change that stop the base')
    p.add_argument('--distance', type=float, default=0.5,
                   help='depth: metres that stop the base')
    p.add_argument('--timeout', type=float, default=30.0,
                   help='stop anyway after this many seconds')
    p.add_argument('--rate', type=float, default=10.0, help='loop rate [Hz]')
    args = p.parse_args()

    robot, ri = connect()
    ri.tuck_arms()  # arms up before driving: hanging, they drag on the floor
    if args.sensor == 'depth' and not ri.has_camera():
        print('this robot has no head camera (nobody publishes '
              '/camera/color/image_rect_raw); use --sensor head or time')
        return
    pitch_index = robot.joint_list.index(robot.head_joint1)
    if args.sensor == 'head':
        ri.servo_off(HEAD)
        print('head servo OFF: tilt the head to stop the base')
    start_pitch = ri.angle_vector()[pitch_index]
    start = ri.now()
    reason = 'timeout'
    try:
        while ri.now() - start < args.timeout:
            ri.go_velocity(args.speed, 0.0, 0.0)   # one command per cycle
            if args.sensor == 'head':
                change = np.rad2deg(ri.angle_vector()[pitch_index]
                                    - start_pitch)
                print('head pitch moved {:.1f} deg'.format(change))
                if abs(change) > args.threshold:
                    reason = 'head'
                    break
            elif args.sensor == 'depth':
                d = centre_depth(ri)
                print('depth ahead {:.2f} m'.format(d))
                if d < args.distance:
                    reason = 'depth'
                    break
            ri.sleep(1.0 / args.rate)
    finally:
        ri.stop()
        if args.sensor == 'head':
            ri.servo_on(HEAD)
    print('stopped by {} after {:.1f} s'.format(
        reason, ri.now() - start))


if __name__ == '__main__':
    main()

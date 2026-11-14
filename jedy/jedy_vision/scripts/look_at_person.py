#!/usr/bin/env python3
"""Keep the head pointed at a person's upper body.

The base does not move: only head_controller is commanded, so this is also
the controller-by-controller exercise (sample_controllers) with a camera in
the loop. Each cycle turns the angle between the image centre and the person
into a pan/tilt correction with the camera intrinsics (``camera_info``):

    angle = atan((u - c_x) / f_x)

    ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true   # terminal 1
    ros2 run jedy_vision look_at_person                           # terminal 2
"""

import argparse
import math

import numpy as np

from jedy_skrobot import connect
from jedy_vision import PersonDetector


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--gain', type=float, default=0.7,
                   help='fraction of the measured angle corrected per cycle')
    p.add_argument('--time', type=float, default=0.5,
                   help='seconds per head motion (and per cycle)')
    p.add_argument('--aim', type=float, default=0.15,
                   help='aim this far down from the top of the box (0..1)')
    p.add_argument('--settle', type=float, default=2.0,
                   help='stop once within this many degrees of the person')
    p.add_argument('--keep', action='store_true', help='keep tracking')
    p.add_argument('--timeout', type=float, default=60.0, help='[s], robot clock')
    args = p.parse_args()

    # Before connect(): connected, the model's first (loading) run is ~8x slower.
    detector = PersonDetector()
    robot, ri = connect()
    if not ri.has_camera():
        raise SystemExit('no head camera image (render_sensors:=true in Gazebo).')
    K, _ = ri.camera_info()
    fx, fy, cx0, cy0 = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    pan, tilt = robot.head_joint0, robot.head_joint1
    start = ri.now()
    reason = 'timeout'
    try:
        while ri.now() - start < args.timeout:
            rgb, stamp = ri.image(newer_than=ri.now() - 0.05)
            people = detector.detect(rgb)
            if not people:
                print('nobody in view')
                ri.sleep(args.time)
                continue
            x1, y1, x2, y2 = people[0].box
            u, v = 0.5 * (x1 + x2), y1 + args.aim * (y2 - y1)
            # Right of centre -> turn right (pan is counter-clockwise positive);
            # below centre -> look down (tilt is positive nose-down).
            d_pan = -math.atan((u - cx0) / fx)
            d_tilt = math.atan((v - cy0) / fy)
            robot.angle_vector(ri.angle_vector())
            print('pan {:+6.1f} deg tilt {:+6.1f} deg | off by {:+5.1f} / {:+5.1f} deg'.format(
                math.degrees(pan.joint_angle()), math.degrees(tilt.joint_angle()),
                math.degrees(d_pan), math.degrees(d_tilt)))
            if (not args.keep and abs(math.degrees(d_pan)) < args.settle
                    and abs(math.degrees(d_tilt)) < args.settle):
                reason = 'settled'
                break
            pan.joint_angle(float(np.clip(pan.joint_angle() + args.gain * d_pan,
                                          pan.min_angle, pan.max_angle)))
            tilt.joint_angle(float(np.clip(tilt.joint_angle() + args.gain * d_tilt,
                                           tilt.min_angle, tilt.max_angle)))
            ri.angle_vector(robot.angle_vector(), args.time,
                            controller_type='head_controller')
            ri.wait_interpolation('head_controller')
    except KeyboardInterrupt:
        reason = 'Ctrl-C'
        # The head stops where it is, not at the last target.
        ri.cancel_angle_vector(controller_type='head_controller')
    print('looking at the person' if reason == 'settled' else 'stopped by ' + reason)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Drive and turn until the robot is tilted, then stop.

The scikit-robot version of jedyeus/exercise/checkpoint2-2-go-forward-imu.l:
read roll/pitch/yaw from the IMU every cycle and stop the base once the tilt
changes by more than a threshold (lift or tip the robot to try it).

    ros2 run jedy_skrobot checkpoint_go_forward_imu

Needs an IMU on /imu. Gazebo has one. The RCB-4mini board does
not, so on the real robot this runs only if a sensor board publishes /imu.
"""

import argparse

import numpy as np

from jedy_skrobot import connect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--speed', type=float, default=0.1, help='[m/s]')
    p.add_argument('--turn', type=float, default=0.1, help='[rad/s]')
    p.add_argument('--threshold', type=float, default=30.0,
                   help='degrees of roll or pitch change that stop the base')
    p.add_argument('--timeout', type=float, default=30.0)
    p.add_argument('--rate', type=float, default=10.0)
    args = p.parse_args()

    _, ri = connect()
    ri.tuck_arms()  # arms up before driving: hanging, they drag on the floor
    if not ri.has_imu():
        print('this robot has no IMU (nobody publishes /imu): the RCB-4mini '
              'board has none of its own. Nothing to do.')
        return
    start_rpy = ri.roll_pitch_yaw()
    start = ri.now()
    reason = 'timeout'
    try:
        while ri.now() - start < args.timeout:
            ri.go_velocity(args.speed, 0.0, args.turn)
            rpy = ri.roll_pitch_yaw()
            print('roll {:6.1f} pitch {:6.1f} yaw {:6.1f} [deg]'.format(
                *np.rad2deg(rpy)))
            tilt = np.rad2deg(np.abs(rpy[:2] - start_rpy[:2]))
            if np.max(tilt) > args.threshold:
                reason = 'tilt'
                break
            ri.sleep(1.0 / args.rate)
    finally:
        ri.stop()
    print('stopped by {} after {:.1f} s'.format(
        reason, ri.now() - start))


if __name__ == '__main__':
    main()

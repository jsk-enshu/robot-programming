#!/usr/bin/env python3
"""Drive the base at one steady speed, e.g. to record its sound.

    ros2 run jedy_skrobot drive_steady                     # forward 0.12 m/s for 6 s (0.72 m)
    ros2 run jedy_skrobot drive_steady --turn              # turn in place 1.2 rad/s for 6 s
    ros2 run jedy_skrobot drive_steady --vx 0.15 --seconds 4

Only the base moves: the arm and head servos are left as they are (ON or
OFF). On the real robot they start OFF and the wheels start driven, so this
drives with the arms limp; turning the arm servos on would hold the start
pose, whose hands rest on the floor and keep the base from moving. Counts
down so a recording can be started, drives, and stops. Ctrl-C stops the base.
The real base does not move below 0.078 m/s (forward / sideways) or
0.83 rad/s (turning), and caps at about 0.156 m/s / 1.66 rad/s.
"""

import argparse
import time

from jedy_skrobot import connect


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--vx', type=float, default=0.12, help='forward [m/s]')
    p.add_argument('--vy', type=float, default=0.0, help='left [m/s]')
    p.add_argument('--vtheta', type=float, default=0.0, help='turn, left positive [rad/s]')
    p.add_argument('--turn', action='store_true', help='turn in place at 1.2 rad/s instead')
    p.add_argument('--seconds', type=float, default=6.0)
    p.add_argument('--countdown', type=int, default=5, help='seconds before it starts')
    args = p.parse_args()
    vx, vy, vtheta = (0.0, 0.0, 1.2) if args.turn else (args.vx, args.vy, args.vtheta)

    robot, ri = connect()
    print('drive vx={} m/s, vy={} m/s, vtheta={} rad/s for {} s'.format(vx, vy, vtheta, args.seconds))
    for n in range(args.countdown, 0, -1):
        print('  starts in {} ...'.format(n), flush=True)
        time.sleep(1.0)
    try:
        print('driving', flush=True)
        ri.go_velocity(vx, vy, vtheta, duration=args.seconds)
    finally:
        ri.stop()
    print('stopped')


if __name__ == '__main__':
    main()

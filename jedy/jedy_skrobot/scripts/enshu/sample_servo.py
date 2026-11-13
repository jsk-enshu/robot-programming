#!/usr/bin/env python3
"""Torque off, then on again: the scikit-robot ``:servo-off`` / ``:servo-on``.

On every backend (the real robot, Gazebo) the released joints go
LIMP and fall under gravity; ``joint_states`` keeps reporting where they are.
On the real robot, support the arms or run it with the robot seated.

    ros2 run jedy_skrobot sample_servo                       # every joint
    ros2 run jedy_skrobot sample_servo --joints head_joint0 head_joint1
    ros2 run jedy_skrobot sample_servo --limb rarm           # the right arm

It prints the joints that moved while off (how far they fell), and checks
that once ``servo_on`` returns the robot holds where it is (moves less than
HOLD_DEG in the next second) instead of snapping back.
"""

import argparse

import numpy as np

from jedy_skrobot import connect

#: Joints that moved less than this while off are not listed [deg].
MOVED_DEG = 1.0
#: servo_on must hold within this of where the joints were [deg].
HOLD_DEG = 3.0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joints', nargs='*', default=None,
                   help='joint names; default every joint')
    p.add_argument('--limb', choices=['rarm', 'larm', 'head'], default=None,
                   help='every joint of one limb instead of --joints')
    p.add_argument('--wait', type=float, default=3.0,
                   help='seconds (robot time) to stay off')
    args = p.parse_args()

    robot, ri = connect()
    joints = args.joints
    if args.limb is not None:
        joints = [j.name for j in getattr(robot, args.limb).joint_list]
    names = [j.name for j in robot.joint_list]

    before = np.rad2deg(ri.angle_vector())
    ri.servo_off(joints=joints)
    print('servo OFF ({}): the joints are limp'.format(
        'every joint' if joints is None else ', '.join(joints)))
    ri.sleep(args.wait)
    fallen = np.rad2deg(ri.angle_vector())
    for name, b, f in zip(names, before, fallen):
        if abs(f - b) >= MOVED_DEG:
            print('  {:20s} {:8.1f} -> {:8.1f} deg (fell {:+.1f})'.format(
                name, b, f, f - b))
    if np.max(np.abs(fallen - before)) < MOVED_DEG:
        print('  nothing moved more than {} deg'.format(MOVED_DEG))

    # servo_on first sends the pose the robot is in now, so nothing jumps.
    # A joint still sliding down may take it a few re-syncs (it says so).
    ri.servo_on(joints=joints)
    on = np.rad2deg(ri.angle_vector())
    further = np.max(np.abs(on - fallen))
    if further >= MOVED_DEG:
        print('  (it fell another {:.1f} deg before servo_on took hold)'
              .format(further))
    ri.sleep(1.0)
    held = np.rad2deg(ri.angle_vector())
    worst = int(np.argmax(np.abs(held - on)))
    jump = abs(held[worst] - on[worst])
    print('servo ON: holding where it fell, largest change {:.2f} deg in the '
          'first second ({}: {:.1f} -> {:.1f})'.format(
              jump, names[worst], on[worst], held[worst]))
    if jump > HOLD_DEG:
        raise SystemExit('servo_on moved {} by {:.2f} deg (> {})'
                         .format(names[worst], jump, HOLD_DEG))


if __name__ == '__main__':
    main()

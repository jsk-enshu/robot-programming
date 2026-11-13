#!/usr/bin/env python3
"""Swing joints back and forth at a steady pace, e.g. to record the servos.

    ros2 run jedy_skrobot swing_joint                              # head left-right, +-30 deg, 0.4 s each way, 10 s
    ros2 run jedy_skrobot swing_joint --joints head_joint1 --amplitude 15
    ros2 run jedy_skrobot swing_joint --seconds 15

Only the named joints move: their servos are turned on and their controllers
alone are commanded, so the arms are left as they are (limp on the real
robot, which starts with the servos OFF). Each joint swings by +-amplitude
around where it is now and comes back. Fast by default: a servo moving
slowly is nearly silent. Before anything moves, the swing is checked against
the robot's own model, posed as the robot is now, for joint limits, for the
joint's speed limit (the trajectory between two poses peaks at about
:data:`PEAK_FACTOR` times its mean speed) and for self-collision (contacts
already present now are ignored); a swing that breaks one is refused.
"""

import argparse
import math
import time

import numpy as np

from jedy_skrobot import connect

#: Poses checked along each half swing.
CHECK_STEPS = 15
#: Peak over mean speed of a trajectory from rest to rest (a quintic: 15/8).
PEAK_FACTOR = 1.875


def new_contacts(model, start, joints, amplitude):
    """Self-contacts the swing makes that ``start`` does not have.

    Returns
    -------
    list of tuple
        ``(angle offset [deg], (link, link))``; empty when the swing is free.
    """
    model.angle_vector(start)
    _, pairs = model.self_collision_check()
    base = {frozenset(p) for p in pairs}
    found = []
    for offset in np.linspace(-amplitude, amplitude, 2 * CHECK_STEPS + 1):
        model.angle_vector(start)
        for name in joints:
            j = getattr(model, name)
            j.joint_angle(j.joint_angle() + math.radians(offset))
        _, pairs = model.self_collision_check()
        for pair in {frozenset(p) for p in pairs} - base:
            found.append((round(float(offset), 1), tuple(sorted(pair))))
    return found


def check_swing(start, joints, amplitude, period):
    """Raise SystemExit if the swing breaks a joint's limits or makes a contact."""
    # A model with the collision meshes; connect() loads none.
    from skrobot.models.jedy import Jedy
    model = Jedy()
    model.angle_vector(start)
    for name in joints:
        j = getattr(model, name)
        lo = j.joint_angle() - math.radians(amplitude)
        hi = j.joint_angle() + math.radians(amplitude)
        if lo < j.min_angle or hi > j.max_angle:
            raise SystemExit('{}: {:.0f}..{:.0f} deg is outside its limits {:.0f}..{:.0f} deg; '
                             'use a smaller --amplitude'.format(
                                 name, math.degrees(lo), math.degrees(hi),
                                 math.degrees(j.min_angle), math.degrees(j.max_angle)))
        peak = PEAK_FACTOR * 2.0 * math.radians(amplitude) / period
        if peak > j.max_joint_velocity:
            raise SystemExit('{}: +-{} deg in {} s peaks at {:.0f} deg/s, over its limit of '
                             '{:.0f} deg/s; use a longer --period'.format(
                                 name, amplitude, period, math.degrees(peak),
                                 math.degrees(j.max_joint_velocity)))
    contacts = new_contacts(model, start, joints, amplitude)
    if contacts:
        raise SystemExit('refused: the swing would make the robot touch itself, e.g. {} '
                         '(offset [deg], links). Use other joints or a smaller --amplitude.'
                         .format(contacts[:3]))


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--joints', nargs='+', default=['head_joint0'])
    p.add_argument('--amplitude', type=float, default=30.0, help='[deg] each way')
    p.add_argument('--period', type=float, default=0.4, help='[s] for one swing one way')
    p.add_argument('--seconds', type=float, default=10.0, help='how long to keep swinging')
    p.add_argument('--countdown', type=int, default=5, help='seconds before it starts')
    args = p.parse_args()

    robot, ri = connect()
    start = ri.angle_vector()
    check_swing(start, args.joints, args.amplitude, args.period)
    controllers = sorted({ri.controller_of(n) for n in args.joints})

    ri.servo_on(joints=args.joints)     # these joints only; holds them where they are
    robot.angle_vector(ri.angle_vector())
    centre = {n: getattr(robot, n).joint_angle() for n in args.joints}

    def send(seconds):
        for c in controllers:
            ri.angle_vector(robot.angle_vector(), seconds, controller_type=c)
        for c in controllers:
            ri.wait_interpolation(c)

    print('swing {} by +-{} deg, {} s each way, for {} s'.format(
        args.joints, args.amplitude, args.period, args.seconds))
    for n in range(args.countdown, 0, -1):
        print('  starts in {} ...'.format(n), flush=True)
        time.sleep(1.0)
    try:
        print('swinging', flush=True)
        end = time.monotonic() + args.seconds
        sign = 1.0
        while time.monotonic() < end:
            for n in args.joints:
                getattr(robot, n).joint_angle(centre[n] + sign * math.radians(args.amplitude))
            send(args.period)
            sign = -sign
    finally:
        for n in args.joints:
            getattr(robot, n).joint_angle(centre[n])
        send(2.0 * args.period)
    print('done')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Teach by hand: move one Jedy (the leader), the other one (the follower) copies it.

Two real robots on one PC, each on its own Dual USB Adapter, each brought up
under its own namespace::

    ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_leader \\
        port:=<leader's adapter> rviz:=false joy:=false wheels:=false
    ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_follower \\
        port:=<follower's adapter> rviz:=false joy:=false wheels:=false
    ros2 run jedy_skrobot teach_follow --record poses.yaml

The leader stays limp (its torque is off after the launch, and this program
never turns it on): move its arms by hand. The follower's torque is turned on
for the chosen parts; it first moves to the leader's pose in ``--approach``
seconds, then follows it. Enter records the leader's pose (with
``--record``), q quits; the follower keeps holding where it is.

Play the recorded poses back on a robot (one second between poses)::

    ros2 run jedy_skrobot teach_follow --play poses.yaml --follower /jedy_follower

Keys (type them in this terminal)::

    Enter   record the leader's pose (with --record)
    q       quit
"""

import argparse
import select
import sys
import termios
import time
import tty

import numpy as np
import yaml

#: Part name -> the trajectory controller that drives it.
PARTS = {
    'head': 'head_controller',
    'rarm': 'right_arm_controller',
    'larm': 'left_arm_controller',
    'rhand': 'right_gripper_controller',
    'lhand': 'left_gripper_controller',
}


def read_key(timeout):
    """Return one character typed in the terminal, or None after timeout."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def joints_of(ri, parts):
    """The joint names of ``parts``, in controller order."""
    return [j for p in parts for j in ri.joints_of(PARTS[p])]


def pose_of(robot, ri, names):
    """{joint: angle [rad]} of the robot ``ri`` now."""
    robot.angle_vector(ri.angle_vector())
    return {n: float(getattr(robot, n).joint_angle()) for n in names}


def send(robot, ri, pose, duration, parts):
    """Send ``pose`` ({joint: rad}) to the controllers of ``parts``."""
    robot.angle_vector(ri.angle_vector())
    for name, angle in pose.items():
        getattr(robot, name).joint_angle(angle)
    av = robot.angle_vector()
    for p in parts:
        ri.angle_vector(av, duration, controller_type=PARTS[p])


def follow(args, parts):
    from jedy_skrobot import connect
    leader_robot, leader = connect(namespace=args.leader)
    follower_robot, follower = connect(namespace=args.follower)
    names = joints_of(follower, parts)

    # The leader is only read. Its torque is off after the launch; if someone
    # turned it on, moving it by hand fights the servos.
    print('leader {}: read only (move it by hand; its torque stays as it is)'
          .format(args.leader))
    follower.servo_on(names)
    print('follower {}: torque on for {}'.format(args.follower, ', '.join(parts)))

    pose = pose_of(leader_robot, leader, names)
    print('moving the follower to the leader\'s pose in {:.1f} s'.format(args.approach))
    send(follower_robot, follower, pose, args.approach, parts)
    for p in parts:
        follower.wait_interpolation(PARTS[p])

    recorded = []
    print('following. Enter: record the pose{}, q: quit'.format(
        '' if args.record else ' (needs --record)'))
    period = 1.0 / args.rate
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            start = time.monotonic()
            pose = pose_of(leader_robot, leader, names)
            send(follower_robot, follower, pose, args.lead, parts)
            key = read_key(max(0.0, period - (time.monotonic() - start)))
            if key == 'q':
                break
            if key in ('\n', '\r') and args.record:
                recorded.append({n: round(float(np.rad2deg(a)), 2)
                                 for n, a in pose.items()})
                with open(args.record, 'w') as f:
                    yaml.safe_dump({'parts': list(parts), 'poses': recorded}, f,
                                   sort_keys=False)
                print('recorded pose {} -> {}'.format(len(recorded), args.record))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    print('done; the follower holds its pose (torque on). '
          'ri.servo_off() releases it.')


def play(args):
    from jedy_skrobot import connect
    with open(args.play) as f:
        recorded = yaml.safe_load(f)
    parts = recorded['parts']
    robot, ri = connect(namespace=args.follower)
    names = joints_of(ri, parts)
    ri.servo_on(names)
    for i, pose_deg in enumerate(recorded['poses'], 1):
        pose = {n: float(np.deg2rad(a)) for n, a in pose_deg.items()}
        print('pose {}/{}'.format(i, len(recorded['poses'])))
        send(robot, ri, pose, args.time, parts)
        for p in parts:
            ri.wait_interpolation(PARTS[p])
    print('done; the robot holds the last pose (torque on).')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--leader', default='/jedy_leader',
                        help='namespace of the robot moved by hand')
    parser.add_argument('--follower', default='/jedy_follower',
                        help='namespace of the robot that copies it')
    parser.add_argument('--parts', default='head,rarm,larm,rhand,lhand',
                        help='comma-separated, of: {}'.format(', '.join(PARTS)))
    parser.add_argument('--approach', type=float, default=5.0,
                        help='seconds for the follower\'s first move to the '
                             'leader\'s pose')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='poses sent to the follower per second')
    parser.add_argument('--lead', type=float, default=0.3,
                        help='seconds each sent pose takes to reach; longer '
                             'is smoother and lags more')
    parser.add_argument('--record', default=None,
                        help='YAML file the Enter key appends poses to')
    parser.add_argument('--play', default=None,
                        help='play a recorded YAML file on --follower instead')
    parser.add_argument('--time', type=float, default=1.0,
                        help='--play: seconds from one pose to the next')
    args = parser.parse_args()
    if args.play:
        play(args)
        return
    parts = [p.strip() for p in args.parts.split(',') if p.strip()]
    unknown = [p for p in parts if p not in PARTS]
    if unknown:
        parser.error('unknown part(s) {}; choose from {}'.format(
            ', '.join(unknown), ', '.join(PARTS)))
    follow(args, parts)


if __name__ == '__main__':
    main()

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


class Streamer:
    """Stream poses to the follower's trajectory controllers.

    Each pose goes out as a two-point trajectory, from the pose sent last to
    the new one in one period, on the controllers' ``joint_trajectory``
    topics, so one segment ends where the next starts and the follower moves
    at a steady speed between poses. Positions only: a goal velocity of zero
    made it stop at every pose, which was visibly jerky; and segments longer
    than the period were cut off by the next one, which jumped the command
    forward. Not through ``ri.angle_vector()``: that starts every trajectory
    at the MEASURED pose, so a joint that sags behind its command (an arm
    under gravity) was pulled back to where it sagged to ten times a second
    and never reached the leader (measured: a shoulder 30 deg short for 10 s).
    """

    def __init__(self, ri, namespace, parts, start):
        from trajectory_msgs.msg import JointTrajectory
        self.parts = parts
        self.joints = {p: ri.joints_of(PARTS[p]) for p in parts}
        self.pubs = {p: ri.create_publisher(
            JointTrajectory, '{}/{}/joint_trajectory'.format(
                namespace.rstrip('/'), PARTS[p]), 10) for p in parts}
        self.last = dict(start)

    def send(self, pose, duration):
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        for p in self.parts:
            names = self.joints[p]
            msg = JointTrajectory(joint_names=names)
            msg.points = [
                JointTrajectoryPoint(positions=[self.last[n] for n in names],
                                     time_from_start=Duration(sec=0, nanosec=0)),
                JointTrajectoryPoint(positions=[pose[n] for n in names],
                                     time_from_start=Duration(
                                         sec=int(duration),
                                         nanosec=int((duration % 1) * 1e9))),
            ]
            self.pubs[p].publish(msg)
        self.last.update(pose)


def follow(args, parts):
    from jedy_skrobot import connect
    from jedy_skrobot.interface import BoardNotAnswering
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
    streamer = Streamer(follower, args.follower, parts, pose)

    recorded = []
    print('following. Enter: record the pose{}, q: quit'.format(
        '' if args.record else ' (needs --record)'))
    period = 1.0 / args.rate
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        paused = False
        while True:
            start = time.monotonic()
            try:
                # Raises while the leader's board does not answer: its
                # joint_states keep coming then, frozen at the last pose read.
                pose = pose_of(leader_robot, leader, names)
            except BoardNotAnswering:
                # The follower holds the last pose it was sent.
                if not paused:
                    print('the leader stopped answering: the follower holds '
                          'its pose until it answers again')
                    paused = True
                if read_key(0.1) == 'q':
                    break
                continue
            if paused:
                print('the leader answers again: following')
                paused = False
            streamer.send(pose, period)
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
    parser.add_argument('--rate', type=float, default=25.0,
                        help='poses sent to the follower per second; the '
                             'follower lags the leader by about one period')
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

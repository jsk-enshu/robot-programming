#!/usr/bin/env python3
"""Task 2.1: one controller at a time, or several at once, independently.

The scikit-robot version of the 2025 EusLisp
``(send *ri* :angle-vector av 2000 :head-controller)``. Each of Jedy's
trajectory controllers (``ri.controller_names``: head_controller,
right_arm_controller, left_arm_controller, right_gripper_controller,
left_gripper_controller) takes a pose of its own joints only, with its own
time, and ``ri.wait_interpolation(name)`` waits for that one alone:

    ri.angle_vector(robot.angle_vector(), 2.0,
                    controller_type='head_controller')   # only the head moves
    ri.wait_interpolation('head_controller')

    ros2 run jedy_skrobot sample_controllers --mode head
    ros2 run jedy_skrobot sample_controllers --mode rarm
    ros2 run jedy_skrobot sample_controllers --mode both    # head 2 s, arm 4 s

Each mode checks what it claims: the commanded joints reach their targets,
and the joints of every other controller stay where they were. ``both``
also prints, in robot time, when each controller finished -- the head first,
while the arm is still on its way.
"""

import argparse

import numpy as np

from jedy_skrobot import connect

#: A commanded joint must end within this of its target [deg].
REACH_DEG = 2.0
#: A joint of a controller that was not commanded may move at most this [deg].
STILL_DEG = 1.0


def robot_time(ri):
    """Seconds on the robot's clock (sim time in a simulator)."""
    return ri.get_clock().now().nanoseconds * 1e-9


def check(ri, names, before, target, commanded):
    """Compare the measured pose with the target and the start.

    Parameters
    ----------
    ri : jedy_skrobot interface
    names : list of str
        Joint names, in ``robot.joint_list`` order.
    before, target : numpy.ndarray
        [rad] The pose before the commands, and the model's pose sent.
    commanded : set of str
        The joints whose controllers were sent ``target``.

    Returns
    -------
    bool
        Whether every check passed.
    """
    after = ri.angle_vector()
    # A wheel reports no position (NaN): there is nothing to compare.
    keep = [i for i in range(len(names))
            if not (np.isnan(before[i]) or np.isnan(after[i]))]
    names = [names[i] for i in keep]
    before, target, after = before[keep], target[keep], after[keep]
    ok = True
    for name, b, t, a in zip(names, before, target, after):
        if name in commanded:
            err = np.rad2deg(abs(a - t))
            if err > REACH_DEG:
                print('  NOT REACHED {:20s} target {:7.1f} measured {:7.1f} '
                      'deg'.format(name, np.rad2deg(t), np.rad2deg(a)))
                ok = False
        else:
            moved = np.rad2deg(abs(a - b))
            if moved > STILL_DEG:
                print('  MOVED       {:20s} {:7.1f} -> {:7.1f} deg, but its '
                      'controller was not commanded'.format(
                          name, np.rad2deg(b), np.rad2deg(a)))
                ok = False
    reached = [n for n in names if n in commanded]
    print('  commanded joints: {}; worst error {:.2f} deg'.format(
        len(reached), max(np.rad2deg(abs(a - t)) for n, t, a in
                          zip(names, target, after) if n in commanded)))
    print('  other joints:     {}; largest motion {:.2f} deg'.format(
        len(names) - len(reached), max(
            [np.rad2deg(abs(a - b)) for n, b, a in zip(names, before, after)
             if n not in commanded] or [0.0])))
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['head', 'rarm', 'both'],
                   default='both')
    p.add_argument('--head-time', type=float, default=2.0)
    p.add_argument('--arm-time', type=float, default=4.0)
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on()                            # real robot: torque on, holding
    robot.reset_pose()
    ri.angle_vector(robot.angle_vector(), 3.0)
    ri.wait_interpolation()

    names = [j.name for j in robot.joint_list]
    head = set(ri.joints_of('head_controller'))
    rarm = set(ri.joints_of('right_arm_controller'))
    before = ri.angle_vector()
    robot.angle_vector(before)      # the model starts from the measured pose

    # The poses. Both go into ONE model; which joints move is decided only
    # by the controller each command is sent to.
    if args.mode in ('head', 'both'):
        robot.head_joint0.joint_angle(np.deg2rad(30.0))
        robot.head_joint1.joint_angle(np.deg2rad(20.0))
    if args.mode in ('rarm', 'both'):
        target = robot.rarm.end_coords.copy_worldcoords()
        target.translate([0.04, 0.0, 0.04], wrt='world')
        solved = robot.rarm.inverse_kinematics(target, rotation_axis=False)
        if solved is False or solved is None:
            raise SystemExit('IK did not converge for this target')
    goal = robot.angle_vector()

    commanded = set()
    if args.mode == 'head':
        ri.angle_vector(goal, args.head_time,
                        controller_type='head_controller')
        ri.wait_interpolation('head_controller')
        commanded = head
    elif args.mode == 'rarm':
        ri.angle_vector(goal, args.arm_time,
                        controller_type='right_arm_controller')
        ri.wait_interpolation('right_arm_controller')
        commanded = rarm
    else:
        # Two commands, back to back, each to its own controller with its
        # own time. Neither waits for nor cancels the other.
        start = robot_time(ri)
        ri.angle_vector(goal, args.arm_time,
                        controller_type='right_arm_controller')
        ri.angle_vector(goal, args.head_time,
                        controller_type='head_controller')
        ri.wait_interpolation('head_controller')
        head_done = robot_time(ri) - start
        # Where the arm is at the moment the head is done: on its way.
        mid = ri.angle_vector()
        arm_idx = [i for i, n in enumerate(names) if n in rarm]
        arm_left = np.rad2deg(np.max(np.abs(goal[arm_idx] - mid[arm_idx])))
        ri.wait_interpolation('right_arm_controller')
        arm_done = robot_time(ri) - start
        print('head_controller      finished at {:5.2f} s (asked {} s)'
              .format(head_done, args.head_time))
        print('  the right arm was then still {:.1f} deg from its goal'
              .format(arm_left))
        print('right_arm_controller finished at {:5.2f} s (asked {} s)'
              .format(arm_done, args.arm_time))
        commanded = head | rarm
        if not head_done < arm_done:
            raise SystemExit('the head did not finish before the arm')

    print('mode {}: sent to {}'.format(args.mode, ', '.join(
        sorted({ri.controller_of(n) for n in commanded}))))
    if not check(ri, names, before, goal, commanded):
        raise SystemExit('a controller moved joints it was not sent, or '
                         'did not reach its target')
    print('OK: each controller moved only its own joints')


if __name__ == '__main__':
    main()

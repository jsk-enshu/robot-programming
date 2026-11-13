#!/usr/bin/env python3
"""Nod Jedy's head: joint angles one by one, then wait for each motion.

The scikit-robot version of jedyeus/euslisp/sample-joint.l.

    ros2 run jedy_skrobot sample_joint --count 3
"""

import argparse

import numpy as np

from jedy_skrobot import connect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--count', type=int, default=2, help='how many nods')
    args = p.parse_args()

    robot, ri = connect()

    # The joints you can use
    print([j.name for j in robot.joint_list])

    # On the real robot the torque starts off. In a simulator the joints are
    # always driven and this does nothing.
    ri.servo_on(joints=['head_joint0', 'head_joint1'])

    # Angles in radians; np.deg2rad for degrees.
    robot.head_joint0.joint_angle(np.deg2rad(20))
    robot.head_joint1.joint_angle(np.deg2rad(30))
    ri.angle_vector(robot.angle_vector(), 3.0)
    ri.wait_interpolation()

    for _ in range(args.count):
        for tilt in (0, 30):
            robot.head_joint0.joint_angle(0.0)
            robot.head_joint1.joint_angle(np.deg2rad(tilt))
            ri.angle_vector(robot.angle_vector(), 1.0)
            ri.wait_interpolation()          # wait for the motion to end
            measured = ri.angle_vector()
            print('head_joint1 target {:6.1f} deg, measured {:6.1f} deg'.format(
                tilt, np.rad2deg(
                    measured[robot.joint_list.index(robot.head_joint1)])))


if __name__ == '__main__':
    main()

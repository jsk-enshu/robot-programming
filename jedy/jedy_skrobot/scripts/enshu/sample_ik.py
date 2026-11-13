#!/usr/bin/env python3
"""Reach with the right arm by inverse kinematics, then check where the hand is.

The scikit-robot version of the IK part of the 2025 EusLisp exercise
(``(send *jedy* :rarm :inverse-kinematics ...)``).

    ros2 run jedy_skrobot sample_ik --dx 0.03 --dz 0.03
"""

import argparse

import numpy as np

from jedy_skrobot import connect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dx', type=float, default=0.03,
                   help='move the hand forward this much [m]')
    p.add_argument('--dy', type=float, default=0.0, help='leftward [m]')
    p.add_argument('--dz', type=float, default=0.03, help='upward [m]')
    p.add_argument('--time', type=float, default=3.0)
    args = p.parse_args()

    robot, ri = connect()
    ri.servo_on()                            # real robot: torque on, holding
    robot.reset_pose()          # IK may not converge from every start
    ri.angle_vector(robot.angle_vector(), args.time)
    ri.wait_interpolation()

    # The target: the hand moved by (dx, dy, dz) in the robot's base frame.
    target = robot.rarm.end_coords.copy_worldcoords()
    target.translate([args.dx, args.dy, args.dz], wrt='world')
    solved = robot.rarm.inverse_kinematics(target, rotation_axis=False)
    if solved is False or solved is None:
        raise SystemExit('IK did not converge for this target')
    ri.angle_vector(robot.angle_vector(), args.time)
    ri.wait_interpolation()

    # Put the MEASURED pose into the model and see where the hand ended up.
    robot.angle_vector(ri.angle_vector())
    reached = robot.rarm.end_coords.worldpos()
    error = np.linalg.norm(reached - target.worldpos())
    print('target  {} m'.format(np.round(target.worldpos(), 4)))
    print('reached {} m'.format(np.round(reached, 4)))
    print('hand position error {:.1f} mm'.format(error * 1000.0))


if __name__ == '__main__':
    main()

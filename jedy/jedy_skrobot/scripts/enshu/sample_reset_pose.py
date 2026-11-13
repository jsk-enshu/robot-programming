#!/usr/bin/env python3
"""Send Jedy to its reset pose and print how close every joint got.

The scikit-robot version of jedyeus/euslisp/reset-pose.l. Runs unchanged on
Gazebo, the real robot and Isaac Sim:

    ros2 launch jedy_bringup jedy_gazebo.launch.py     # or another bring-up
    ros2 run jedy_skrobot sample_reset_pose
"""

import argparse

import numpy as np

from jedy_skrobot import connect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--time', type=float, default=4.0,
                   help='seconds to get there')
    args = p.parse_args()

    robot, ri = connect()                    # robot: model, ri: the real thing
    ri.servo_on()                            # real robot: torque on, holding
    robot.reset_pose()                       # pose the MODEL
    target = robot.angle_vector()
    ri.angle_vector(target, args.time)       # send it to the robot
    ri.wait_interpolation()                  # wait until it is there

    measured = ri.angle_vector()             # where the robot IS now
    # Only the joints a controller commands; the mimic gripper joints and the
    # wheels are not posed by an angle vector.
    names = [j for c in ri.controller_names for j in ri.joints_of(c)]
    index = [robot.joint_list.index(getattr(robot, n)) for n in names]
    print('{:20s} {:>9s} {:>9s} {:>8s}'.format(
        'joint', 'target', 'measured', 'error'))
    for name, i in zip(names, index):
        print('{:20s} {:9.2f} {:9.2f} {:8.2f}  [deg]'.format(
            name, np.rad2deg(target[i]), np.rad2deg(measured[i]),
            np.rad2deg(measured[i] - target[i])))
    worst = np.max(np.abs(measured[index] - target[index]))
    print('worst error over {} joints: {:.2f} deg'.format(
        len(index), np.rad2deg(worst)))

if __name__ == '__main__':
    main()

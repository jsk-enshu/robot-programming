#!/usr/bin/env python3
"""Where the checkerboard is, from the head camera: day 1's checkerboard_detector.

The scikit-robot version of the 2025 day-1 exercise (``checkerboard_detector``
publishing the board's TF). The board's corners are found in the colour image
(OpenCV), its pose relative to the camera comes from them and the camera's
intrinsics (``ri.camera_info()``, solvePnP), and the robot model -- with the
MEASURED joint angles -- carries it into ``base_link``:

    camera <- board         solvePnP
    base_link <- camera     robot.camera_color_optical_frame (forward kinematics)

    ros2 launch jedy_bringup jedy_gazebo.launch.py checkerboard:=true
    ros2 run jedy_skrobot checkerboard_pose
    ros2 run jedy_skrobot checkerboard_pose --expect 0.693 0 0.30  # check it

``--expect X Y Z``: where the board's centre should be in base_link [m]
(the simulators put the black squares' faces at x = 0.693 right after start);
the sample then fails if it is further than ``--tolerance``.
"""

import argparse

import cv2
import numpy as np

from jedy_skrobot import connect


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--corners', type=int, nargs=2, default=[7, 4],
                   metavar=('COLS', 'ROWS'), help='inner corners')
    p.add_argument('--square', type=float, default=0.04,
                   help='side of one square [m] (the printed 2025 board: '
                        '0.02; the simulators: 0.04)')
    p.add_argument('--expect', type=float, nargs=3, default=None,
                   metavar=('X', 'Y', 'Z'))
    p.add_argument('--tolerance', type=float, default=0.02, help='[m]')
    p.add_argument('--timeout', type=float, default=10.0)
    args = p.parse_args()

    robot, ri = connect()
    if not ri.has_camera():
        print('this robot has no head camera running (nobody publishes the '
              'colour image): nothing to do.')
        return
    ri.servo_on()
    robot.reset_pose()
    ri.angle_vector(robot.angle_vector(), 3.0)
    ri.wait_interpolation()

    # An image taken after the head stopped, and the joint angles it was
    # taken at.
    image, stamp = ri.image(timeout=args.timeout, newer_than=ri.now())
    robot.angle_vector(ri.angle_vector())
    K, distortion = ri.camera_info(timeout=args.timeout)

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    size = tuple(args.corners)
    found, corners = cv2.findChessboardCorners(gray, size)
    if not found:
        raise SystemExit('no {}x{} checkerboard in the image'.format(*size))
    corners = cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))

    # The board's own frame: corner (0, 0) at the origin, x along a row, y
    # along a column, z out of the board.
    grid = np.zeros((size[0] * size[1], 3), np.float64)
    grid[:, :2] = np.mgrid[0:size[0], 0:size[1]].T.reshape(-1, 2)
    grid *= args.square
    ok, rvec, tvec = cv2.solvePnP(grid, corners, K, distortion)
    if not ok:
        raise SystemExit('solvePnP failed')
    rotation, _ = cv2.Rodrigues(rvec)
    centre_in_camera = (rotation @ grid.mean(axis=0)) + tvec.ravel()

    camera = robot.camera_color_optical_frame.copy_worldcoords()
    centre = camera.transform_vector(centre_in_camera)
    normal = camera.rotate_vector(rotation[:, 2])
    print('found {}x{} corners (image at t={:.2f} s)'.format(
        size[0], size[1], stamp))
    print('board centre in the camera optical frame: {} m'.format(
        np.round(centre_in_camera, 3)))
    print('board centre in base_link:                {} m'.format(
        np.round(centre, 3)))
    print('board normal in base_link:                {}'.format(
        np.round(normal, 3)))
    if args.expect is not None:
        error = np.linalg.norm(centre - np.asarray(args.expect))
        print('distance from the expected {}: {:.1f} mm'.format(
            args.expect, error * 1000.0))
        if error > args.tolerance:
            raise SystemExit('further than {} m'.format(args.tolerance))


if __name__ == '__main__':
    main()

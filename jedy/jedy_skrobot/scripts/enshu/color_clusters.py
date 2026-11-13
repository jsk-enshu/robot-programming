#!/usr/bin/env python3
"""Coloured objects in 3-D: day 1's hsi_color_filter + clustering, in numpy.

The 2025 day-1 exercise ran jsk_pcl_ros' ``hsi_color_filter`` and Euclidean
clustering on the point cloud (ROS 1). Here the same idea on the head
camera's colour and depth images, with no point-cloud library:

1. keep the pixels whose hue/saturation/value are in range (OpenCV HSV:
   hue 0-179),
2. group touching pixels into clusters (connected components) and drop
   small ones,
3. turn each cluster's pixels into 3-D points with the depth image and the
   camera's intrinsics (``ri.camera_info()``), and
4. carry the cluster's centre into ``base_link`` with the robot model at the
   MEASURED joint angles.

    ros2 launch jedy_bringup jedy_gazebo.launch.py box:=true
    ros2 run jedy_skrobot color_clusters                  # red (the default)
    ros2 run jedy_skrobot color_clusters --hue 100 130    # blue

``--expect X Y``: where the largest cluster should be in base_link [m]
(the red box's centre right after start: 1.2 0.5); its visible surface is
closer than its centre, so the check is on x/y with ``--tolerance``.
"""

import argparse

import cv2
import numpy as np

from jedy_skrobot import connect


def masks(rgb, hue, saturation, value):
    """Pixels in the colour range.

    Parameters
    ----------
    rgb : numpy.ndarray
        ``(h, w, 3)`` uint8.
    hue : tuple of int
        OpenCV hue range (0-179). ``lo > hi`` wraps around (red: 170 10).
    saturation, value : int
        Lower bounds (0-255).

    Returns
    -------
    numpy.ndarray
        ``(h, w)`` bool.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    lo, hi = hue
    in_hue = (h >= lo) & (h <= hi) if lo <= hi else (h >= lo) | (h <= hi)
    return in_hue & (s >= saturation) & (v >= value)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hue', type=int, nargs=2, default=[170, 10],
                   metavar=('LO', 'HI'))
    p.add_argument('--saturation', type=int, default=120)
    p.add_argument('--value', type=int, default=50)
    p.add_argument('--min-pixels', type=int, default=200)
    p.add_argument('--expect', type=float, nargs=2, default=None,
                   metavar=('X', 'Y'))
    p.add_argument('--tolerance', type=float, default=0.15, help='[m]')
    p.add_argument('--timeout', type=float, default=10.0)
    args = p.parse_args()

    robot, ri = connect()
    if not ri.has_camera():
        print('this robot has no head camera running (nobody publishes the '
              'colour image): nothing to do.')
        return
    after = ri.now()
    rgb, _ = ri.image(timeout=args.timeout, newer_than=after)
    depth, _ = ri.depth_image(timeout=args.timeout, newer_than=after)
    robot.angle_vector(ri.angle_vector())
    K, _ = ri.camera_info(timeout=args.timeout)
    if depth.shape != rgb.shape[:2]:
        raise SystemExit('depth {} and colour {} are not the same size: this '
                         'sample assumes an aligned depth image'.format(
                             depth.shape, rgb.shape[:2]))

    mask = masks(rgb, args.hue, args.saturation, args.value)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    camera = robot.camera_color_optical_frame.copy_worldcoords()
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    clusters = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < args.min_pixels:
            continue
        v, u = np.nonzero(labels == label)
        z = depth[v, u]
        good = np.isfinite(z) & (z > 0)
        if not np.any(good):
            continue
        z, u, v = z[good], u[good], v[good]
        points = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
        centre = camera.transform_vector(np.median(points, axis=0))
        clusters.append((len(z), centre))
    clusters.sort(key=lambda c: -c[0])
    print('{} pixel(s) in hue {}..{}, {} cluster(s) of at least {} pixels'
          .format(int(mask.sum()), args.hue[0], args.hue[1], len(clusters),
                  args.min_pixels))
    for n, centre in clusters:
        print('  {:6d} px  centre in base_link {} m'.format(
            n, np.round(centre, 3)))
    if args.expect is not None:
        if not clusters:
            raise SystemExit('no cluster to compare with {}'.format(
                args.expect))
        error = np.linalg.norm(clusters[0][1][:2] - np.asarray(args.expect))
        print('largest cluster is {:.3f} m from the expected {} (x, y)'
              .format(error, args.expect))
        if error > args.tolerance:
            raise SystemExit('further than {} m'.format(args.tolerance))


if __name__ == '__main__':
    main()

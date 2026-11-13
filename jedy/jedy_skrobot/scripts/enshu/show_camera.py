#!/usr/bin/env python3
"""Show the head camera's colour and depth images with matplotlib.

    ros2 run jedy_skrobot show_camera                      # keeps updating; close the window to stop
    ros2 run jedy_skrobot show_camera --once --save a.png  # one image, saved to a file

The images come from ``ri.image()`` / ``ri.depth_image()`` as numpy arrays, so
any Python code can work on them. ``recognize()`` below is the place for your
own recognizer: it gets the images and returns boxes, which are drawn on the
colour image. As it stands it finds red regions with OpenCV.
"""

import argparse
import warnings

import cv2
# The venv's matplotlib also finds the system's mpl_toolkits and warns that 3-D
# plotting is unavailable; it is not used here.
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from jedy_skrobot import connect  # noqa: E402


def recognize(rgb, depth):
    """Find things in the image: here, red regions.

    Replace this with your own recognizer.

    Parameters
    ----------
    rgb : numpy.ndarray
        ``(H, W, 3)`` uint8, RGB (``ri.image()``).
    depth : numpy.ndarray or None
        ``(H, W)`` float32 [m], NaN where there is no reading
        (``ri.depth_image()``); None without a depth camera.

    Returns
    -------
    list of tuple
        ``(name, (x1, y1, x2, y2))``, boxes in pixels.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    # Red wraps around the hue circle (0..10 and 170..180 in OpenCV's 0..180).
    mask = (cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
            | cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    found = []
    for x, y, w, h, area in stats[1:]:     # label 0 is the background
        if area < 200:                     # drop specks [px]
            continue
        name = 'red'
        if depth is not None:
            patch = depth[y:y + h, x:x + w]
            if np.isfinite(patch).any():
                name = 'red {:.2f} m'.format(float(np.nanmedian(patch)))
        found.append((name, (int(x), int(y), int(x + w), int(y + h))))
    return found


def draw(ax_rgb, ax_depth, rgb, depth, found):
    """Draw the images and the boxes."""
    ax_rgb.clear()
    ax_rgb.imshow(rgb)
    for name, (x1, y1, x2, y2) in found:
        ax_rgb.add_patch(patches.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                           fill=False, edgecolor='lime', linewidth=2))
        ax_rgb.text(x1, y1 - 4, name, color='lime', fontsize=9)
    ax_rgb.set_title('colour: {} found'.format(len(found)))
    ax_rgb.axis('off')
    ax_depth.clear()
    if depth is not None:
        cmap = plt.get_cmap('viridis').copy()
        cmap.set_bad('black')                      # NaN: no reading
        shown = ax_depth.imshow(depth, cmap=cmap)
        if not hasattr(ax_depth, 'jedy_colorbar'):
            ax_depth.jedy_colorbar = plt.colorbar(shown, ax=ax_depth, label='[m]')
        ax_depth.set_title('depth (black: no reading)')
    else:
        ax_depth.set_title('no depth camera')
    ax_depth.axis('off')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--once', action='store_true', help='show one image and stop')
    p.add_argument('--save', help='also save the figure to this file')
    args = p.parse_args()

    robot, ri = connect()
    fig, (ax_rgb, ax_depth) = plt.subplots(1, 2, figsize=(11, 4.5))
    stamp = None
    while True:
        rgb, stamp = ri.image(newer_than=stamp)    # wait for an image newer than the last
        try:
            depth, _ = ri.depth_image()
        except RuntimeError:                       # no depth camera running
            depth = None
        found = recognize(rgb, depth)
        draw(ax_rgb, ax_depth, rgb, depth, found)
        if args.save:
            fig.savefig(args.save)
        if args.once:
            print('found:', found)
            if not args.save:
                plt.show()
            return
        plt.pause(0.05)                            # draw, and let the window react
        if not plt.fignum_exists(fig.number):      # the window was closed
            return


if __name__ == '__main__':
    main()
